"""Portable planning functions. NumPy only; never sends robot commands.

An accepted grasp means sampled geometric feasibility, not successful contact
dynamics or a safe navigation path. Object XYZ is the desired TCP grasp center.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import numpy as np

from .model import RobotModel, wrap


@dataclass(frozen=True)
class GraspPolicy:
    pitch_rad: float = math.pi/2
    approach_height_m: float = .060
    lift_height_m: float = .060
    cartesian_step_m: float = .005
    joint_step_rad: float = math.radians(2)
    joint_margin_rad: float = math.radians(2)
    collision_margin_m: float = 0.
    floor_margin_m: float = .003
    object_width_m: float = .030
    floor_z_m: float = 0.

    def validate(self):
        values = np.array(list(asdict(self).values()))
        if not np.isfinite(values).all(): raise ValueError("Policy values must be finite")
        if min(self.approach_height_m,self.lift_height_m,self.cartesian_step_m,self.joint_step_rad,self.object_width_m) <= 0:
            raise ValueError("Heights, steps and object width must be positive")
        if min(self.joint_margin_rad,self.collision_margin_m,self.floor_margin_m) < 0: raise ValueError("Margins must be nonnegative")
        # This implementation specifically certifies a vertical approach corridor.
        if abs(self.pitch_rad-math.pi/2) > 1.e-9:
            raise ValueError("Grasp policy currently requires tool +X pointing vertically down (pitch=pi/2)")


def _select_continuous_branch(model, point, policy, obstacles=()):
    """Every analytic branch is evaluated independently; no elbow switching."""
    height = max(policy.approach_height_m,policy.lift_height_m)
    offsets = np.unique(np.r_[np.linspace(0,height,max(2,math.ceil(height/policy.cartesian_step_m)+1)),policy.approach_height_m,policy.lift_height_m])
    points = np.tile(point,(len(offsets),1)); points[:,2] += offsets
    paths, ok = model.ik(points,policy.pitch_rad,policy.joint_margin_rad)
    opening = model.gripper_limits[1]
    radius = policy.object_width_m/2
    # The sphere is a conservative width envelope, not a banana mesh/contact model.
    best = None
    for branch in range(4):
        if not ok[:,branch].all(): continue
        q = paths[:,branch,:]
        if np.any(np.abs(np.diff(q,axis=0)) > np.pi): continue
        # Insert joint interpolation samples to also check between Cartesian waypoints.
        dense, tcp_expected = [q[0]], [points[0]]
        for i in range(len(q)-1):
            steps = max(1,math.ceil(np.max(np.abs(q[i+1]-q[i]))/policy.joint_step_rad))
            for t in np.linspace(0,1,steps+1)[1:]:
                dense.append(q[i]*(1-t)+q[i+1]*t)
                tcp_expected.append(points[i]*(1-t)+points[i+1]*t)
        dense = np.asarray(dense)
        clear, floors = model.screen(dense,opening,policy.collision_margin_m,policy.floor_z_m,
                                     (point,radius,False),obstacles,floor_margin=policy.floor_margin_m)
        if not clear.all(): continue
        # Stay close to the vertical corridor even during joint interpolation.
        tcp = model.fk(dense)["end_effector_link"][:,:3,3]
        corridor_error = float(np.max(np.linalg.norm(tcp-np.asarray(tcp_expected),axis=1)))
        if corridor_error > .001: continue
        # Close to conservative object width, then lift with closed fingers.
        closing = (policy.object_width_m-model.zero_gap)/2
        if not model.gripper_limits[0] <= closing <= opening: continue
        closing_samples = np.linspace(opening,closing,9)
        close_ok, close_floor = model.screen(np.repeat(q[:1],len(closing_samples),axis=0),closing_samples,
                                             policy.collision_margin_m,policy.floor_z_m,(point,radius,True),obstacles,floor_margin=policy.floor_margin_m)
        # Held object travels with TCP during lift; retain robot-body / object tests.
        lift_ok, lift_floor = model.screen(dense,closing,policy.collision_margin_m,policy.floor_z_m,
                                           (tcp,radius,True),obstacles,floor_margin=policy.floor_margin_m)
        if not close_ok.all() or not lift_ok.all(): continue
        # Collision of the carried sphere with the floor/known obstacles along lift.
        if np.min(tcp[:,2])-radius < policy.floor_z_m: continue
        for center, rotation, half in obstacles:
            local = (tcp-center) @ rotation
            if np.any(np.linalg.norm(np.maximum(np.abs(local)-half,0),axis=1) < radius+policy.collision_margin_m):
                break
        else:
            quality = float(np.min(model.joint_quality(dense)))
            min_floor = float(min(floors.min(),close_floor.min(),lift_floor.min()))
            self_clearance = model.keyframe_self_clearance(q[[0,-1]])
            clearance_score = 0. if self_clearance is None else min(1.,max(0.,self_clearance["distance_m"])/.01)
            quality_score = .65*quality+.2*min(1.,min_floor/.05)+.15*clearance_score
            if best is None or quality_score > best["quality_score"]:
                pre_i = int(np.argmin(np.abs(offsets-policy.approach_height_m)))
                lift_i = int(np.argmin(np.abs(offsets-policy.lift_height_m)))
                best = {"branch":branch,"q_grasp_rad":q[0].tolist(),"q_pregrasp_rad":q[pre_i].tolist(),
                        "q_lift_rad":q[lift_i].tolist(),"joint_limit_margin_score":quality,
                        "quality_score":quality_score,"min_floor_clearance_m":min_floor,
                        "self_clearance_at_keyframes":self_clearance,
                        "interpolation_tcp_error_m":corridor_error,"opening_m":opening,"closing_m":float(closing),
                        "checked_joint_samples":len(dense),"corridor_offsets_m":offsets.tolist(),
                        "corridor_q_rad":q.tolist()}
    return best


def check_grasp(model, object_base_xyz, policy=None, obstacles=()):
    """Return a sampled pregrasp/descent/close/lift solution or None.

    Input is the desired grasp-center in base_footprint, not raw camera XYZ.
    `obstacles` are OBBs in that same base frame. No stochastic probability is claimed.
    """
    policy = policy or GraspPolicy(); policy.validate()
    point = np.asarray(object_base_xyz,dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all(): raise ValueError("object_base_xyz must be finite XYZ")
    if policy.object_width_m+2*policy.collision_margin_m > model.zero_gap+2*model.gripper_limits[1]:
        return None
    if point[2]-policy.object_width_m/2 < policy.floor_z_m: return None
    return _select_continuous_branch(model,point,policy,obstacles)


def transform_object_to_base(object_world_xyz, base_xyyaw, base_z=0.):
    obj, pose = np.asarray(object_world_xyz,float), np.asarray(base_xyyaw,float)
    if obj.shape != (3,) or pose.shape != (3,) or not np.isfinite(np.r_[obj,pose,base_z]).all():
        raise ValueError("Expected finite object XYZ and base x,y,yaw")
    x,y,yaw = pose; c,s = math.cos(yaw),math.sin(yaw)
    d = obj[:2]-[x,y]
    return np.array([c*d[0]+s*d[1],-s*d[0]+c*d[1],obj[2]-base_z])


def boxes_in_base(world_boxes, base_xyyaw, base_z=0.):
    """World AABB {center:[x,y,z], size:[dx,dy,dz]} -> base-frame OBB."""
    yaw = float(base_xyyaw[2]); c,s = math.cos(yaw),math.sin(yaw)
    rotation = np.array([[c,s,0],[-s,c,0],[0,0,1]])
    boxes = []
    for b in world_boxes:
        center,size = np.asarray(b["center"],float),np.asarray(b["size"],float)
        if center.shape != (3,) or size.shape != (3,) or not np.isfinite(np.r_[center,size]).all() or np.any(size <= 0):
            raise ValueError("Obstacle center/size must be finite XYZ and positive sizes")
        boxes.append((transform_object_to_base(center,base_xyyaw,base_z),rotation,size/2))
    return boxes


def plan_base_positions(object_world_xyz, current_base_xyyaw, model=None, policy=None,
                        frame="odom", base_z=0., radii=None, azimuth_samples=24,
                        yaw_samples=24, max_translation_m=2., obstacles_world=None,
                        desired_closing_yaw_world=None, closing_yaw_tolerance_rad=math.radians(15),
                        top_k=20):
    """Compute base goals without ROS/Isaac dependencies or motion commands.

    Input object and base must ALREADY share one odom/map frame and timestamp.
    Search exploits arm axial symmetry: certify radial grasp branches once,
    transform them to base goals, then recheck every retained goal + obstacle set.
    Scores are geometric heuristics, never calibrated success probabilities.
    """
    model = model or RobotModel(); policy = policy or GraspPolicy(); policy.validate()
    obj, current = np.asarray(object_world_xyz,float), np.asarray(current_base_xyyaw,float)
    transform_object_to_base(obj,current,base_z)  # validate inputs
    if not frame or frame in ("camera_link","camera_color_optical_frame","camera_rgb_optical_frame"):
        raise ValueError("Transform camera observations into a shared odom/map frame first")
    if min(azimuth_samples,yaw_samples,top_k) < 1 or max_translation_m < 0:
        raise ValueError("Invalid search resolution/translation limit")
    if abs(policy.floor_z_m) > 1.e-10:
        raise ValueError("Base planner assumes the URDF base_footprint lies on floor z=base_z; use floor_z_m=0")
    radii = np.arange(.04,.341,.01) if radii is None else np.asarray(radii,float)
    if radii.ndim != 1 or not np.isfinite(radii).all() or np.any(radii <= 0): raise ValueError("radii must be finite positive 1D values")
    if desired_closing_yaw_world is not None and not np.isfinite(desired_closing_yaw_world): raise ValueError("Invalid closing yaw")
    if not np.isfinite(closing_yaw_tolerance_rad) or closing_yaw_tolerance_rad < 0: raise ValueError("Invalid closing yaw tolerance")
    obstacles_world = [] if obstacles_world is None else obstacles_world
    boxes_in_base(obstacles_world,current,base_z)  # validate even if no candidates
    current_local = transform_object_to_base(obj,current,base_z)
    current_solution = check_grasp(model,current_local,policy,boxes_in_base(obstacles_world,current,base_z))
    candidates = []

    def add(pose, point, solution):
        yaw_tool = float(wrap(pose[2]+solution["q_grasp_rad"][0]))
        closing_yaw = float(wrap(yaw_tool+math.pi/2))
        if desired_closing_yaw_world is not None:
            error = abs(float(wrap(2*(closing_yaw-desired_closing_yaw_world))))/2
            if error > closing_yaw_tolerance_rad: return
        distance = float(np.linalg.norm(pose[:2]-current[:2]))
        if distance > max_translation_m+1.e-9: return
        turn = abs(float(wrap(pose[2]-current[2])))
        if distance < 1.e-8:
            first_turn, second_turn, drive_sign = 0.,turn,1
        else:
            bearing = math.atan2(pose[1]-current[1],pose[0]-current[0])
            options = []
            for sign in (1,-1):
                heading = bearing+(0 if sign==1 else math.pi)
                a,b = abs(float(wrap(heading-current[2]))),abs(float(wrap(pose[2]-heading)))
                options.append((a+b,a,b,sign))
            _,first_turn,second_turn,drive_sign = min(options)
        route_turn = first_turn+second_turn
        cost = distance+.08*turn+.04*route_turn+.10*(1-solution["quality_score"])
        candidates.append({"base_x_m":float(pose[0]),"base_y_m":float(pose[1]),"base_yaw_rad":float(wrap(pose[2])),
                           "object_base_footprint_m":point.tolist(),"distance_m":distance,"final_yaw_change_rad":turn,
                           "differential_drive_turn_sum_rad":route_turn,"suggested_drive_sign":drive_sign,
                           "closing_yaw_world_rad":closing_yaw,"score_cost":cost,**solution})
    if current_solution is not None: add(current,current_local,current_solution)
    # Include exact present relative angles in addition to grid to avoid needless rotation.
    yaws = np.unique(np.r_[wrap(current[2]+np.linspace(-np.pi,np.pi,yaw_samples,endpoint=False)),current[2]])
    azimuths = np.linspace(-math.pi,math.pi,azimuth_samples,endpoint=False)
    for r in radii:
        for az in azimuths:
            point = model.shoulder+np.array([r*math.cos(az),r*math.sin(az),0.])
            point[2] = obj[2]-base_z
            # Check robot self/floor first; obstacle check must use the goal transform.
            solution = check_grasp(model,point,policy)
            if solution is None: continue
            for yaw in yaws:
                c,s = math.cos(yaw),math.sin(yaw)
                xy = obj[:2]-np.array([[c,-s],[s,c]])@point[:2]
                pose = np.r_[xy,yaw]
                if np.linalg.norm(xy-current[:2]) > max_translation_m: continue
                add(pose,point,solution)
    candidates.sort(key=lambda c:c["score_cost"])
    selected = []
    for candidate in candidates:
        pose = np.array([candidate["base_x_m"],candidate["base_y_m"],candidate["base_yaw_rad"]])
        if any(np.linalg.norm(pose[:2]-np.array([a["base_x_m"],a["base_y_m"]])) < .01
               and abs(float(wrap(pose[2]-a["base_yaw_rad"]))) < math.radians(5) for a in selected): continue
        checked = check_grasp(model,candidate["object_base_footprint_m"],policy,boxes_in_base(obstacles_world,pose,base_z))
        if checked is None: continue
        selected.append(candidate)
        if len(selected) == top_k: break
    return {"schema_version":1,"frame_id":frame,"object_xyz_m":obj.tolist(),"current_base_xyyaw":current.tolist(),
            "base_footprint_z_world_m":base_z,"robot_model_sha256":model.sha256,"collision_backend":model.collision_backend,"policy":asdict(policy),
            "status":"feasible_candidates" if selected else "no_feasible_candidate_in_sampled_search",
            "best":selected[0] if selected else None,"candidates":selected,"evaluated_feasible_goals":len(candidates),
            "navigation_collision_free":False,"navigation_path_checked":False,"arm_from_current_to_pregrasp_checked":False,
            "continuous_collision_checked":False,"contact_dynamics_validated":False,
            "known_obstacle_boxes_checked":len(obstacles_world),"score_is_success_probability":False,
            "warnings":["Candidate poses are planning outputs only; no robot was moved.",
                        "No navigation path or continuous collision proof is provided, even when obstacle boxes are supplied.",
                        "Grasp feasibility uses enclosing collision convex hulls/exact primitives (or conservative OBB fallback) and a spherical width envelope; banana shape/friction/force are unmodelled.",
                        "Default self-collision padding is zero (original geometry); floor clearance is 3 mm. Increase padding explicitly to model uncertainty; no automatic relaxation occurs.",
                        "Validate camera-to-odom TF, timestamps, TCP calibration, stowed-arm base motion, and current-arm-to-pregrasp path separately."]}
