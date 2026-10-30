"""Use offline candidates as hints; retain actual executor IK as final authority."""

import json
from pathlib import Path
import subprocess
import sys
from functools import lru_cache


@lru_cache(maxsize=2)
def load_model(root):
    """Load only NumPy/FCL runtime; no Isaac, mesh loader or plotting imports."""
    root = Path(root).expanduser().resolve()
    sys.path.insert(0, str(root))
    from tb3_grasp.model import RobotModel
    model = RobotModel()
    if not model.fcl_shapes:
        raise RuntimeError('FCL is required; do not silently switch collision backends')
    return model


def native_query(executable, xyz_link1, floor_link1, joints, min_body_depth):
    """Call a read-only binary built from verified original candidate headers."""
    values = [*xyz_link1, floor_link1, *joints, min_body_depth]
    completed = subprocess.run([str(executable), *[format(float(v), '.17g') for v in values]],
                               check=True, capture_output=True, text=True, timeout=3)
    return json.loads(completed.stdout)


def plan_worker(root, target, current, base_z, forward_offset, object_width, max_distance):
    """Run in a child process while ROS sensor callbacks/watchdog keep running.

    Offline targets are TCP centers; actual observations are surfaces. Use a
    10 mm lower proxy, retaining the live target unchanged for the real service.
    Forward offset is applied ONCE as the original executor does in link1 +X.
    This offset depends on proposed base yaw, so evaluate each proposed yaw.
    The proxy is explicitly a hint; native and live IK both must pass at arrival.
    """
    import math
    import numpy as np
    model = load_model(root)
    from tb3_grasp.planner import GraspPolicy, plan_base_positions, transform_object_to_base
    policy = GraspPolicy(object_width_m=object_width, floor_margin_m=.006)
    # First use current yaw to seed candidates; refine each inverse placement for
    # its yaw. A forward offset along link1 is constant in base coordinates.
    proxy = np.array(target, float)
    proxy[:2] += forward_offset * np.array([math.cos(current[2]), math.sin(current[2])])
    proxy[2] -= .010
    result = plan_base_positions(proxy, current, model=model, policy=policy, base_z=base_z,
                                 max_translation_m=max_distance, top_k=40,
                                 azimuth_samples=24, yaw_samples=24)
    for candidate in result['candidates']:
        yaw = candidate['base_yaw_rad']
        correction = forward_offset * (np.array([math.cos(yaw), math.sin(yaw)]) -
                                       np.array([math.cos(current[2]), math.sin(current[2])]))
        candidate['base_x_m'] += float(correction[0])
        candidate['base_y_m'] += float(correction[1])
        old_distance = candidate['distance_m']
        candidate['distance_m'] = math.hypot(candidate['base_x_m'] - current[0], candidate['base_y_m'] - current[1])
        candidate['score_cost'] += candidate['distance_m'] - old_distance
        pose = [candidate['base_x_m'], candidate['base_y_m'], yaw]
        candidate['live_observation_at_candidate_base_m'] = transform_object_to_base(target, pose, base_z).tolist()
    result['surface_proxy_depth_m'] = .010
    result['actual_executor_forward_offset_m'] = forward_offset
    result['actual_ik_equivalent'] = False
    result['warnings'] = [w for w in result['warnings'] if 'floor clearance is 3 mm' not in w]
    result['warnings'].append('Bridge seeds require 6 mm floor clearance; live executor limits and floor checks remain authoritative.')
    result['candidates'] = sorted([c for c in result['candidates'] if c['distance_m'] <= max_distance], key=lambda c: c['score_cost'])
    result['best'] = result['candidates'][0] if result['candidates'] else None
    return result


def screen_native_path(model, plan, current_joints, opening, floor_z=0.):
    """Supplement, never replace, native limits/finger-clearance checks with FCL."""
    import numpy as np
    if not plan.get('feasible'):
        return False
    previous = np.array(current_joints, float)
    samples = []
    for goal in [plan['pregrasp'], *plan['descent'], *plan['lift']]:
        goal = np.array(goal, float)
        for fraction in np.linspace(0, 1, 101):
            samples.append(previous + fraction * (goal - previous))
        previous = goal
    samples = np.array(samples)
    for width in (opening, model.gripper_limits[1]):
        # The offline OBB floor bound is very loose for the executor's oblique
        # fingers. Test full convex hull support against the REAL floor below;
        # use screen here for unchanged joint limits and self collisions only.
        valid, _ = model.screen(samples, opening=width, floor_z=-1e6, floor_margin=.006)
        if not valid.all() or np.min(tight_floor_clearance(model, samples, width)-floor_z) < .006:
            return False
    return True


def tight_floor_clearance(model, samples, opening):
    """Exact downward support of enclosing hulls/primitives, in base_footprint."""
    import numpy as np
    cache = json.loads(model.urdf.with_name('collision_bounds.json').read_text())
    poses = model.fk(samples, opening)
    clear = np.full(len(samples), np.inf)
    for shape in cache['shapes']:
        if shape['link'] not in model.dynamic:
            continue
        t = poses[shape['link']] @ np.asarray(shape['origin'])
        r = t[:, :3, :3]
        if shape['kind'] == 'mesh':
            vertices = np.asarray(shape['convex']['vertices'])
            lowest = (r[:, 2, :] @ vertices.T).min(axis=1) + t[:, 2, 3]
        elif shape['kind'] == 'cylinder':
            extent = shape['radius']*np.linalg.norm(r[:, 2, :2], axis=1) + shape['length']/2*np.abs(r[:, 2, 2])
            lowest = t[:, 2, 3] - extent
        elif shape['kind'] == 'sphere':
            lowest = t[:, 2, 3] - shape['radius']
        else:
            lowest = t[:, 2, 3] + r[:, 2, :] @ np.asarray(shape['center']) - np.abs(r[:, 2, :]) @ np.asarray(shape['half'])
        clear = np.minimum(clear, lowest)
    return clear


def native_screen_worker(root, executable, local, floor_link1, q, opening, min_depth, floor_base=0.):
    """Keep native/FCL computation outside ROS sensor callback processing."""
    model = load_model(root)
    plan = native_query(executable, local, floor_link1, q, min_depth)
    return plan if screen_native_path(model, plan, q, opening, floor_z=floor_base) else None


def refine_goal_seeds(plan, current, max_distance):
    """Explore bounded stand-off variants for the executor's oblique approach.

    The vertical workspace is a seed, not a requirement imposed on the different
    native pitch planner. Every returned pose still requires native/FCL testing.
    """
    import math
    unique = {}
    for candidate in plan['candidates']:
        yaw = candidate['base_yaw_rad']
        for retreat in (0., .02, .04, .06, .08, .10):
            x = candidate['base_x_m'] - retreat*math.cos(yaw)
            y = candidate['base_y_m'] - retreat*math.sin(yaw)
            distance = math.hypot(x-current[0], y-current[1])
            if distance > max_distance:
                continue
            angle = abs(math.atan2(math.sin(yaw-current[2]), math.cos(yaw-current[2])))
            key = (round(x, 3), round(y, 3), round(yaw, 3))
            unique[key] = {'goal': [x, y, yaw], 'cost': distance+.08*angle+.1*(1-candidate['quality_score']),
                           'retreat_from_offline_seed_m': retreat}
    return sorted(unique.values(), key=lambda c: c['cost'])[:60]
