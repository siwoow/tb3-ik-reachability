"""Import the resolved original robot into Isaac Lab and validate PhysX FK/limits.

Simulation only, no ROS extension, controller connection, or real robot command.
"""
import argparse
import json
from pathlib import Path

from isaaclab.app import AppLauncher

p = argparse.ArgumentParser()
p.add_argument("--samples",type=int,default=32)
p.add_argument("--show-workspace",action="store_true")
p.add_argument("--hold",action="store_true",help="Keep GUI open after validation")
AppLauncher.add_app_launcher_args(p)
args = p.parse_args()
launcher = AppLauncher(args)
app = launcher.app

import numpy as np
import torch
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import isaaclab.sim as sim_utils
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from tb3_grasp.model import ROOT, RobotModel, ARM


def main():
    model = RobotModel()
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(device=args.device,dt=1/120))
    sim.set_camera_view([.9,-.9,.8],[-.02,0,.18])
    converter = UrdfConverter(UrdfConverterCfg(
        asset_path=str(model.urdf),usd_dir=str(ROOT/"assets/usd"),usd_file_name="robot.usd",
        fix_base=True,merge_fixed_joints=False,force_usd_conversion=True,
        self_collision=True,collision_from_visuals=False,collider_type="convex_hull",
        convert_mimic_joints_to_normal_joints=False,
        joint_drive=UrdfConverterCfg.JointDriveCfg(target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=50.,damping=2.))))
    cfg = ArticulationCfg(prim_path="/World/Robot",spawn=sim_utils.UsdFileCfg(usd_path=converter.usd_path),
                          init_state=ArticulationCfg.InitialStateCfg(joint_pos={".*":0.}),
                          actuators={"imported":ImplicitActuatorCfg(joint_names_expr=[".*"],stiffness=None,damping=None)})
    robot = Articulation(cfg)
    sim_utils.GroundPlaneCfg().func("/World/Ground",sim_utils.GroundPlaneCfg())
    light = sim_utils.DomeLightCfg(intensity=2000.)
    light.func("/World/Light",light)
    sim.reset(); robot.update(sim.get_physics_dt())
    stage = sim.stage
    meshes, collision_prims, unresolved = [], [], []
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh):
            meshes.append(str(prim.GetPath()))
            if len(UsdGeom.Mesh(prim).GetPointsAttr().Get() or []) == 0: unresolved.append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.CollisionAPI): collision_prims.append(str(prim.GetPath()))
    # Instanceable geometry can live below prototypes instead of ordinary Traverse.
    for prototype in stage.GetPrototypes():
        for prim in Usd.PrimRange(prototype):
            if prim.IsA(UsdGeom.Mesh):
                meshes.append(str(prim.GetPath()))
                if len(UsdGeom.Mesh(prim).GetPointsAttr().Get() or []) == 0: unresolved.append(str(prim.GetPath()))
    robot_colliders = [path for path in collision_prims if path.startswith('/World/Robot/')]
    ids = [robot.joint_names.index(n) for n in ARM]
    limits = robot.data.joint_pos_limits[0,ids].cpu().numpy()
    if not np.allclose(limits,np.column_stack((model.lower,model.upper)),atol=2.e-6):
        raise RuntimeError(f"Imported limits differ from URDF: {limits}")
    link_id = robot.body_names.index("link5")
    rng = np.random.default_rng(42)
    qs = model.lower+.05+(model.upper-model.lower-.1)*rng.random((args.samples,4))
    errors, orientation_errors, mimic_errors = [], [], []
    from scipy.spatial.transform import Rotation
    for q in qs:
        positions = robot.data.default_joint_pos.clone()
        positions[:,ids] = torch.tensor(q,dtype=positions.dtype,device=sim.device)
        for name in ("gripper_left_joint","gripper_right_joint"):
            if name in robot.joint_names: positions[:,robot.joint_names.index(name)] = .019
        robot.write_joint_state_to_sim(positions,torch.zeros_like(positions))
        sim.forward(); robot.update(sim.get_physics_dt())
        body = robot.data.body_link_pose_w[0,link_id].cpu().numpy()
        expected = model.fk([q])["link5"][0]
        errors.append(float(np.linalg.norm(body[:3]-expected[:3,3])))
        actual_r = Rotation.from_quat(body[[4,5,6,3]]).as_matrix()
        orientation_errors.append(float(Rotation.from_matrix(actual_r.T@expected[:3,:3]).magnitude()))
        if all(n in robot.joint_names for n in ("gripper_left_joint","gripper_right_joint")):
            jpos = robot.data.joint_pos[0].cpu().numpy()
            mimic_errors.append(float(abs(jpos[robot.joint_names.index("gripper_left_joint")]-jpos[robot.joint_names.index("gripper_right_joint")])))
    report = {"isaac_lab_loaded":True,"usd_path":converter.usd_path,"urdf_sha256":model.sha256,
              "joint_names":robot.joint_names,"body_names":robot.body_names,"arm_limits_rad":limits.tolist(),
              "joint_velocity_limits":robot.data.joint_vel_limits[0].cpu().tolist(),
              "joint_effort_limits":robot.data.joint_effort_limits[0].cpu().tolist(),
              "mesh_prims":len(meshes),"collision_prims":len(collision_prims),"robot_collider_prims":robot_colliders,"empty_mesh_prims":unresolved,
              "fk_samples":len(qs),"max_fk_position_error_m":max(errors),"max_fk_orientation_error_rad":max(orientation_errors),
              "max_authored_gripper_symmetry_error_m":max(mimic_errors,default=0.),
              "mimic_note":"URDF mimic retained; paired values authored for static FK. Dynamic mimic/contact not validated.",
              "importer_warning_note":"Empty visual-reference warnings on links with no URDF visual geometry are expected. Mesh completeness and all 14 robot colliders are checked explicitly.",
              "self_collisions_enabled":True,"physics_grasp_success_validated":False,"real_robot_commands_sent":False}
    report["validation_passed"] = bool(max(errors) < 1.e-4 and max(orientation_errors) < 1.e-4 and not unresolved and meshes and len(robot_colliders) >= len(model.shapes))
    (ROOT/"results/isaac_validation.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    # A known safe task pose, if an analysis run is available, is better for viewing.
    samples_file = ROOT/"results/grasp_workspace.csv"
    if args.show_workspace and samples_file.exists():
        points = np.genfromtxt(samples_file,delimiter=",",names=True)
        points = np.atleast_1d(points)
        xyz = np.column_stack((points['x_m'],points['y_m'],points['z_m']))
        marker = UsdGeom.Points.Define(stage,"/World/GraspWorkspace")
        marker.CreatePointsAttr([Gf.Vec3f(*x) for x in xyz])
        marker.CreateWidthsAttr([.006]*len(xyz))
        marker.CreateDisplayColorAttr([Gf.Vec3f(.05,.85,.4)])
        if len(points):
            q = np.array([points[n][0] for n in ("q1_rad","q2_rad","q3_rad","q4_rad")])
            positions[:,ids] = torch.tensor(q,dtype=positions.dtype,device=sim.device)
            robot.write_joint_state_to_sim(positions,torch.zeros_like(positions)); sim.forward()
    if args.hold:
        while app.is_running(): sim.render()
    if not report["validation_passed"]: raise RuntimeError("Isaac import/FK validation failed; see report")


try:
    main()
finally:
    # This validation worker has no recording workflows to flush.
    app.close(wait_for_replicator=False,skip_cleanup=bool(args.headless))
