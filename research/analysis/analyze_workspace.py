"""Calculate reachable / top-down graspable samples and save scientific figures."""
import argparse
from dataclasses import asdict
import json
import math
import time

import numpy as np
from scipy.stats import qmc
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from tb3_grasp.model import ROOT, RobotModel, ARM
from tb3_grasp.planner import GraspPolicy, check_grasp


def save_csv(path, data, columns):
    np.savetxt(path,np.asarray(data).reshape(-1,len(columns)),delimiter=",",header=",".join(columns),comments="",fmt="%.10g")


def bounds(points):
    return None if not len(points) else {"min_xyz_m":points.min(axis=0).tolist(),"max_xyz_m":points.max(axis=0).tolist()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--samples-power",type=int,default=15,help="2**N joint-space Sobol samples")
    p.add_argument("--grid-step",type=float,default=.01)
    p.add_argument("--object-width",type=float,default=.030)
    p.add_argument("--approach-height",type=float,default=.060)
    p.add_argument("--collision-margin",type=float,default=0.)
    p.add_argument("--output",default=str(ROOT/"results"))
    args = p.parse_args()
    if not 1 <= args.samples_power <= 21 or args.grid_step <= 0: p.error("Invalid sample count/grid step")
    from pathlib import Path
    out = Path(args.output); out.mkdir(parents=True,exist_ok=True)
    model = RobotModel()
    policy = GraspPolicy(object_width_m=args.object_width,approach_height_m=args.approach_height,collision_margin_m=args.collision_margin)
    policy.validate()
    started = time.monotonic()
    u = qmc.Sobol(4,scramble=True,seed=42).random_base2(args.samples_power)
    qs = model.lower+u*(model.upper-model.lower)
    positions = model.fk(qs)["end_effector_link"][:,:3,3]
    safe, floors = model.screen(qs,margin=policy.collision_margin_m,floor_margin=policy.floor_margin_m)
    save_csv(out/"reachable_workspace.csv",np.c_[positions,qs,safe.astype(int),floors],
             ["x_m","y_m","z_m","q1_rad","q2_rad","q3_rad","q4_rad","collision_screened","floor_clearance_m"])
    print(f"FK: {len(qs)} configurations, {safe.sum()} pass conservative collision/floor checks",flush=True)
    max_r = model.l1+model.l2+model.tool
    radii = np.arange(.04,max_r,args.grid_step)
    heights = np.arange(.025,model.shoulder[2]+model.l1+model.l2-model.tool,args.grid_step)
    # Global coverage + dense front sector: this URDF has very small lidar clearance.
    azimuths = np.unique(np.r_[np.linspace(-math.pi,math.pi,49,endpoint=False),np.deg2rad(np.arange(-12,13,1)),0.])
    r,a,z = np.meshgrid(radii,azimuths,heights,indexing="ij")
    grid = np.c_[model.shoulder[0]+r.ravel()*np.cos(a.ravel()),model.shoulder[1]+r.ravel()*np.sin(a.ravel()),z.ravel()]
    candidate_q, valid = model.ik(grid,policy.pitch_rad,policy.joint_margin_rad)
    pre = grid.copy(); pre[:,2] += max(policy.approach_height_m,policy.lift_height_m)
    _, pre_valid = model.ik(pre,policy.pitch_rad,policy.joint_margin_rad)
    valid &= pre_valid
    survivors = np.zeros(len(grid),dtype=bool)
    for branch in range(4):
        indices = np.flatnonzero(valid[:,branch])
        if len(indices):
            screened,_ = model.screen(candidate_q[indices,branch],margin=policy.collision_margin_m,floor_margin=policy.floor_margin_m)
            survivors[indices[screened]] = True
    print(f"Top-down grid: {len(grid)} points, {survivors.sum()} need full approach/close/lift verification",flush=True)
    rows, examples = [], []
    ids = np.flatnonzero(survivors)
    for k,i in enumerate(ids):
        result = check_grasp(model,grid[i],policy)
        if result is not None:
            rows.append([*grid[i],*result["q_grasp_rad"],result["quality_score"],result["joint_limit_margin_score"],result["min_floor_clearance_m"],result["branch"]])
            if len(examples) < 3: examples.append({"object_base_xyz_m":grid[i].tolist(),**result})
        if k%250 == 0: print(f"Approach verification {k}/{len(ids)}; feasible {len(rows)}",flush=True)
    columns = ["x_m","y_m","z_m","q1_rad","q2_rad","q3_rad","q4_rad","quality_score","joint_margin_score","floor_clearance_m","branch"]
    save_csv(out/"grasp_workspace.csv",rows,columns)
    grasps = np.asarray(rows).reshape(-1,len(columns))
    summary = {"schema_version":1,"frame_id":"base_footprint","units":"m,rad","policy":asdict(policy),
               "robot_model_sha256":model.sha256,"collision_backend":model.collision_backend,"self_collision_pairs":len(model.pairs),
               "reach_configurations":len(qs),"collision_screened_configurations":int(safe.sum()),"kinematic_bounds":bounds(positions),
               "collision_screened_bounds":bounds(positions[safe]),"grasp_grid_points":len(grid),"feasible_grasp_samples":len(grasps),
               "grasp_bounds":bounds(grasps[:,:3]),"grasp_radial_range_from_shoulder_m":None if not len(grasps) else [float(np.linalg.norm(grasps[:,:2]-model.shoulder[:2],axis=1).min()),float(np.linalg.norm(grasps[:,:2]-model.shoulder[:2],axis=1).max())],
               "examples":examples,"elapsed_seconds":time.monotonic()-started,
               "limitations":["Discrete samples, not an exact continuous workspace boundary or certified volume.",
                              "Kinematic top-down grasp corridor, no dynamic grasp/contact/torque validation.",
                              "Collision meshes conservatively replaced by enclosing convex hulls; possible false rejections.",
                              "Only structural adjacent SRDF exemptions used; Never/User exemptions intentionally not used.",
                              "Zero extra self-collision padding is nominal URDF geometry, not calibration/uncertainty clearance.",
                              "Floor, robot self geometry, and spherical target checked. Tables/environment absent unless supplied as boxes to planner.",
                              "Object XYZ means grasp-center TCP; width envelope is not a banana shape model."]}
    (out/"workspace_summary.json").write_text(json.dumps(summary,indent=2))
    plt.rcParams.update({"font.size":10,"figure.facecolor":"white"})
    fig = plt.figure(figsize=(15,6),layout="constrained")
    ax = fig.add_subplot(121,projection="3d")
    stride = max(1,len(positions)//6000)
    ax.scatter(*positions[::stride].T,c="#b5bdca",s=1,alpha=.15,label="Joint-limit FK samples")
    ax.scatter(*positions[safe].T,c="#3385b8",s=3,alpha=.4,label="Self / floor collision screened")
    if len(grasps): ax.scatter(*grasps[:,:3].T,c=grasps[:,7],cmap="YlOrRd",s=7,alpha=.7,label="Top-down descent + close + lift")
    ax.set(xlabel="base_footprint X [m]",ylabel="Y [m]",zlabel="Z [m]",title="4-DOF reachable and graspable samples")
    ax.legend(loc="upper left",fontsize=8); ax.view_init(22,-65)
    ax2 = fig.add_subplot(122)
    ax2.scatter(np.linalg.norm(positions[::stride,:2]-model.shoulder[:2],axis=1),positions[::stride,2],s=2,c="#b5bdca",alpha=.15,label="Position reachability")
    if len(grasps):
        plot = ax2.scatter(np.linalg.norm(grasps[:,:2]-model.shoulder[:2],axis=1),grasps[:,2],c=grasps[:,7],s=10,cmap="YlOrRd",vmin=0,vmax=1)
        fig.colorbar(plot,ax=ax2,label="Geometric margin score (not a probability)",shrink=.7)
    ax2.axhline(0,c="#364152",lw=1); ax2.axhline(model.shoulder[2],c="#607d8b",ls=":",label="Shoulder height")
    ax2.set(xlabel="Radius from shoulder yaw axis [m]",ylabel="TCP grasp-center Z [m]",title=f"Vertical tool, {policy.approach_height_m*100:.0f} cm approach/lift, {policy.object_width_m*1000:.0f} mm width")
    ax2.legend(loc="lower right",fontsize=8); ax2.grid(alpha=.2)
    fig.suptitle("TurtleBot3 + OpenMANIPULATOR-X | sampled geometric feasibility",fontsize=15)
    fig.savefig(out/"workspace.png",dpi=180); fig.savefig(out/"workspace.pdf"); plt.close(fig)
    fig,ax = plt.subplots(figsize=(9,7),layout="constrained")
    if len(grasps):
        color=ax.scatter(grasps[:,0],grasps[:,1],c=grasps[:,2],cmap="viridis",s=12,alpha=.6)
        fig.colorbar(color,ax=ax,label="Grasp-center height [m]")
    ax.add_patch(plt.Rectangle((-.197,-.133),.266,.266,fill=False,edgecolor="#3b4c60",lw=2,label="URDF base collision box"))
    ax.scatter([model.shoulder[0]],[model.shoulder[1]],marker="+",s=80,c="red",label="Shoulder yaw axis")
    ax.set(xlabel="base_footprint X [m]",ylabel="Y [m]",title="Top-down graspable samples: conservative collision model",aspect="equal")
    ax.grid(alpha=.2); ax.legend(fontsize=8); fig.savefig(out/"grasp_top_view.png",dpi=180); plt.close(fig)
    print(json.dumps({k:summary[k] for k in ("feasible_grasp_samples","grasp_bounds","grasp_radial_range_from_shoulder_m","elapsed_seconds")},indent=2),flush=True)


if __name__ == "__main__": main()
