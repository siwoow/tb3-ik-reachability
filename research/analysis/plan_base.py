"""Offline CLI: odom-frame grasp center -> ranked base positions, JSON + CSV."""
import argparse
import csv
import json
from pathlib import Path
import math
from tb3_grasp.model import ROOT, RobotModel
from tb3_grasp.planner import GraspPolicy, plan_base_positions


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--object",type=float,nargs=3,required=True,metavar=("X","Y","Z"))
    p.add_argument("--base",type=float,nargs=3,required=True,metavar=("X","Y","YAW_RAD"))
    p.add_argument("--frame",default="odom")
    p.add_argument("--base-z",type=float,default=0.)
    p.add_argument("--object-width",type=float,default=.030)
    p.add_argument("--approach-height",type=float,default=.060)
    p.add_argument("--collision-margin",type=float,default=0.)
    p.add_argument("--closing-yaw-deg",type=float,default=None)
    p.add_argument("--obstacles",help="JSON list of world-frame boxes with center and size")
    p.add_argument("--output",default=str(ROOT/"results/base_plan.json"))
    args = p.parse_args()
    policy = GraspPolicy(object_width_m=args.object_width,approach_height_m=args.approach_height,collision_margin_m=args.collision_margin)
    result = plan_base_positions(args.object,args.base,policy=policy,frame=args.frame,base_z=args.base_z,
                                desired_closing_yaw_world=None if args.closing_yaw_deg is None else math.radians(args.closing_yaw_deg),
                                obstacles_world=None if not args.obstacles else json.loads(Path(args.obstacles).read_text()))
    out = Path(args.output); out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2))
    keys = ["base_x_m","base_y_m","base_yaw_rad","distance_m","final_yaw_change_rad","differential_drive_turn_sum_rad","quality_score","score_cost","closing_yaw_world_rad"]
    with out.with_suffix(".csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=keys,extrasaction="ignore"); writer.writeheader(); writer.writerows(result["candidates"])
    best=result["best"]
    print(json.dumps({"status":result["status"],"best":None if best is None else {k:best[k] for k in keys},
                      "json":str(out),"navigation_collision_free":False},indent=2))


if __name__ == "__main__": main()
