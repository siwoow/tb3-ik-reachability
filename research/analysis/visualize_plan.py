"""Plot saved candidate base goals and the best geometric grasp corridor."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tb3_grasp.model import RobotModel,ROOT


def main():
    p=argparse.ArgumentParser(); p.add_argument("plan"); args=p.parse_args()
    path=Path(args.plan); result=json.loads(path.read_text()); best=result['best']
    fig,axes=plt.subplots(1,2,figsize=(13,5),layout="constrained")
    ax=axes[0]; obj=np.array(result['object_xyz_m']); current=np.array(result['current_base_xyyaw'])
    ax.scatter(*current[:2],s=100,marker="s",c="#31688e",label="Current base")
    ax.scatter(*obj[:2],s=120,marker="*",c="#ee8c32",label="Input grasp center")
    candidates=result['candidates']
    for c in candidates:
        ax.arrow(c['base_x_m'],c['base_y_m'],.025*math.cos(c['base_yaw_rad']),.025*math.sin(c['base_yaw_rad']),head_width=.008,color="#839c9c",alpha=.55,length_includes_head=True)
    if best:
        xy=np.array([best['base_x_m'],best['base_y_m']]); yaw=best['base_yaw_rad']
        ax.scatter(*xy,s=100,c="#239870",label="Selected base goal")
        ax.annotate(f"({xy[0]:.3f}, {xy[1]:.3f}), yaw={math.degrees(yaw):.1f} deg",xy,xytext=(10,15),textcoords="offset points",fontsize=9)
        model=RobotModel(); q=np.array(best['corridor_q_rad']); poses=model.fk(q)
        view=axes[1]
        for i,k in enumerate([0,-1]):
            joints=np.array([poses[name][k,:3,3] for name in ['link2','link3','link4','link5','end_effector_link']])
            view.plot(joints[:,0],joints[:,2],"o-",c=['#ee8c32','#31688e'][i],label=['Grasp','Pregrasp / lift'][i])
        view.plot(poses['end_effector_link'][:,0,3],poses['end_effector_link'][:,2,3],"--",c="#239870",label="Checked vertical corridor")
        view.axhline(0,c="#344054"); view.set(xlabel="base_footprint X [m]",ylabel="Z [m]",title="Selected 4-DOF branch, side view",aspect="equal");view.legend(fontsize=8);view.grid(alpha=.2)
    ax.set(xlabel=f"{result['frame_id']} X [m]",ylabel="Y [m]",title="Base goal candidates | no navigation path checked",aspect="equal");ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.suptitle("Offline planning result: geometric feasibility only",fontsize=14)
    out=path.with_suffix('.png');fig.savefig(out,dpi=180);print(out)


if __name__=='__main__':main()
