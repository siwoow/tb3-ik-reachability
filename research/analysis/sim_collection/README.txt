TurtleBot3 + OpenMANIPULATOR-X collection demonstration

Simulation only. No ROS extensions, Jetson connection or real motor commands.

Run from Ubuntu terminal:
cd /home/mobility/tb3_grasp_analysis
.venv/bin/python sim_collection/prepare_scene.py
/home/mobility/miniconda3/envs/env_isaaclab/bin/python sim_collection/collection_demo.py --headless --enable_cameras

Physical trial:
  sim_collection/physical/collection.mp4
  sim_collection/physical/scene.usda
  sim_collection/physical/result.json
The script stops after a failed lift instead of pretending success.

Explicit visualization alternative (if contact grasp fails):
/home/mobility/miniconda3/envs/env_isaaclab/bin/python sim_collection/collection_demo.py --headless --enable_cameras --assisted --output /home/mobility/tb3_grasp_analysis/sim_collection/video_final
  sim_collection/video_final/collection.mp4
Assisted mode prescribes arm joint states and writes banana poses at the TCP during transport. This is animation,
NOT a successful physical grasp. The video carries that label permanently.

Model and limits: existing resolved.urdf, original collision meshes and joint limits.
Existing Planner supplies FCL-screened vertical approach/descent/lift corridors;
these are not the real robot's -45 degree native path. No new IK is implemented.
Object position comes from simulation ground truth, not simulated perception.
No camera is added to the robot; the third-person filming camera is independent.
Banana geometry is a synthetic compound rigid object, mass 80g, not a scan.
Base pose follows a prescribed smooth trajectory, not wheel traction control.
In default physical mode, contact dynamics and arm drives run in PhysX. Bin contact is simulated; the
legacy planner's sphere approximation does not certify whole-banana clearance.
USD saves the scene and assets; rerun the Python file for motion/recording.
Original USD, URDF, SDK, Bringup and hardware files are not modified.

Final narrative demo: video_final/collection.mp4 and video_final/scene.usda.
Assisted mode prescribes the banana pose for pickup, transport and placement.
Its collisions are disabled from initialization ONLY in this explicit animation
alternative. It does not validate contact, release or bin-settling physics.
Default physical mode retains the banana collision geometry and dynamics.
