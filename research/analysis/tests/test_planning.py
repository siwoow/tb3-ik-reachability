"""Geometric tests independent of Isaac/ROS; no hardware commands."""
import math
import unittest
from pathlib import Path
import tempfile
import shutil
import numpy as np
from tb3_grasp.model import RobotModel, obb_overlap, wrap
from tb3_grasp.planner import GraspPolicy, check_grasp, transform_object_to_base, plan_base_positions, boxes_in_base


class PlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.m = RobotModel()

    def test_random_4dof_fk_ik_roundtrip(self):
        m=self.m; rng=np.random.default_rng(923)
        qs=m.lower+.001+rng.random((300,4))*(m.upper-m.lower-.002)
        points=m.fk(qs)["end_effector_link"][:,:3,3]
        for p,q in zip(points,qs):
            solutions,valid=m.ik([p],float(q[1:].sum()))
            self.assertTrue(valid.any(),(p,q))
            recovered=solutions[0,valid[0]]
            self.assertLess(float(np.linalg.norm(wrap(recovered-q),axis=1).min()),1.e-6)

    def test_top_down_orientation_and_path(self):
        result=check_grasp(self.m,[.12,0,.05])
        self.assertIsNotNone(result)
        qs=np.asarray(result["corridor_q_rad"])
        poses=self.m.fk(qs)["end_effector_link"]
        np.testing.assert_allclose(poses[:,:3,0],np.tile([0,0,-1],(len(qs),1)),atol=1.e-9)
        np.testing.assert_allclose(poses[:,:2,3],np.tile([.12,0],(len(qs),1)),atol=1.e-9)
        self.assertAlmostEqual(poses[-1,2,3],.11)
        self.assertGreaterEqual(result["min_floor_clearance_m"],.003)
        self.assertTrue(np.all(qs>=self.m.lower+math.radians(2)-1.e-9))
        self.assertTrue(np.all(qs<=self.m.upper-math.radians(2)+1.e-9))

    def test_native_limit_and_floor_rejection(self):
        q=self.m.upper.copy(); q[1]+=.01
        self.assertFalse(self.m.screen([q])[0][0])
        self.assertIsNone(check_grasp(self.m,[.12,0,.005]))
        self.assertIsNone(check_grasp(self.m,[.12,0,.05],GraspPolicy(floor_z_m=.1)))

    def test_strict_padding_does_not_silently_relax(self):
        self.assertIsNone(check_grasp(self.m,[.12,0,.05],GraspPolicy(collision_margin_m=.003)))

    def test_unreachable_height_and_wide_object(self):
        self.assertIsNone(check_grasp(self.m,[.12,0,1.]))
        self.assertIsNone(check_grasp(self.m,[.12,0,.05],GraspPolicy(object_width_m=.1)))
        result=plan_base_positions([.5,0,1.],[0,0,0],model=self.m,radii=[.2],azimuth_samples=4,yaw_samples=4)
        self.assertIsNone(result["best"])
        self.assertFalse(result["navigation_collision_free"])

    def test_rotated_translated_odom_frame(self):
        pose=np.array([1.,2.,math.pi/2]); local=np.array([.12,0,.05]); world=np.array([1.,2.12,.05])
        np.testing.assert_allclose(transform_object_to_base(world,pose),local,atol=1.e-10)
        result=plan_base_positions(world,pose,model=self.m,radii=[.2],azimuth_samples=4,yaw_samples=4,top_k=1)
        self.assertIsNotNone(result["best"])
        np.testing.assert_allclose([result['best'][n] for n in ('base_x_m','base_y_m','base_yaw_rad')],pose,atol=1.e-9)
        self.assertFalse(result["navigation_path_checked"])

    def test_base_inverse_transform_and_heading_constraint(self):
        result=plan_base_positions([.5,0,.05],[0,0,0],model=self.m,radii=[.2],azimuth_samples=4,yaw_samples=4,desired_closing_yaw_world=math.pi/2,top_k=3)
        self.assertIsNotNone(result["best"])
        for c in result["candidates"]:
            pose=[c["base_x_m"],c["base_y_m"],c["base_yaw_rad"]]
            np.testing.assert_allclose(transform_object_to_base([.5,0,.05],pose),c["object_base_footprint_m"],atol=1.e-9)
            self.assertIsNotNone(check_grasp(self.m,c["object_base_footprint_m"]))
            self.assertLess(abs(float(wrap(2*(c["closing_yaw_world_rad"]-math.pi/2))))/2,math.radians(15)+1.e-9)

    def test_known_obstacle_rejection(self):
        boxes=boxes_in_base([{"center":[.12,0,.05],"size":[.10,.10,.10]}],[0,0,0])
        self.assertIsNone(check_grasp(self.m,[.12,0,.05],obstacles=boxes))
        # Box encloses every nearby base and grasp corridor, including containment.
        result=plan_base_positions([.5,0,.05],[0,0,0],model=self.m,radii=[.2],azimuth_samples=4,yaw_samples=4,
                                   obstacles_world=[{"center":[.5,0,.2],"size":[3,3,2]}])
        self.assertIsNone(result["best"])

    def test_invalid_input_fails_explicitly(self):
        for p in ([math.nan,0,0],[0,0],[0,0,math.inf]):
            with self.assertRaises(ValueError): check_grasp(self.m,p)
        with self.assertRaises(ValueError): GraspPolicy(cartesian_step_m=0).validate()
        with self.assertRaises(ValueError): GraspPolicy(pitch_rad=0).validate()
        with self.assertRaises(ValueError): plan_base_positions([0,0,0],[0,0,0],model=self.m,frame="camera_rgb_optical_frame")

    def test_detached_collision_cache_and_model_integrity(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); urdf=root/'robot.urdf'; cache=root/'collision_bounds.json'
            shutil.copyfile(self.m.urdf,urdf)
            shutil.copyfile(self.m.urdf.with_name('collision_bounds.json'),cache)
            portable=RobotModel(urdf=urdf,bounds=cache,srdf=root/'missing.srdf')
            self.assertEqual(len(portable.pairs),len(self.m.pairs))
            self.assertIsNotNone(check_grasp(portable,[.12,0,.05]))
            urdf.write_text(urdf.read_text().replace('velocity="4.8"','velocity="4.7"',1))
            with self.assertRaises(ValueError): RobotModel(urdf=urdf,bounds=cache,srdf=root/'missing.srdf')

    def test_obb_sat_rotations_and_touching(self):
        eye=np.eye(3)[None]; zero=np.zeros((1,3)); half=np.ones(3)
        self.assertTrue(obb_overlap(zero,eye,half,zero,eye,half)[0])
        self.assertFalse(obb_overlap(zero,eye,half,np.array([[3,0,0]]),eye,half)[0])
        # Counterexample where world AABBs overlap but the oriented thin boxes do not.
        t=math.pi/4; r=np.array([[math.cos(t),-math.sin(t),0],[math.sin(t),math.cos(t),0],[0,0,1]])[None]
        self.assertFalse(obb_overlap(zero,r,np.array([2,.1,.1]),np.array([[-.5,.5,0]]),r,np.array([2,.1,.1]))[0])


if __name__ == "__main__": unittest.main()
