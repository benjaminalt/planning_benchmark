import logging
import os
import yaml
from unittest import TestCase
import os.path as osp
from xmlrpclib import ServerProxy

from yaml import SafeLoader

import planning_benchmark_common as pbc
import openravepy

from planning_benchmark_common.transform_utils import affine_to_pose

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PR2_URDF_PATH = os.path.join(SCRIPT_DIR, os.pardir, "robots_urdf", "pr2.urdf")
PR2_SCENES_DIR = os.path.join(SCRIPT_DIR, os.pardir, "pr2_scenes")


class TestAMDiffRobotModelPR2Kinematics(TestCase):
    def setUp(self):
        logging.basicConfig()
        logging.getLogger().setLevel(logging.DEBUG)

        # Setup OpenRave robot
        env = openravepy.Environment()
        env.StopSimulation()
        env.Load(osp.join(pbc.envfile_dir, "bookshelves.env.xml"))
        env.Load("robots/pr2-beta-static.zae")
        self.or_robot = env.GetRobots()[0]

        with open("/home/alt/catkin_ws/src/planning_benchmark/problem_sets/bookshelves_rightarm.yaml") as problem_file:
            self.planning_problem = yaml.load(problem_file, Loader=SafeLoader)

        # Setup AMR robot (planning service)
        self.planner = ServerProxy("http://localhost:3456/dgpmp2")
        self.planner.set_env(os.path.join(PR2_SCENES_DIR, "bookshelves.scene"),
                             {
                                 "voxel_size": 0.05,
                                 "collision_padding": 0.01
                             },
                             {
                                 "urdf": PR2_URDF_PATH,
                                 "good_initial_conf": self.planning_problem["default_joint_values"],
                                 "good_initial_conf_joint_names": self.planning_problem["joint_names"],
                                 "ee_link_name": "r_gripper_palm_link",
                                 "active_joint_names": self.planning_problem["active_joints"]
                             })

    def testForwardKinematicsForDefaultJointValues(self):
        goal_state_active = self.planning_problem["problems"][0]["start"]["active_dof_values"]
        ee_pose_planner = self.planner.forward_kinematics(goal_state_active)
        ee_poses_all_links = self.planner.forward_kinematics_all_links(goal_state_active)
        rave_inds, rave_values = [], []
        rave_joint_names = [joint.GetName() for joint in self.or_robot.GetJoints()]
        # Set default joint values
        for (name, val) in zip(self.planning_problem["joint_names"], self.planning_problem["default_joint_values"]):
            if name in rave_joint_names:
                i = rave_joint_names.index(name)
                rave_inds.append(i)
                rave_values.append(val)
                # rave_values.append(0.0)
        self.or_robot.SetDOFValues(rave_values, rave_inds)
        active_joint_inds = [rave_joint_names.index(name) for name in self.planning_problem["active_joints"]]
        self.or_robot.SetActiveDOFs(active_joint_inds, self.planning_problem["active_affine"])
        self.or_robot.SetActiveDOFValues(goal_state_active)

        manip = self.or_robot.GetManipulator("rightarm")
        rave_ee_tf = manip.GetEndEffectorTransform()
        rave_ee_pose = affine_to_pose(rave_ee_tf)
        rave_raw_manip_tf = manip.GetEndEffector().GetTransform()
        rave_raw_manip_pose = affine_to_pose(rave_raw_manip_tf)

        print "ee_pose_planner: {}".format(ee_pose_planner)
        print "rave_ee_pose: {}".format(rave_ee_pose)
        print "rave_raw_manip_pose: {}".format(rave_raw_manip_pose)
        print "#########################"
        for link in self.or_robot.GetLinks():
            link_tf = link.GetTransform()
            link_pose = affine_to_pose(link_tf)
            print "{}: {}".format(link.GetName(), link_pose)
        print "Done"

    def testForwardKinematicsAllLinks(self):
        self.planner.forward_kinematics_all_links(self.planning_problem[""])