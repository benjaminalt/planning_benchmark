import argparse, sys
import random
import stat
import subprocess
from functools import partial
from xmlrpclib import ServerProxy

parser = argparse.ArgumentParser()

g0 = parser.add_argument_group('common', 'Common args')
g0.add_argument("problemfile", type=argparse.FileType("r"), help="Problem description YAML file")
g0.add_argument("planner", choices=["trajopt", "ompl", "chomp", "chomp2", "dgpmp2"], help="Planner to run")
g0.add_argument("-o", "--outfile", type=argparse.FileType("w"),
                help="File to dump results (generated trajectories, timing info, etc.)")
g0.add_argument("--record_failed_problems", type=argparse.FileType("w"), help="File to save failed start/goal pairs")
g0.add_argument("--problems", type=argparse.FileType("r"),
                help="ignore the problems in problemfile and use these only (good for output from --record_failed_problems)")
g0.add_argument("--animate_all", action="store_true", help="animate solutions to every problem after solving")

# chomp+ompl options
g1 = parser.add_argument_group('chomp+ompl', 'Options for CHOMP and OMPL')
g1.add_argument("--max_planning_time", type=float, default=10, help="max planning time for chomp and ompl")

# trajopt+chomp options
g2 = parser.add_argument_group("trajopt+chomp", "Options for optimization-based planners (trajopt and CHOMP)")
g2.add_argument("--n_steps", type=int, default=11, help="num steps in generated trajectory for trajopt and chomp")
g2.add_argument("--multi_init", action="store_true", help="Use multiple initializations, for trajopt and chomp")
g2.add_argument("--use_random_inits", action="store_true",
                help="Use 5 random collision-free states for initializations. Only active if --multi_init is passed too")

# trajopt options
g3 = parser.add_argument_group("trajopt-only", "Options specific to trajopt")
g3.add_argument("--interactive", action="store_true", help="Interactively display steps in the optimization")
g3.add_argument("--include_2step_sweeps", action="store_true", help="add in costs for swept volumes 0-2,1-3,2-4")

# ompl options
g4 = parser.add_argument_group("ompl-only", "Options specific to OMPL")
g4.add_argument("--ompl_planner_id", default="", help="OMPL planner ID",
                choices=[
                    "",
                    "SBLkConfigDefault",
                    "LBKPIECEkConfigDefault",
                    "RRTkConfigDefault",
                    "RRTConnectkConfigDefault",
                    "ESTkConfigDefault",
                    "KPIECEkConfigDefault",
                    "BKPIECEkConfigDefault",
                    "RRTStarkConfigDefault"])

# chomp options
g5 = parser.add_argument_group("chomp-only", "Options specific to CHOMP")
g5.add_argument("--chomp_argstr", default="comp", help="CHOMP arg string. Can be 'comp' or 'hmc-seedXXXX'")

g6 = parser.add_argument_group("dgpmp2-only", "Options specific to DGPMP2")
g6.add_argument("--python_env", type=str)
g6.add_argument("--dgpmp2_dir", type=str)

args = parser.parse_args()

if args.outfile is None: args.outfile = sys.stdout

import yaml, openravepy, trajoptpy
import os
import os.path as osp

sys.path.append(osp.dirname(osp.dirname(osp.abspath(sys.argv[0]))))
import planning_benchmark_common as pbc
from trajoptpy.check_traj import traj_is_safe, traj_collisions
from planning_benchmark_common.sampling import sample_base_positions
import trajoptpy.math_utils as mu
from time import time, sleep
import numpy as np
import json
import planning_benchmark_common.func_utils as fu

TMP_DATA_DIR = os.path.normpath(os.path.join(os.path.abspath(os.path.dirname(__file__)), os.pardir, "tmp_data"))
PLANNER_CONFIGS_DIR = os.path.normpath(os.path.join(os.path.abspath(os.path.dirname(__file__)), os.pardir, "planner_configs"))

LEFT_POSTURES = [
    [-0.243379, 0.103374, -1.6, -2.27679, 3.02165, -2.03223, -1.6209],  # chest fwd
    [-1.68199, -0.088593, -1.6, -2.08996, 3.04403, -0.41007, -1.39646],  # side fwd
    [-0.0428341, -0.489164, -0.6, -1.40856, 2.32152, -0.669566, -2.13699],  # face up
    [0.0397607, 1.18538, -0.8, -0.756239, -2.84594, -1.06418, -2.42207]  # floor down
]

KITCHEN_WAYPOINTS = [
    [0., 0.0436, 0.8844, 1.493, -0.2914, 2.6037, -0.4586,
     0.6602, 0.0155, 0.8421, -2.0777, -0.544, -2.5683, -0.4762,
     -1.5533, 1.4904, -0.4271, 1.8619],
    [0., 0.0436, 0.8844, 1.493, -0.2914, 2.6037, -0.4586,
     0.6602, 0.0155, 0.8421, -2.0777, -0.544, -2.5683, -0.4762,
     -1.5533, 2.1866, 2.4017, -2.285]
]

dgpmp2 = ServerProxy("http://localhost:3456/dgpmp2")
DGPMP_ITER = 0


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

    def disable(self):
        self.HEADER = ''
        self.OKBLUE = ''
        self.OKGREEN = ''
        self.WARNING = ''
        self.FAIL = ''
        self.ENDC = ''


class PlannerStatus:
    SUCCESS = 'success'
    FAIL_TIMEOUT = 'fail_timeout'
    FAIL_JOINT_VIOLATION = 'fail_joint_violation'
    FAIL_COLLISION = 'fail_collision'
    FAIL_OTHER = 'fail_other'


def traj_in_joint_limits(traj, robot):
    lower, upper = robot.GetActiveDOFLimits()
    in_limits = (lower[None, :] <= traj) & (traj <= upper[None, :])
    return in_limits.all()


def mirror_arm_joints(x):
    "mirror image of joints (r->l or l->r)"
    return [-x[0], x[1], -x[2], x[3], -x[4], x[5], -x[6]]


def get_postures(group_name):
    if group_name == "left_arm": return LEFT_POSTURES
    if group_name == "right_arm": return [mirror_arm_joints(posture) for posture in LEFT_POSTURES]
    # if group_name=="whole_body": return KITCHEN_WAYPOINTS
    raise Exception


def animate_traj(traj, robot, pause=True, restore=True):
    """make sure to set active DOFs beforehand"""
    if restore: _saver = openravepy.RobotStateSaver(robot)
    viewer = trajoptpy.GetViewer(robot.GetEnv())
    for (i, dofs) in enumerate(traj):
        print "step %i/%i" % (i + 1, len(traj))
        robot.SetActiveDOFValues(dofs)
        if pause:
            viewer.Idle()
        else:
            viewer.Step()


def traj_to_array(robot, traj):
    """make sure to set active DOFs (including affine, if needed) beforehand"""
    assert robot.GetAffineDOF() in [0, 11]  # we only support these right now
    cs = traj.GetConfigurationSpecification()
    out = np.empty((traj.GetNumWaypoints(), robot.GetActiveDOF()))
    has_affine = robot.GetAffineDOF() == 11
    with robot:
        for i in range(traj.GetNumWaypoints()):
            wp = traj.GetWaypoint(i)
            joints = cs.ExtractJointValues(wp, robot, robot.GetActiveDOFIndices())
            if has_affine:
                robot.SetTransform(cs.ExtractTransform(None, wp, robot))
                affine = robot.GetActiveDOFValues()[-3:]
                out[i] = np.r_[joints, affine]
            else:
                out[i] = joints
    return out


def array_to_traj(robot, a, dt=1):
    spec = openravepy.ConfigurationSpecification()
    name = "joint_values %s %s" % (robot.GetName(), ' '.join(map(str, robot.GetActiveDOFIndices())))
    spec.AddGroup(name, robot.GetActiveDOF(), interpolation="linear")
    spec.AddDeltaTimeGroup()
    traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), '')
    traj.Init(spec)
    for i, joints in enumerate(a):
        pt = np.zeros(spec.GetDOF())
        spec.InsertJointValues(pt, joints, robot, robot.GetActiveDOFIndices(), 0)
        spec.InsertDeltaTime(pt, 0 if i == 0 else dt)
        traj.Insert(i, pt)
    return traj


def traj_no_self_collisions(traj, robot):
    with robot:
        for joints in traj:
            robot.SetActiveDOFValues(joints)
            if robot.CheckSelfCollision():
                return False
    return True


def hash_env(env):
    # hash based on non-robot object transforms and geometries
    import hashlib
    return hashlib.sha1(','.join(
        str(body.GetTransform()) + body.GetKinematicsGeometryHash() for body in env.GetBodies() if
        not body.IsRobot())).hexdigest()


def ros_quat_to_aa(q):
    return openravepy.axisAngleFromQuat([q.w, q.x, q.y, q.z])


@fu.once
def get_ompl_service():
    import rospy
    import moveit_msgs.srv as ms
    svc = rospy.ServiceProxy('plan_kinematic_path', ms.GetMotionPlan)
    print "waiting for plan_kinematic_path"
    svc.wait_for_service()
    print "ok"
    return svc


def setup_ompl(env):
    import rospy
    from planning_benchmark_common.rave_env_to_ros import rave_env_to_ros
    rospy.init_node("benchmark_ompl", disable_signals=True)
    get_ompl_service()
    rave_env_to_ros(env)


def postsetup_trajopt(env):
    "use the ROS config file to ignore some impossible self collisions. very slight speedup"
    import xml.etree.ElementTree as ET
    robot = env.GetRobot("pr2")
    cc = trajoptpy.GetCollisionChecker(env)
    root = ET.parse("/opt/ros/groovy/share/pr2_moveit_config/config/pr2.srdf").getroot()
    disabled_elems = root.findall("disable_collisions")
    for elem in disabled_elems:
        linki = robot.GetLink(elem.get("link1"))
        linkj = robot.GetLink(elem.get("link2"))
        if linki and linkj:
            cc.ExcludeCollisionPair(linki, linkj)


@fu.once
def get_chomp_module(env):
    if args.planner == "chomp":
        from orcdchomp import orcdchomp
        # chomp_module_path = "/home/jonathan/build/chomp/liborcdchomp.so"
        chomp_module_path = "/home/alt/catkin_ws/devel/share/openrave-0.9/plugins/or_cdchomp_plugin.so"
    elif args.planner == "chomp2":
        from orcdchomp2 import orcdchomp
        chomp_module_path = "/home/jonathan/code/orcdchomp2/lib/liborcdchomp2.so"
    else:
        assert False
    openravepy.RaveLoadPlugin(chomp_module_path)
    m_chomp = openravepy.RaveCreateModule(env, 'orcdchomp')
    env.Add(m_chomp, True, 'blah_load_string')
    orcdchomp.bind(m_chomp)
    return m_chomp


def setup_chomp(env):
    get_chomp_module(env)


def gen_init_trajs(problemset, robot, n_steps, start_joints, end_joints):
    waypoint_step = (n_steps - 1) // 2
    joint_waypoints = [(np.asarray(start_joints) + np.asarray(end_joints)) / 2]
    if args.multi_init:
        if args.use_random_inits:
            print 'using random initializations'
            joint_waypoints.extend(sample_base_positions(robot, num=5, tucked=True))
        # if the problem file has waypoints, just use those
        else:
            if problemset["group_name"] in ["right_arm", "left_arm", "whole_body"]:
                joint_waypoints.extend(get_postures(problemset["group_name"]))
    trajs = []
    for i, waypoint in enumerate(joint_waypoints):
        if i == 0:
            inittraj = mu.linspace2d(start_joints, end_joints, n_steps)
        else:
            inittraj = np.empty((n_steps, robot.GetActiveDOF()))
            inittraj[:waypoint_step + 1] = mu.linspace2d(start_joints, waypoint, waypoint_step + 1)
            inittraj[waypoint_step:] = mu.linspace2d(waypoint, end_joints, n_steps - waypoint_step)
        trajs.append(inittraj)
    return trajs


def make_trajopt_request(n_steps, coll_coeff, dist_pen, end_joints, inittraj, use_discrete_collision):
    d = {
        "basic_info": {
            "n_steps": n_steps,
            "manip": "active",
            "start_fixed": True
        },
        "costs": [
            {
                "type": "joint_vel",
                "params": {"coeffs": [1]}
            },
            {
                "type": "collision",
                "params": {"coeffs": [coll_coeff], "dist_pen": [dist_pen], "continuous": True, "gap": 1}
            }
        ],
        "constraints": [
            {"type": "joint", "params": {"vals": end_joints}}
        ],
        "init_info": {
            "type": "given_traj",
            "data": [row.tolist() for row in inittraj]
        }
    }
    if use_discrete_collision:
        d["costs"].append({
            "name": "discrete_collision",
            "type": "collision",
            "params": {"coeffs": [coll_coeff], "dist_pen": [dist_pen], "continuous": False}
        })
    if args.include_2step_sweeps:
        d["costs"].append({
            "type": "collision",
            "params": {"coeffs": [coll_coeff], "dist_pen": [dist_pen], "continuous": True, "gap": 2}
        })

    return json.dumps(d)


def trajopt_plan(robot, group_name, active_joint_names, active_affine, end_joints, init_trajs):
    start_joints = robot.GetActiveDOFValues()

    n_steps = args.n_steps
    coll_coeff = 40
    dist_pen = .02

    def single_trial(inittraj, use_discrete_collision):
        s = make_trajopt_request(n_steps, coll_coeff, dist_pen, end_joints, inittraj, use_discrete_collision)
        robot.SetActiveDOFValues(inittraj[0, :])
        prob = trajoptpy.ConstructProblem(s, robot.GetEnv())
        result = trajoptpy.OptimizeProblem(prob)
        traj = result.GetTraj()
        prob.SetRobotActiveDOFs()
        return traj, traj_is_safe(traj, robot)  # and (use_discrete_collision or traj_no_self_collisions(traj, robot))

    status = PlannerStatus.FAIL_COLLISION
    msg = ''
    t_start = time()
    for (i_init, inittraj) in enumerate(init_trajs):
        traj, is_safe = single_trial(inittraj, True)
        if is_safe:
            msg = "planning successful after %s initialization" % (i_init + 1)
            status = PlannerStatus.SUCCESS
            break
    t_total = time() - t_start

    return status, t_total, [row.tolist() for row in traj], msg


def ompl_plan(robot, group_name, active_joint_names, active_affine, target_dof_values, init_trajs):
    import moveit_msgs.msg as mm
    from planning_benchmark_common.rave_env_to_ros import rave_env_to_ros, ros_joints_to_rave
    import rospy
    ps = rave_env_to_ros(robot.GetEnv())
    msg = mm.MotionPlanRequest()
    msg.group_name = group_name
    msg.planner_id = args.ompl_planner_id
    msg.allowed_planning_time = args.max_planning_time
    c = mm.Constraints()
    joints = robot.GetJoints()

    if active_affine == 0:
        base_joint_names = []
    elif active_affine == 11:
        base_joint_names = ["world_joint/x", "world_joint/y", "world_joint/theta"]
    else:
        raise Exception
    for (name, val) in zip(active_joint_names + base_joint_names, target_dof_values):
        c.joint_constraints.append(
            mm.JointConstraint(joint_name=name, position=val, weight=1, tolerance_above=.0001, tolerance_below=.0001))

    msg.start_state = ps.robot_state
    msg.goal_constraints = [c]
    svc = get_ompl_service()
    try:
        t_start = time()
        svc_response = svc.call(msg)
        response = svc_response.motion_plan_response
        # success
        joint_names = response.trajectory.joint_trajectory.joint_names
        pts = response.trajectory.joint_trajectory.points
        base_pts = response.trajectory.multi_dof_joint_trajectory.points
        traj = []
        for i, p in enumerate(pts):
            row = ros_joints_to_rave(robot, joint_names, p.positions)[0]
            if base_pts:
                base_trans = base_pts[i].transforms[0]
                row += [base_trans.translation.x, base_trans.translation.y]
                row += [ros_quat_to_aa(base_trans.rotation)[2]]
            traj.append(row)
        traj = np.asarray(traj)
        return PlannerStatus.SUCCESS, response.planning_time, traj, ''
    except rospy.service.ServiceException, e:
        return PlannerStatus.FAIL_TIMEOUT, time() - t_start, [], 'failed due to ServiceException: ' + repr(e)


def chomp_plan(robot, group_name, active_joint_names, active_affine, target_dof_values, init_trajs):
    datadir = os.path.join(TMP_DATA_DIR, "chomp")
    n_points = args.n_steps

    assert active_affine == 0 or active_affine == 11
    use_base = active_affine == 11

    saver = openravepy.RobotStateSaver(robot)

    target_base_pose = None
    if use_base:
        with robot:
            robot.SetActiveDOFValues(target_dof_values)
            target_base_pose = openravepy.poseFromMatrix(robot.GetTransform())
        robot.SetActiveDOFs(robot.GetActiveDOFIndices(), 0)  # turn of affine dofs; chomp takes that separately
        target_dof_values = target_dof_values[:-3]  # strip off the affine part

    openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)
    m_chomp = get_chomp_module(robot.GetEnv())

    env_hash = hash_env(robot.GetEnv())
    if active_affine != 0:
        env_hash += "_aa" + str(active_affine)

    # load distance field
    j1idxs = [m.GetArmIndices()[0] for m in robot.GetManipulators()]
    for link in robot.GetLinks():
        for j1idx in j1idxs:
            if robot.DoesAffect(j1idx, link.GetIndex()):
                link.Enable(False)
    try:
        aabb_padding = 1.0 if not use_base else 3.0  # base problems should need a distance field over a larger volume
        m_chomp.computedistancefield(kinbody=robot, aabb_padding=aabb_padding,
                                     cache_filename=os.path.join(datadir, "chomp-sdf-%s.dat" % env_hash))
    except Exception, e:
        print 'Exception in computedistancefield:', repr(e)
    for link in robot.GetLinks():
        link.Enable(True)

    # build chomp command
    if args.chomp_argstr == 'comp':
        kwargs = dict(robot=robot, n_iter=1000000,
                      max_time=args.max_planning_time,
                      lambda_=100.0, no_collision_exception=True,
                      no_collision_check=True, n_points=n_points)
    elif args.chomp_argstr.startswith('hmc-seed'):
        seed = int(args.chomp_argstr[len('hmc-seed'):])
        print 'Using seed:', seed
        kwargs = dict(robot=robot, n_iter=10000,
                      max_time=args.max_planning_time,
                      lambda_=100.0, no_collision_exception=True,
                      use_momentum=True, use_hmc=True, seed=seed,
                      no_collision_check=True, n_points=n_points)
    else:
        raise RuntimeError('must be chomp-seedXXXX')

    if use_base:
        kwargs["floating_base"] = True
        kwargs["basegoal"] = np.r_[target_base_pose[4:7], target_base_pose[0:4]]

    # run chomp
    msg = ''
    t_start = time()
    status = PlannerStatus.FAIL_COLLISION
    traj = []
    if args.multi_init:
        for i_init, inittraj in enumerate(init_trajs):
            t = kwargs["starttraj"] = array_to_traj(robot, inittraj)
            try:
                rave_traj = m_chomp.runchomp(**kwargs)
                saver.Restore()  # set active dofs to original (including affine), needed for traj_to_array
                traj = traj_to_array(robot, rave_traj)
                if not traj_in_joint_limits(traj, robot):
                    status = PlannerStatus.FAIL_JOINT_VIOLATION
                elif not traj_is_safe(traj, robot):
                    status = PlannerStatus.FAIL_COLLISION
                else:
                    status = PlannerStatus.SUCCESS
                    msg = "planning successful after %s initialization" % (i_init + 1)
                    break
            except Exception, e:
                msg = "CHOMP failed with exception: %s" % str(e)
                if 'limit' in msg:
                    status = PlannerStatus.FAIL_JOINT_VIOLATION
                else:
                    status = PlannerStatus.FAIL_OTHER
                continue
    else:
        kwargs["adofgoal"] = target_dof_values
        try:
            rave_traj = m_chomp.runchomp(**kwargs)
            saver.Restore()  # set active dofs to original (including affine), needed for traj_to_array
            traj = traj_to_array(robot, rave_traj)
            if not traj_in_joint_limits(traj, robot):
                status = PlannerStatus.FAIL_JOINT_VIOLATION
            elif not traj_is_safe(traj, robot):
                status = PlannerStatus.FAIL_COLLISION
            else:
                status = PlannerStatus.SUCCESS
        except Exception, e:
            msg = "CHOMP failed with exception: %s" % str(e)
            if 'limit' in msg:
                status = PlannerStatus.FAIL_JOINT_VIOLATION
            else:
                status = PlannerStatus.FAIL_OTHER
    t_total = time() - t_start

    return status, t_total, traj, msg


def dgpmp2_plan(problemset, rave_robot, group_name, active_joint_names, active_affine, target_dof_values, init_trajs):
    global DGPMP_ITER

    start_dof_values = init_trajs[0][0].tolist()

    scene_filename = problemset["env_file"].split(".")[0] + ".scene"
    scene_filepath = osp.join(pbc.scenefile_dir, scene_filename)
    pr2_urdf_path = "/home/alt/catkin_ws/src/am-robots-python/am_robots_python/urdf/pr2/pr2.urdf"

    # Load planner config
    env_name = problemset["env_file"].split(".")[0]
    dgpmp_config_path = PLANNER_CONFIGS_DIR + "/dgpmp2_" + env_name + ".json"
    with open(dgpmp_config_path) as dgpmp_config_file:
        dgpmp_config = json.load(dgpmp_config_file)

    subdirname = "iteration_" + str(DGPMP_ITER)

    dgpmp2.configure(scene_filepath,
                     {
                         "voxel_size": 0.05,
                         "collision_padding": 0.01
                     },
                     {
                         "urdf": pr2_urdf_path,
                         "good_initial_conf": problemset["default_joint_values"],
                         "good_initial_conf_joint_names": problemset["joint_names"],
                         "ee_link_name": "r_gripper_palm_link",
                         "active_joint_names": problemset["active_joints"]
                     },
                     dgpmp_config, subdirname)

    t_start = time()

    camera_distance = 1.06
    camera_yaw = -26.8
    camera_pitch = -18.2
    camera_target_position = [0.47, -0.49, 0.76]

    traj = dgpmp2.plan({
        "active_joint_names": active_joint_names,
        "start_conf": start_dof_values,
        "goal_conf": target_dof_values,
        "vel": 0.05
    }, subdirname)

    t_total = time() - t_start

    # dgpmp2.visualize_plan(traj, "iteration_" + str(DGPMP_ITER), camera_distance, camera_yaw, camera_pitch,
    #                       camera_target_position)
    DGPMP_ITER += 1

    # Check planning result
    if not traj_in_joint_limits(traj, rave_robot):
        status = PlannerStatus.FAIL_JOINT_VIOLATION
        msg = "DGPMP2 failed due to joint violation"
    elif not traj_is_safe(traj, rave_robot):
        status = PlannerStatus.FAIL_COLLISION
        msg = "DGPMP2 failed due to collision"
    else:
        status = PlannerStatus.SUCCESS
        msg = "ok"

    return status, t_total, traj, msg


def dgpmp2_search(problemset, rave_robot, group_name, active_joint_names, active_affine, target_dof_values, init_trajs):
    global DGPMP_ITER

    start_dof_values = init_trajs[0][0].tolist()

    scene_filename = problemset["env_file"].split(".")[0] + ".scene"
    scene_filepath = osp.join(pbc.scenefile_dir, scene_filename)
    pr2_urdf_path = "/home/alt/catkin_ws/src/am-robots-python/am_robots_python/urdf/pr2/pr2.urdf"

    with open(osp.join(pbc.miscdata_dir, "dgpmp2_param_search.json")) as param_search_file:
        cfg = json.load(param_search_file)

    for search_iter in range(100):
        # Default config
        planner_config = {
            "gp_params": {
                "K_s": 0.01,
                "K_g": 0.01,
                "K_obs": 0.01,
                "K_gp": 1,
                "K_lims": 0.01
            },
            "optim_params": {
                "method": "gauss_newton",
                "reg": 1.0,
                "init_lr": 0.01,
                "max_iters": 500,
                "error_patience": 500
            }
        }

        # Sample random config
        for category in cfg.keys():
            for key, value in cfg[category].items():
                if isinstance(value, list):
                    val = random.uniform(value[0], value[1])
                else:
                    val = value
                planner_config[category][key] = val

        subdirname = "iteration_" + str(DGPMP_ITER) + "_search_" + str(search_iter)

        dgpmp2.configure(scene_filepath,
                         {
                             "voxel_size": 0.05,
                             "collision_padding": 0.01
                         },
                         {
                             "urdf": pr2_urdf_path,
                             "good_initial_conf": problemset["default_joint_values"],
                             "good_initial_conf_joint_names": problemset["joint_names"],
                             "ee_link_name": "r_gripper_palm_link",
                             "active_joint_names": problemset["active_joints"]
                         },
                         planner_config, subdirname)

        t_start = time()
        status = PlannerStatus.SUCCESS

        camera_distance = 1.06
        camera_yaw = -26.8
        camera_pitch = -18.2
        camera_target_position = [0.47, -0.49, 0.76]

        # dgpmp2.visualize_env(problemset["default_joint_values"], problemset["joint_names"], subdirname, camera_distance, camera_yaw,
        #                       camera_pitch, camera_target_position)

        traj = dgpmp2.plan({
            "active_joint_names": active_joint_names,
            "start_conf": start_dof_values,
            "goal_conf": target_dof_values,
            "vel": 0.05
        }, subdirname)

        t_total = time() - t_start

        dgpmp2.visualize_plan(traj, "iteration_" + str(DGPMP_ITER) + "_search_" + str(search_iter),
                              camera_distance, camera_yaw, camera_pitch, camera_target_position)
    DGPMP_ITER += 1

    return status, t_total, traj, "ok"


def init_env(problemset):
    env = openravepy.Environment()
    env.StopSimulation()

    robot2file = {
        "pr2": "robots/pr2-beta-static.zae"
    }

    if args.planner == "trajopt":
        if args.interactive: trajoptpy.SetInteractive(True)
        plan_func = trajopt_plan
    elif args.planner == "ompl":
        setup_ompl(env)
        plan_func = ompl_plan
    elif args.planner in ["chomp", "chomp2"]:
        setup_chomp(env)
        plan_func = chomp_plan
        # chomp needs a robot with spheres
        chomp_pr2_file = "pr2_with_spheres.robot.xml" if problemset[
                                                             "active_affine"] == 0 else "pr2_with_spheres_fullbody.robot.xml"
        robot2file["pr2"] = osp.join(pbc.envfile_dir, chomp_pr2_file)
    elif args.planner == "dgpmp2":
        plan_func = partial(dgpmp2_plan, problemset)
        # plan_func = partial(dgpmp2_search, problemset)

    env.Load(osp.join(pbc.envfile_dir, problemset["env_file"]))
    env.Load(robot2file[problemset["robot_name"]])
    robot = env.GetRobots()[0]

    if args.planner == "trajopt":
        try:
            postsetup_trajopt(env)
        except Exception:
            print "can't find ros config file. skipping self collision ignore"

    robot.SetTransform(openravepy.matrixFromPose(problemset["default_base_pose"]))
    rave_joint_names = [joint.GetName() for joint in robot.GetJoints()]
    rave_inds, rave_values = [], []
    for (name, val) in zip(problemset["joint_names"], problemset["default_joint_values"]):
        if name in rave_joint_names:
            i = rave_joint_names.index(name)
            rave_inds.append(i)
            rave_values.append(val)

    robot.SetDOFValues(rave_values, rave_inds)
    active_joint_inds = [rave_joint_names.index(name) for name in problemset["active_joints"]]
    robot.SetActiveDOFs(active_joint_inds, problemset["active_affine"])

    return env, robot, plan_func


def main():
    np.random.seed(0)
    problemset = yaml.load(args.problemfile)
    env, robot, plan_func = init_env(problemset)

    # enumerate the problems
    problem_joints = []
    problems = problemset["problems"] if args.problems is None else yaml.load(args.problems)
    for prob in problems:
        # special "all_pairs" problem type
        if "all_pairs" in prob:
            states = prob["all_pairs"]["active_dof_values"]
            for i in range(len(states)):
                for j in range(i + 1, len(states)):
                    problem_joints.append((states[i], states[j]))
            continue

        if "active_dof_values" not in prob["start"] or "active_dof_values" not in prob["goal"]:
            raise NotImplementedError
        problem_joints.append((prob["start"]["active_dof_values"], prob["goal"]["active_dof_values"]))

    # solve the problems
    results = []
    failed = []
    for i, (start, goal) in enumerate(problem_joints):
        robot.SetActiveDOFValues(start)
        init_trajs = gen_init_trajs(problemset, robot, args.n_steps, start, goal)
        status, t_total, traj, msg = plan_func(robot, problemset["group_name"], problemset["active_joints"],
                                               problemset["active_affine"], goal, init_trajs)
        success = status == PlannerStatus.SUCCESS
        print '%s[%d/%d] %s%s' % (bcolors.OKGREEN if success else bcolors.FAIL, i + 1, len(problem_joints),
                                  ('success' if success else 'FAILURE') + (': ' + msg if msg else '') + (
                                          ' (code: %s)' % status), bcolors.ENDC)
        res = {"success": success, "status": status, "time": t_total}
        if args.outfile is not sys.stdout:
            res["traj"] = traj
        results.append(res)

        if not success:
            failed.append({"start": {"active_dof_values": list(start)}, "goal": {"active_dof_values": list(goal)}})

        if args.animate_all:
            animate_traj(traj, robot)

    print "success rate: %i/%i" % (np.sum(result["success"] for result in results), len(results))
    times = np.asarray([float(result["time"]) for result in results])
    print "average time: %f" % np.mean(times[np.isfinite(times)])

    if args.record_failed_problems is not None:
        yaml.dump(failed, args.record_failed_problems)
        print 'Recorded %d failures' % len(failed)

    yaml.dump(results, args.outfile)


if __name__ == "__main__":
    main()
