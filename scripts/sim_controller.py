#!/usr/bin/env python

from roboy_simulation_msgs.srv import FloatToMetab, TestStatus
import rospy
from get_metabolic_costs import get_random_metab_csv_from_dir
import roslaunch
from metab_to_csv import file_name
from std_srvs.srv import Trigger, Empty
from gazebo_ros_muscle_interface.srv import SetMuscleActivations
from gazebo_msgs.srv import SpawnEntity, DeleteModel, GetJointProperties
from geometry_msgs.msg import Pose

start_record = False
stop_record = False
set_activation = None
flex = None
unflex = None
start_test = False
running_test = False
test_config = None


def start_recording(req):
    global start_record
    start_record = True
    return True, "worked"


def stop_recording(req):
    global stop_record
    stop_record = True
    return True, "worked"


def actuate_skeleton(req):
    global set_activation
    set_activation(req.activations)
    return True


def set_test(req):
    global flex
    global unflex
    global start_test
    global test_config
    flex = req.activations[:6]
    unflex = req.activations[6:]
    start_test = True
    test_config = "activation"
    return True


def start_pure_osim_test(req):
    # set test flag
    global start_test
    global test_config
    start_test = True
    test_config = "pure_osim"
    return True, "started"


def test_still_running(req):
    global running_test
    return running_test


def spawn_model(spawn_model_prox):
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0
    initial_pose.position.z = 0

    f = open('/home/kevin/Dokumente/NRP/GazeboRosPackages/src/exoskeleton/output/CARDSFlowExo/model.sdf', 'r')
    sdff = f.read()

    spawn_model_prox("CARDSFlowExo", sdff, "robotos_name_space", initial_pose, "world")


def delete_model(delete_model_prox):
    delete_model_prox("CARDSFlowExo")


def get_metab_server():
    global start_record
    global stop_record
    global set_activation
    global start_test
    global running_test
    global test_config

    rospy.init_node('sim_control')
    rospy.Service('sim_control/actuate_skeleton', FloatToMetab, actuate_skeleton)
    rospy.Service("sim_control/start_recording", Trigger, start_recording)
    rospy.Service("sim_control/stop_recording", Trigger, stop_recording)
    rospy.Service("sim_control/running_test", TestStatus, test_still_running)
    rospy.Service("sim_control/set_test", FloatToMetab, set_test)
    rospy.Service("sim_control/start_pure_osim_test", Trigger, start_pure_osim_test)
    set_activation = rospy.ServiceProxy("/gazebo_muscle_interface/CARDSFlowExo/set_activations", SetMuscleActivations)
    spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_entity', SpawnEntity)
    delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    get_joint_properties = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
    unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    launch_file_path = "/home/kevin/Dokumente/NRP/GazeboRosPackages/src/exoskeleton/launch/record_metab_cost.launch"
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [
        launch_file_path])
    r = rospy.Rate(5)
    print "straight into while loop"
    activation_duration = 1
    increase_timeout = 0.2
    test_stage = 0
    test_timestamp = rospy.get_time()
    act = 0.0
    while not rospy.is_shutdown():
        if start_record:
            start_record = False
            launch.start()
            rospy.loginfo("started recording")

        if stop_record:
            stop_record = False
            launch.shutdown()
            launch = roslaunch.parent.ROSLaunchParent(uuid, [
                launch_file_path])
            rospy.loginfo("stoped recording")

        if start_test:
            if test_config == "pure_osim":
                spawn_model(spawn_model_prox)
                # unpause_physics()
                rospy.loginfo("unpaused physics")
                rospy.sleep(1.0)
            test_stage, test_timestamp = start_metab_recorder(launch)

        if running_test:
            if test_config == "activation":
                if rospy.get_time() - test_timestamp > activation_duration:
                    if test_stage == 1:
                        test_stage, test_timestamp = activation_test_stage_one(set_activation, test_stage,
                                                                               test_timestamp)
                        continue

                    if test_stage == 2:
                        test_stage, test_timestamp = activation_test_stage_two(set_activation, test_stage,
                                                                               test_timestamp)
                        continue

                    if test_stage == 3:
                        launch, test_stage = activation_test_termination(launch, set_activation, uuid)
                        test_config = None
                        continue

            if test_config == "pure_osim":
                if rospy.get_time() - test_timestamp > increase_timeout:
                    position = get_joint_properties("r_shoulder").position[0]
                    print "pos: ", position, "| act: ", act
                    if position <= -1.0 or act >= 1.0:
                        act = 0.0
                        set_activation([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                        test_config = None
                        running_test = False
                        # pause_physics()
                        rospy.loginfo("paused physics")
                        launch.shutdown()
                        launch = roslaunch.parent.ROSLaunchParent(uuid, [
                            launch_file_path])
                        rospy.loginfo("stoped recording")
                        delete_model(delete_model_prox)
                    else:
                        set_activation([act, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, act, act])
                        act += 0.1
                        test_timestamp = rospy.get_time()
                    continue

        r.sleep()


def activation_test_termination(launch, set_activation, uuid):
    global running_test
    set_activation([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rospy.loginfo("set unactuated")
    launch.shutdown()
    launch = roslaunch.parent.ROSLaunchParent(uuid, [
        launch_file_path])
    rospy.loginfo("stoped recording")
    running_test = False
    test_stage = 0
    return launch, test_stage


def activation_test_stage_two(set_activation, test_stage, test_timestamp):
    set_activation(unflex)
    rospy.loginfo("set unflex activation")
    test_stage = 3
    test_timestamp = rospy.get_time()
    return test_stage, test_timestamp


def activation_test_stage_one(set_activation, test_stage, test_timestamp):
    set_activation(flex)
    rospy.loginfo("set flex activation")
    test_stage = 2
    test_timestamp = rospy.get_time()
    return test_stage, test_timestamp


def start_metab_recorder(launch):
    global start_test, running_test
    start_test = False
    running_test = True
    launch.start()
    rospy.loginfo("started recording")
    test_stage = 1
    test_timestamp = rospy.get_time()
    return test_stage, test_timestamp


if __name__ == "__main__":
    get_metab_server()
