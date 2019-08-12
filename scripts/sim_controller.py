#!/usr/bin/env python

from roboy_simulation_msgs.srv import FloatToMetab, TestStatus
import rospy
from get_metabolic_costs import get_random_metab_csv_from_dir
import roslaunch
from metab_to_csv import file_name
from std_srvs.srv import Trigger
from gazebo_ros_muscle_interface.srv import SetMuscleActivations

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
    set_activation = rospy.ServiceProxy("/gazebo_muscle_interface/arm26/set_activations", SetMuscleActivations)
    # todo: find right service type
    get_joint_properties = rospy.ServiceProxy("/gazebo/get_joint_properties", WhateverSrv)
    # todo: find right final angle value
    neat_value = 0.5

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [
        "/home/roboy/Documents/NRP/GazeboRosPackages/src/exoskeleton/launch/record_metab_cost.launch"])
    r = rospy.Rate(5)
    print "straight into while loop"
    activation_duration = 1
    test_stage = 0
    test_timestamp = rospy.get_time()
    while not rospy.is_shutdown():
        if start_record:
            start_record = False
            launch.start()
            rospy.loginfo("started recording")

        if stop_record:
            stop_record = False
            launch.shutdown()
            launch = roslaunch.parent.ROSLaunchParent(uuid, [
                "/home/roboy/Documents/NRP/GazeboRosPackages/src/exoskeleton/launch/record_metab_cost.launch"])
            rospy.loginfo("stoped recording")

        if start_test:
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
                if test_stage == 1:
                    if rospy.get_time() - test_timestamp > activation_duration:
                        # todo: find right activation list
                        neat_activation = []
                        set_activation(neat_activation)
                        test_stage = 2
                        continue
                if test_stage == 2:
                    # todo: right variable usage of property
                    if get_joint_properties("r_shoulder") >= neat_value:
                        set_activation([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                        stop_record = True
                        test_config = None
                        running_test = False
                        test_stage = 0

        r.sleep()


def activation_test_termination(launch, set_activation, uuid):
    global running_test
    set_activation([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rospy.loginfo("set unactuated")
    launch.shutdown()
    launch = roslaunch.parent.ROSLaunchParent(uuid, [
        "/home/roboy/Documents/NRP/GazeboRosPackages/src/exoskeleton/launch/record_metab_cost.launch"])
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
