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
    flex = req.activations[:6]
    unflex = req.activations[6:]
    start_test = True
    return True


def test_still_running(req):
    global running_test
    return running_test


def get_metab_server():
    global start_record
    global stop_record
    global set_activation
    global start_test
    global running_test

    rospy.init_node('sim_control')
    rospy.Service('sim_control/actuate_skeleton', FloatToMetab, actuate_skeleton)
    rospy.Service("sim_control/start_recording", Trigger, start_recording)
    rospy.Service("sim_control/stop_recording", Trigger, stop_recording)
    rospy.Service("sim_control/running_test", TestStatus, test_still_running)
    rospy.Service("sim_control/set_test", FloatToMetab, set_test)
    set_activation = rospy.ServiceProxy("/gazebo_muscle_interface/arm26/set_activations", SetMuscleActivations)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [
        "/home/kevin/Dokumente/NRP/GazeboRosPackages/src/exoskeleton/launch/record_metab_cost.launch"])
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
                "/home/kevin/Dokumente/NRP/GazeboRosPackages/src/exoskeleton/launch/record_metab_cost.launch"])
            rospy.loginfo("stoped recording")

        if start_test:
            start_test = False
            running_test = True
            launch.start()
            rospy.loginfo("started recording")
            test_stage = 1
            test_timestamp = rospy.get_time()

        if running_test:
            if rospy.get_time() - test_timestamp > activation_duration:
                if test_stage == 1:
                    set_activation(flex)
                    rospy.loginfo("set flex activation")
                    test_stage = 2
                    test_timestamp = rospy.get_time()
                    continue

                if test_stage == 2:
                    set_activation(unflex)
                    rospy.loginfo("set unflex activation")
                    test_stage = 3
                    test_timestamp = rospy.get_time()
                    continue

                if test_stage == 3:
                    set_activation([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    rospy.loginfo("set unactuated")
                    launch.shutdown()
                    launch = roslaunch.parent.ROSLaunchParent(uuid, [
                        "/home/kevin/Dokumente/NRP/GazeboRosPackages/src/exoskeleton/launch/record_metab_cost.launch"])
                    rospy.loginfo("stoped recording")
                    running_test = False
                    test_stage = 0
                    continue

        r.sleep()


if __name__ == "__main__":
    get_metab_server()
