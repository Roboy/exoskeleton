#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from roboy_simulation_msgs.msg import MetabolicCost
import sys

if len(sys.argv) < 2:
    print "verkackt brudi"
    exit(1)
file_name = sys.argv[1]



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "array length %d", len(data.umbergerTotal))
    with open(file_name, "a+") as csv_file:
        csv_file.write(str(data.simTimestamp) + "; " + str(data.umbergerTotal[0]) + "; " +
                       str(data.umbergerTotal[1]) + "; " +
                       str(data.umbergerTotal[2]) + "; " + str(data.umbergerTotal[3]) + "; " +
                       str(data.umbergerTotal[4]) + "; " + str(data.umbergerTotal[5]) + "\n")


def listener():
    # create the file and fill in the header
    with open(file_name, "w") as csv_file:
        csv_file.write("simTimestamp; umbergerTotal_m0; umbergerTotal_m1; umbergerTotal_m2; umbergerTotal_m3; umbergerTotal_m4; umbergerTotal_m5 \n")

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('metab_to_csv', anonymous=True)

    rospy.Subscriber("/metabolic_plugin/metabolic_cost", MetabolicCost, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()