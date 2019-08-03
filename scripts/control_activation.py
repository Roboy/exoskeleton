#!/usr/bin/env python

import csv
import pprint
import rospy
from gazebo_msgs.srv import GetWorldProperties
from std_msgs.msg import Float64


def main(mult_rate=1):
    control_data = []
    with open("../resource/raw_arm26_controls.csv") as control_csv:
        control_line = csv.DictReader(control_csv, delimiter="\t")

        for line in control_line:
            control_data.append(line)

    #pprint.pprint(control_data)

    # /gazebo_muscle_interface/arm26/BIClong/cmd_activation
    pub_BIClong = rospy.Publisher('/gazebo_muscle_interface/arm26/BIClong/cmd_activation', Float64, queue_size=10)

    # /gazebo_muscle_interface/arm26/BICshort/cmd_activation
    pub_BICshort = rospy.Publisher('/gazebo_muscle_interface/arm26/BICshort/cmd_activation', Float64, queue_size=10)

    # /gazebo_muscle_interface/arm26/BRA/cmd_activation
    pub_BRA = rospy.Publisher('/gazebo_muscle_interface/arm26/BRA/cmd_activation', Float64, queue_size=10)

    # /gazebo_muscle_interface/arm26/TRIlat/cmd_activation
    pub_TRIlat = rospy.Publisher('/gazebo_muscle_interface/arm26/TRIlat/cmd_activation', Float64, queue_size=10)

    # /gazebo_muscle_interface/arm26/TRIlong/cmd_activation
    pub_TRIlong = rospy.Publisher('/gazebo_muscle_interface/arm26/TRIlong/cmd_activation', Float64, queue_size=10)

    # /gazebo_muscle_interface/arm26/TRImed/cmd_activation
    pub_TRImed = rospy.Publisher('/gazebo_muscle_interface/arm26/TRImed/cmd_activation', Float64, queue_size=10)

    get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    rospy.init_node('control_activation', anonymous=True)

    rate = rospy.Rate(100)  # 100hz
    for data in control_data:
        not_ready = True
        while not_ready:
            world_props = get_world_properties()
            if world_props.sim_time > float(data["time"]):
                not_ready = False
            else:
                rate.sleep()

        print "publishing data for ", data["time"]
        pub_BIClong.publish(float(data["BIClong"]) * mult_rate)
        pub_BICshort.publish(float(data["BICshort"]) * mult_rate)
        pub_BRA.publish(float(data["BRA"]) * mult_rate)
        pub_TRIlat.publish(float(data["TRIlat"]) * mult_rate)
        pub_TRIlong.publish(float(data["TRIlong"]) * mult_rate)
        pub_TRImed.publish(float(data["TRImed"]) * mult_rate)

    print "finished the set"



if __name__ == "__main__":
    main()
