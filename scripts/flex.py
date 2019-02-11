#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64


def flex():
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
    rospy.init_node('flex', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub_BIClong.publish(1.0)
        pub_BICshort.publish(1.0)
        pub_BRA.publish(1.0)
        pub_TRIlat.publish(0.0)
        pub_TRIlong.publish(0.0)
        pub_TRImed.publish(0.0)
        rate.sleep()


if __name__ == '__main__':
    try:
        flex()
    except rospy.ROSInterruptException:
        pass