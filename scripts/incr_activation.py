import rospy
from gazebo_ros_muscle_interface.srv import SetMuscleActivations
import numpy as np
from gazebo_msgs.srv import GetJointProperties

if __name__ == "__main__":
    rospy.init_node("incr_activation")
    set_activation = rospy.ServiceProxy("/gazebo_muscle_interface/CARDSFlowExo/set_activations", SetMuscleActivations)
    get_joint_properties = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
    for i in np.arange(0.0, 1.0, 0.1):
        set_activation([i, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i, i])
        rospy.sleep(0.2)
        position = get_joint_properties("r_shoulder").position[0]
        print i, " : ", position, type(position)
        if position <= -1.0:
            break

    set_activation([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

