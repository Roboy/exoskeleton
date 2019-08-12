import rospy
from gazebo_msgs.srv import DeleteModel

rospy.init_node("terminator")
delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
delete_model_prox("some_robo_name")