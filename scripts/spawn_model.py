import rospy
from gazebo_msgs.srv import SpawnEntity, DeleteModel
from geometry_msgs.msg import Pose

DOCUMENT_PATH = "kevin/Dokumente"

rospy.init_node('insert_object', log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 0

f = open('/home/%s/NRP/GazeboRosPackages/src/exoskeleton/output/CARDSFlowExo/model.sdf' % DOCUMENT_PATH, 'r')
sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_entity')
spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_entity', SpawnEntity)
spawn_model_prox("CARDSFlowExo", sdff, "robotos_name_space", initial_pose, "world")
