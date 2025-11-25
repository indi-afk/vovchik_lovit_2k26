import rospy
import random
from gazebo_msgs.srv import SpawnModel, SetLinkState
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import LinkState


def add_model_to_Gazebo(model_name, model_path, pose):
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    with open(model_path, 'r') as model_file:
        model_xml = model_file.read()
    spawn_model(model_name, model_xml,"", pose, "")
    rospy.loginfo(f"Model  '{model_name}' spawned successfully!")

rospy.init_node('model_loader', anonymous=True)

model_name0 = 'main_pipe'
model_path0 = '/home/clover/vovchik_lovit_2k26/pipe_models/pipe_alfa_version_26/model.sdf' #########path
pose0 = Pose()
pose0.position.x = 3.237962
pose0.position.y = 3.877562
pose0.position.z = 0
pose0.orientation.w = 1

add_model_to_Gazebo(model_name0, model_path0, pose0)


x1 = random.random() * 3.8545 + 1
if x1 >= 3.8284: 
    y1 = 2.7475 * x1 - 6.69
else:
    y1 = x1


x2 = random.random() * 3.8545 + 1
if x2 >= 3.8284: 
    y2 = 2.7475 * x2 - 6.69
else:
    y2 = x2

while (x1 - x2) ** 2 + (y1 - y2) ** 2 < 0.75 ** 2:
    x2 = random.random() * 3.8545 + 1
    if x2 >= 3.8284: 
        y2 = 2.7475 * x2 - 6.69
    else:
        y2 = x2



x3 = random.random() * 3.8545 + 1
if x3 >= 3.8284: 
    y3 = 2.7475 * x3 - 6.69
else:
    y3 = x3



while ((x1 - x3) ** 2 + (y1 - y3) ** 2 < 0.75 ** 2) or ((x2 - x3) ** 2 + (y2 - y3) ** 2 < 0.75 ** 2):
    x3 = random.random() * 3.8545 + 1
    if x3 >= 3.8284: 
        y3 = 2.7475 * x3 - 6.69
    else:
        y3 = x3


x4 = random.random() * 3.8545 + 1
if x4 >= 3.8284: 
    y4 = 2.7475 * x4 - 6.69
else:
    y4 = x4


while ((x1 - x4) ** 2 + (y1 - y4) ** 2 < 0.75 ** 2) or ((x2 - x4) ** 2 + (y2 - y4) ** 2 < 0.75 ** 2) or ((x3 - x4) ** 2 + (y3 - y4) ** 2 < 0.75 ** 2):
    x4 = random.random() * 3.8545 + 1
    if x4 >= 3.8284: 
        y4 = 2.7475 * x4 - 6.69
    else:
        y4 = x4


x5 = random.random() * 3.8545 + 1
if x5 >= 3.8284: 
    y5 = 2.7475 * x5 - 6.69
else:
    y5 = x5

while ((x1 - x5) ** 2 + (y1 - y5) ** 2 < 0.75 ** 2) or ((x2 - x5) ** 2 + (y2 - y5) ** 2 < 0.75 ** 2) or ((x3 - x5) ** 2 + (y3 - y5) ** 2 < 0.75 ** 2)  or ((x4 - x5) ** 2 + (y4 - y5) ** 2 < 0.75 ** 2):
    x5 = random.random() * 3.8545 + 1
    if x5 >= 3.8284: 
        y5 = 2.7475 * x5 - 6.69
    else:
        y5 = x5


model_path_45 = '/home/clover/vovchik_lovit_2k26/pipe_models/secondary_pipe_26/model.sdf' ##############path
model_path_70 = '/home/clover/vovchik_lovit_2k26/pipe_models/third_pipe/model.sdf'

pose1 = Pose()
pose1.position.z = 0
pose1.orientation.w = 1
if 1 <= x1 <= 1 + 2*(2**0.5):
    pose1.position.x = x1 - 1/2/(2**0.5) - 0.15
    pose1.position.y = y1 + 1/2/(2**0.5) 
    add_model_to_Gazebo('secondary_pipe_1', model_path_45, pose1)
else:
    pose1.position.x = x1 - 0.5 * 0.93969 - 0.15
    pose1.position.y = y1 + 0.5 * 0.34202 
    add_model_to_Gazebo('secondary_pipe_1', model_path_70, pose1)



pose2 = Pose()
pose1.position.z = 0
pose1.orientation.w = 1
if 1 <= x2 <= 1 + 2*(2**0.5):
    pose2.position.x = x2 - 1/2/(2**0.5) - 0.15
    pose2.position.y = y2 + 1/2/(2**0.5) 
    add_model_to_Gazebo('secondary_pipe_2', model_path_45, pose2)
else:
    pose2.position.x = x2 - 0.5 * 0.93969 - 0.15
    pose2.position.y = y2 + 0.5 * 0.34202 
    add_model_to_Gazebo('secondary_pipe_2', model_path_70, pose2)


pose3 = Pose()
pose3.position.z = 0
pose3.orientation.w = 1
if 1 <= x3 <= 1 + 2*(2**0.5):
    pose3.position.x = x3 - 1/2/(2**0.5) - 0.15
    pose3.position.y = y3 + 1/2/(2**0.5) 
    add_model_to_Gazebo('secondary_pipe_3', model_path_45, pose3)
else:
    pose3.position.x = x3 - 0.5 * 0.93969 - 0.15
    pose3.position.y = y3 + 0.5 * 0.34202 
    add_model_to_Gazebo('secondary_pipe_3', model_path_70, pose3)


pose4 = Pose()
pose4.position.z = 0
pose4.orientation.w = 1
if 1 <= x4 <= 1 + 2*(2**0.5):
    pose4.position.x = x4 - 1/2/(2**0.5) - 0.15
    pose4.position.y = y4 + 1/2/(2**0.5) 
    add_model_to_Gazebo('secondary_pipe_4', model_path_45, pose4)
else:
    pose4.position.x = x4 - 0.5 * 0.93969 - 0.15
    pose4.position.y = y4 + 0.5 * 0.34202 
    add_model_to_Gazebo('secondary_pipe_4', model_path_70, pose4)



pose5 = Pose()
pose5.position.z = 0
pose5.orientation.w = 1
if 1 <= x5 <= 1 + 2*(2**0.5):
    pose5.position.x = x5 - 1/2/(2**0.5) - 0.15
    pose5.position.y = y5 + 1/2/(2**0.5)
    add_model_to_Gazebo('secondary_pipe_5', model_path_45, pose5)
else:
    pose5.position.x = x5 - 0.5 * 0.93969 - 0.15
    pose5.position.y = y5 + 0.5 * 0.34202 
    add_model_to_Gazebo('secondary_pipe_5', model_path_70, pose5)
