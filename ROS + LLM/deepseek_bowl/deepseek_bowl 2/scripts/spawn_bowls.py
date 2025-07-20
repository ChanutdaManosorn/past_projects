# spawn_bowls.py

import rospy, random
import yaml
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

ROOM_LOCATION = {
    "room1": [7.5, 5.5, 2.5, 0],
    "room2": [7.5, 0, 5, -5.5],
    "room3": [2.5, 5.5, 0, 1],
    "room4": [0, 5.5, -5, 0],
    "room5": [-5, 5.5, -7.5, 1],
    "room6": [-5, 1, -7.5, -4],
    "hall": [2.5, 1, 0, 0]    
}

def spawn_bowl(name, x, y):
    with open(rospkg.RosPack().get_path('deepseek_world_search') + '/models/bowl/model.sdf', 'r') as f:
        model_xml = f.read()
    pose = Pose(position=Point(x=x, y=y, z=0.0), orientation=Quaternion(w=1.0))
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn(name, model_xml, "/", pose, "world")

def spawn_charger(name, x, y):
    with open(rospkg.RosPack().get_path('deepseek_world_search') + '/models/turtlebot3_dock/model.sdf', 'r') as f:
        model_xml = f.read()
    pose = Pose(position=Point(x=x, y=y, z=0.0), orientation=Quaternion(w=1.0))
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn(name, model_xml, "", pose, "world")

if __name__ == '__main__':
    rospy.init_node('spawn_bowls')
    num_bowls = random.randint(1, 2)
    selected_rooms = random.sample(list(ROOM_LOCATION.keys()), num_bowls)
    
    bowl_cnt = 0
    for k in selected_rooms:
        x1 = ROOM_LOCATION[k][0]
        y1 = ROOM_LOCATION[k][1]
        x2 = ROOM_LOCATION[k][2]
        y2 = ROOM_LOCATION[k][3]        
        x, y = random.uniform(x1, x2), random.uniform(y1, y2)            
        print(f"bowl:{bowl_cnt} room:{k} x:{x} y:{y}")
        spawn_bowl(f"bowl_{bowl_cnt}", x, y)
        bowl_cnt = bowl_cnt+1
    spawn_charger('charger', -5, 1.25)
