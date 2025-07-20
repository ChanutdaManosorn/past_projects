#!/usr/bin/env python3
import rospy
import ast
import actionlib
import subprocess
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import json
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class LLMCommandListener:
    def __init__(self):
        rospy.init_node("ros_command_executor")
        self.sub = rospy.Subscriber("/llm_commands", String, self.callback)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

    def callback(self, msg):
        data = msg.data.strip()
        if data == "take_photo":
            rospy.loginfo("Simulated: Taking photo 📸")
            self.take_photo()
            return

        if data.startswith("move:"):
            try:
                waypoints = json.loads(data[len("move:"):])
                if isinstance(waypoints, list):
                    for pt in waypoints:
                        self.send_goal(pt[0], pt[1])
                else:
                    rospy.logwarn("Invalid waypoint format")
            except Exception as e:
                rospy.logerr("Failed to parse waypoints: %s", e)
                
    # def take_photo(self):
    #     rospy.loginfo("📸 taking a picture...")
    #     try:
    #         subprocess.call(["rosrun", "ros_discovery", "take_photo_node.py"])
    #     except Exception as e:
    #         rospy.logerr(f"❌ taking picture failed: {e}")
    def take_photo(self):
        rospy.wait_for_service('/take_photo_node/take_photo')
        try:
            take = rospy.ServiceProxy('/take_photo_node/take_photo', Trigger)
            resp = take()
            rospy.loginfo(f"✅ take_photo response: {resp.message}")
        except Exception as e:
            rospy.logerr(f"❌ calling take_photo service failed: {e}")

    def send_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # Face forward

        rospy.loginfo(f"Sending goal: ({x}, {y})")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Goal reached.")

if __name__ == "__main__":
    try:
        LLMCommandListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
