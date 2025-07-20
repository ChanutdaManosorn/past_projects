#!/usr/bin/env python3
import rospy
import ast
import actionlib
import subprocess
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
import tf
from tf.transformations import quaternion_from_euler
import json
from geometry_msgs.msg import PoseStamped
import threading
import cv2
import numpy as np

ROS_TOPIC = "/planner/command"

class LLMCommandListener:
    def __init__(self):
        rospy.init_node("ros_command_executor")        
        self.sub = rospy.Subscriber(ROS_TOPIC, String, self.callback)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")
        
        self.status_pub = rospy.Publisher("/executor/status", String, queue_size=10) 
        
        # Battery simulation
        self.battery_level = 100
        self.battery_lock = threading.Lock()

        # Start battery drain thread
        threading.Thread(target=self.battery_drain_loop, daemon=True).start()
        
    def battery_drain_loop(self):
        rate = rospy.Rate(1/60.0)  # 1 cycle per minute
        while not rospy.is_shutdown():
            with self.battery_lock:
                if self.battery_level > 0:
                    self.battery_level -= 1
                else:
                    self.battery_level = 0
            self.publish_status(bowl_detected=False)  # periodic status update without bowl detection
            rate.sleep()
            
    def publish_status(self, bowl_detected=False):
        status_msg = {
            "battery": self.battery_level,
            "bowl_detected": bowl_detected
        }
        self.status_pub.publish(json.dumps(status_msg))

    def callback(self, msg):
        print(f"Message:{msg}")
        try:
            data = json.loads(msg.data)
            command = data.get("command")

            if command == "move":
                waypoints = data.get("waypoints", [])
                rospy.loginfo("Received waypoints: %s", waypoints)
                self.move_through_waypoints(waypoints)
            elif command == "take_photo":
                rospy.loginfo("Command to take photo received (simulate)")
                self.take_photo()
        except Exception as e:
            rospy.logerr("Failed to process planner command: %s", e)
            
        # data = msg.data.strip()
        # print(f"!!!callback data {data}")
        # if data == "take_photo":
        #     rospy.loginfo("Simulated: Taking photo 📸")
        #     self.take_photo()
        #     return

        # if data.startswith("move:"):
        #     try:
        #         waypoints = json.loads(data[len("move:"):])
        #         if isinstance(waypoints, list):
        #             for pt in waypoints:
        #                 self.send_goal(pt[0], pt[1])
        #         else:
        #             rospy.logwarn("Invalid waypoint format")
        #     except Exception as e:
        #         rospy.logerr("Failed to parse waypoints: %s", e)
        
    def move_through_waypoints(self, waypoints):
        for point in waypoints:
            x, y = point
            self.send_goal(x, y)
            # self.send_move_base_goal(x, y)
            self.client.wait_for_result()
            rospy.sleep(1)

    # def send_move_base_goal(self, x, y, yaw=0.0):
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = "map"
    #     goal.target_pose.header.stamp = rospy.Time.now()

    #     goal.target_pose.pose.position.x = x
    #     goal.target_pose.pose.position.y = y
    #     q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    #     goal.target_pose.pose.orientation.x = q[0]
    #     goal.target_pose.pose.orientation.y = q[1]
    #     goal.target_pose.pose.orientation.z = q[2]
    #     goal.target_pose.pose.orientation.w = q[3]

    #     rospy.loginfo(f"Sending goal to ({x}, {y})")
    #     self.client.send_goal(goal)        
                
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CV Bridge Error: %s", e)
            return

        if self.detect_bowl(cv_image):
            rospy.loginfo("Bowl detected! Notifying planner.")
            self.publish_status(bowl_detected=True)

    def detect_bowl(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                return True
        return False                
                
    def take_photo(self):
        rospy.loginfo("📸 กำลังถ่ายภาพ...")
        try:
            subprocess.call(["rosrun", "ros_discovery", "take_photo_node.py"])
        except Exception as e:
            rospy.logerr(f"❌ ถ่ายรูปไม่สำเร็จ: {e}")

    def send_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

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
