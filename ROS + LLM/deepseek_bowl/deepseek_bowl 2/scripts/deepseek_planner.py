#!/usr/bin/env python3
import rospy
import requests
from std_msgs.msg import String
from dotenv import load_dotenv
import os
from geometry_msgs.msg import PoseWithCovarianceStamped

load_dotenv()
API_KEY = os.getenv("DEEPSEEK_API_KEY")
DEEPSEEK_URL = "https://api.deepseek.com/v1/chat/completions"
ROS_TOPIC = "/llm_command"

class DeepSeekPlanner:
    def __init__(self):
        rospy.init_node("deepseek_planner")

        # Current position
        self.current_pos = (-4.5, 1.25)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        # Publisher to robot executor
        self.cmd_pub = rospy.Publisher("/llm_commands", String, queue_size=10)

        # Timer to run planning periodically
        rospy.Timer(rospy.Duration(10), self.plan_and_send)

    def pose_callback(self, msg):
        self.current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        rospy.loginfo_throttle(5, f"[Planner] Current Position: {self.current_pos}")

    def plan_and_send(self, event):
        x, y = self.current_pos
        direction = "forward"
        step_size = 0.5
        tolerance = 0.2
        top_left_prohibit_x = 5.0
        top_left_prohibit_y = 0.0
        bottom_right_prohibit_x = -5.0
        bottom_right_prohibit_y = -5.5

        prompt = (
            f"You are a robot planner. The robot starts at position ({x}, {y}) "
            f"and wants to move toward {direction} by {step_size} meters per step. "
            f"Simulate moving forward in a straight line until it hits a wall (about 20 steps max). "
            f"Return ONLY a list of waypoints like: [[x1, y1], [x2, y2], ...] and do NOT include text. "
            f"Also please avoid moving to positions with length close to zero with tolerance {tolerance}. "
            f"If robot gets stuck, perform a recovery by doing a u-turn with gap 0.2 between lines. "
            f"Prohibited area top-left: ({top_left_prohibit_x}, {top_left_prohibit_y}), "
            f"bottom-right: ({bottom_right_prohibit_x}, {bottom_right_prohibit_y}). "
            f"If the robot hits a bowl, send 'take_photo'. Otherwise, send 'move' followed by the waypoints."
        )

        headers = {
            "Authorization": f"Bearer {API_KEY}",
            "Content-Type": "application/json"
        }

        data = {
            "model": "deepseek-coder",
            "messages": [
                {"role": "system", "content": "You are a navigation robot planner."},
                {"role": "user", "content": prompt}
            ]
        }

        try:
            rospy.loginfo("Querying DeepSeek to generate plan...")
            response = requests.post(DEEPSEEK_URL, headers=headers, json=data, timeout=20)
            response.raise_for_status()
            content = response.json()["choices"][0]["message"]["content"]
            rospy.loginfo("DeepSeek Response:\n" + content)

            if "take_photo" in content:
                self.cmd_pub.publish("take_photo")
            elif content.startswith("[["):
                cmd = f"move:{content}"
                self.cmd_pub.publish(cmd)
            else:
                rospy.logwarn("Unexpected DeepSeek output: " + content)

        except Exception as e:
            rospy.logerr("DeepSeek API call failed: %s", e)

if __name__ == "__main__":
    try:
        planner = DeepSeekPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
