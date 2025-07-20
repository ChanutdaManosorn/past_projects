#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TakePhotoNode:
    def __init__(self):
        rospy.init_node('take_photo_node', anonymous=False)

        self.bridge = CvBridge()
        self.image_received = False
        self.image = None

        img_topic = "/camera/rgb/image_raw"
        rospy.Subscriber(img_topic, Image, self.image_callback)

        rospy.loginfo("🎥 wait for camera...")
        rospy.sleep(2)

        # Image title (parameter or default)
        self.img_title = rospy.get_param('~image_title', 'bowl.jpg')

        # wait till get photo
        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(10)  # 10 secs
        while not self.image_received and rospy.Time.now() < timeout:
            rospy.loginfo_throttle(2, "📷 wait for photo...")
            rospy.sleep(0.5)

        if self.image_received:
            cv2.imwrite(self.img_title, self.image)
            rospy.loginfo(f"✅ saved photo: {self.img_title}")
        else:
            rospy.logwarn("❌ no photo comes")

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(f"failed converting photo: {e}")

if __name__ == '__main__':
    try:
        TakePhotoNode()
    except rospy.ROSInterruptException:
        pass
