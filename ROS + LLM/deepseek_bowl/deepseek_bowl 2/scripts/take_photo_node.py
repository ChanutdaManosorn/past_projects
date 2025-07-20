#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Trigger, TriggerResponse

class TakePhotoNode:
    def __init__(self):
        rospy.init_node('take_photo_node', anonymous=False)

        self.bridge = CvBridge()
        self.image_received = False
        self.image = None

        img_topic = "/camera/rgb/image_raw"
        rospy.Subscriber(img_topic, Image, self.image_callback)

        rospy.loginfo("🎥 waiting for photo from the camera...")
        rospy.sleep(2)

        self.img_title = rospy.get_param('~image_title', 'bowl.jpg')

        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(10)  # 10 sec
        while not self.image_received and rospy.Time.now() < timeout:
            rospy.loginfo_throttle(2, "📷 waiting for a photo...")
            rospy.sleep(0.5)

        if self.image_received:
            cv2.imwrite(self.img_title, self.image)
            rospy.loginfo(f"✅ photo is saved: {self.img_title}")
        else:
            rospy.logwarn("❌ no photo received")
        
        self.srv = rospy.Service('~take_photo', Trigger, self.handle_take_photo)
        rospy.spin()

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(f"failure in converting photo: {e}")
    
    def handle_take_photo(self, req):
        rospy.loginfo("📸 take_photo service triggered!")
        if self.image_received:
            cv2.imwrite(self.img_title, self.image)
            return TriggerResponse(success=True, message="Photo saved.")
        else:
            return TriggerResponse(success=False, message="No image received.")

if __name__ == '__main__':
    try:
        TakePhotoNode()
    except rospy.ROSInterruptException:
        pass
