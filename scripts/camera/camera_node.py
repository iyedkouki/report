#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Ensure Camera is Enabled
# sudo apt-get install python3-picamera2 python3-libcamera

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.bridge = CvBridge()
        
        # Initialize camera (try different sources if needed)
        self.cap = self.initialize_camera()
        
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera!")
            exit(1)
        
        # set a resoluiton to 1280x720 (HD 720p) 
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        rospy.loginfo(f"Camera resolution set to: 1280x720 (HD 720p)  ")
    
    def initialize_camera(self):
        # Try different camera sources (adjust based on your setup)
        # For Raspberry Pi Camera (using libcamera)
        cap = cv2.VideoCapture(0, apiPreference=cv2.CAP_V4L2)
        
        # Alternative for USB webcam if above fails
        if not cap.isOpened():
            cap = cv2.VideoCapture(0)
        
        return cap
    def capture_and_publish(self):
        rate = rospy.Rate(30)  # 30 FPS
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                try:
                    # Convert OpenCV image to ROS message
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    ros_image.header.stamp = rospy.Time.now()
                    self.image_pub.publish(ros_image)
                except Exception as e:
                    rospy.logerr(f"Error converting image: {e}")
            else:
                rospy.logwarn("Failed to capture frame")
            rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the camera node
        node = CameraNode()
        node.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
    finally:
        if hasattr(node, 'cap'):
            node.cap.release()
        rospy.loginfo("Camera node shutdown.")