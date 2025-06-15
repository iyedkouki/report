#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest
import firebase_admin
from firebase_admin import credentials, db
import threading
import time
from std_msgs.msg import String

class WebFanController:
    def __init__(self):
        rospy.init_node('web_fan_controller', anonymous=True)
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        rospy.wait_for_service('/gazebo/clear_body_wrenches')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.clear_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
        self.max_torque = 2.0
        self.fan_right = "boatcleaningc::fandroit"
        self.fan_left = "boatcleaningc::fangauche"
        
        
        # Firebase setup
        try:
            cred = credentials.Certificate('/home/iyed71/asv_ws/credinales/auth.json')
            firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://oceancleaner-eb432-default-rtdb.firebaseio.com'
            })
            rospy.loginfo("Firebase initialized successfully")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Firebase: {e}")
            raise

        self.webcontroler_state = 'resume'
        rospy.Subscriber('/webcontroler', String, self.webcontroler_callback)
        self.control_ref = db.reference('robot_control')
        self.control_ref.listen(self.firebase_callback)
        rospy.loginfo("WebFanController node started and listening to Firebase.")

    def apply_torque(self, link_name, torque):
        try:
            torque = max(min(torque, self.max_torque), -self.max_torque)
            self.clear_wrench(link_name)
            if abs(torque) > 0.001:
                req = ApplyBodyWrenchRequest()
                req.body_name = link_name
                req.reference_frame = "world"
                req.wrench.torque.x = torque
                req.duration = rospy.Duration(-1)
                self.apply_wrench(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to apply torque on {link_name}: {e}")

    def firebase_callback(self, event):
        data = event.data
        if not data:
            return
        value = float(data.get('value', 0.0))  # -2.0 (forward) to 2.0 (backward)
        rotation = float(data.get('rotation', 0.0))  # -2.0 (left) to 2.0 (right)
        # Calculate fan torques
        # Forward/backward: both fans same
        # Rotation: left fan minus, right fan plus for right turn, and vice versa
        left_torque = value - rotation
        right_torque = value + rotation
        self.apply_torque(self.fan_left, left_torque)
        self.apply_torque(self.fan_right, right_torque)
        rospy.loginfo(f"Firebase command: value={value}, rotation={rotation} -> left={left_torque}, right={right_torque}")

    def webcontroler_callback(self, msg):
        self.webcontroler_state = msg.data
        rospy.loginfo(f"[WebFanController] Webcontroler state: {self.webcontroler_state}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.webcontroler_state == 'pause':
                rospy.loginfo("[WebFanController] Paused by Waiting for 'resume'...")
                while self.webcontroler_state == 'pause' and not rospy.is_shutdown():
                    rate.sleep()
                rospy.loginfo("[WebFanController] Resumed ")
            rate.sleep()

if __name__ == '__main__':
    try:
        node = WebFanController()
        node.run()
    except rospy.ROSInterruptException:
        pass
