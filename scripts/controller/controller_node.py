#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
import firebase_admin
from firebase_admin import credentials, db

class ControllerNode:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        # Subscribe to waste detected status from waste_tracker_node
        rospy.Subscriber('/waste_detected', Bool, self.waste_detected_callback)
        # Subscribe to collection status from ai_waste_collector
        rospy.Subscriber('/collection_status', String, self.collection_status_callback)
        # Subscribe to navigation status from navigation_node
        rospy.Subscriber('/navigation_status', String, self.navigation_status_callback)
        # Publisher for navigation control
        self.navigation_control_pub = rospy.Publisher('/navigation_control', String, queue_size=10)
        # Publisher for collection control
        self.collection_control_pub = rospy.Publisher('/collection_control', String, queue_size=10)
        self.webcontroler_pub = rospy.Publisher('/webcontroler', String, queue_size=10)
        # Firebase setup
        try:
            cred = credentials.Certificate('/home/iyed71/asv_ws/credinales/auth.json')
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://oceancleaner-eb432-default-rtdb.firebaseio.com'
            })
            rospy.loginfo("Firebase initialized successfully in ControllerNode.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Firebase in ControllerNode: {e}")
            raise
        self.auto_ref = db.reference('robot_control/auto')
        self.auto_mode = True
        self.auto_ref.listen(self.firebase_auto_callback)
        self.waste_detected = False
        self.collection_status = 'pause'
        self.navigation_status = 'pause'
        self.collection_control_pub.publish('end_collection')
        self.navigation_control_pub.publish('navigate')


        rospy.loginfo("Controller Node started and subscribed to all status topics.")

    def firebase_auto_callback(self, event):
        data = event.data
        if data is not None:
            self.auto_mode = bool(data)
            rospy.loginfo(f"[Controller] Auto mode set to: {self.auto_mode}")

    def control_logic(self):
        # Check auto mode from Firebase
        if not self.auto_mode:
            self.collection_control_pub.publish('pause')
            self.navigation_control_pub.publish('pause')
            self.webcontroler_pub.publish('resume')
            rospy.loginfo("[Controller] Manual mode: Pausing collection/navigation, resuming web control.")
            return
        else:
            self.webcontroler_pub.publish('pause')
            rospy.loginfo("[Controller] Auto mode: Pausing web control, running autonomous logic.")
        # If waste is detected and collection is not already started, start collection and pause navigation
        if self.waste_detected :
            self.collection_control_pub.publish('start_collection')
            self.navigation_control_pub.publish('pause')
            rospy.loginfo("[Controller] Start waste collection and Pause navigation.")
        
        elif not self.waste_detected or self.collection_status == 'pause': #if you want to run both noeuds navigataion and collection  add  'and' not " OR " condition 
            self.navigation_control_pub.publish('navigate')
            rospy.loginfo("[Controller] Resume navigation.")



    def run(self):
        rate = rospy.Rate(2)  # 2 Hz
        while not rospy.is_shutdown():
            self.control_logic()
            rate.sleep()

    def waste_detected_callback(self, msg):
        self.waste_detected = msg.data
        rospy.loginfo(f"[Controller] Waste detected: {self.waste_detected}")

    def collection_status_callback(self, msg):
        self.collection_status = msg.data
        rospy.loginfo(f"[Controller] Collection status: {self.collection_status}")

    def navigation_status_callback(self, msg):
        self.navigation_status = msg.data
        rospy.loginfo(f"[Controller] Navigation status: {self.navigation_status}")

if __name__ == '__main__':
    try:
        node = ControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
