#!/usr/bin/env python3

import rospy
from asv_wave_sim_gazebo.msg import WasteDetection
from std_msgs.msg import String
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest
import time

class WasteCollectionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('waste_collection_node', anonymous=True)
        
        # Wait for Gazebo services
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        rospy.wait_for_service('/gazebo/clear_body_wrenches')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.clear_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
        
        # State variables
        self.is_collecting = False
        self.last_waste_detection_time = None
        self.waste_detection_timeout = 5.0  # 5 seconds timeout
        
        # Fan control parameters
        self.forward_speed = -0.8
        self.strong_rotation = -0.4
        self.gentle_rotation = -0.2
        self.fan_right = "boatcleaningc::fandroit"
        self.fan_left = "boatcleaningc::fangauche"
        
        # Publishers and Subscribers
        self.collection_status_pub = rospy.Publisher('/collection_status', String, queue_size=1)
        rospy.Subscriber('/collection_control', String, self.collection_control_callback)
        rospy.Subscriber('/waste_detection', WasteDetection, self.waste_detection_callback)
        
        rospy.loginfo("Waste Collection Node started")
        
        # this 2 line addded if tehre is any command still going to the gazebo {didint test it }
        self.apply_torque(self.fan_right, 0.0)
        self.apply_torque(self.fan_left, 0.0)

    def apply_torque(self, link_name, torque):
        """Apply torque to a specific fan"""
        try:
            self.clear_wrench(link_name)
            if abs(torque) > 0.001:
                req = ApplyBodyWrenchRequest()
                req.body_name = link_name
                req.reference_frame = "world"
                req.wrench.torque.x = torque
                req.duration = rospy.Duration(-1)
                self.apply_wrench(req)
                rospy.loginfo(f"Applied torque {torque} on {link_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to apply torque on {link_name}: {e}")

    def stop_fans(self):
        """Stop both fans"""
        self.apply_torque(self.fan_right, 0.0)
        self.apply_torque(self.fan_left, 0.0)
        rospy.loginfo("Boat stopped.")

    def move_forward(self):
        """Move the boat forward at collection speed"""
        self.apply_torque(self.fan_right, self.forward_speed)
        self.apply_torque(self.fan_left, self.forward_speed)
        rospy.loginfo("Boat moving forward at collection speed.")

    def rotate_left(self, torque):
        """Rotate the boat left with specified torque"""
        self.apply_torque(self.fan_right,torque )
        self.apply_torque(self.fan_left, 0.0)
        rospy.loginfo(f"Boat rotating left with torque {torque}")
        rospy.sleep(0.5)
        self.stop_fans()

    def rotate_right(self, torque):
        """Rotate the boat right with specified torque"""
        self.apply_torque(self.fan_right, 0.0)
        self.apply_torque(self.fan_left, torque)
        rospy.loginfo(f"Boat rotating right with torque {torque}")
        rospy.sleep(0.5)
        self.stop_fans()

    def control_boat_movement(self, section_counts):
        """Control boat movement based on waste detection sections"""
        if not self.is_collecting:
            self.stop_fans()
            return

        # Check each section and control movement accordingly
        if section_counts["Middle"] > 0 or section_counts["Bottom middle"] > 0:
            self.move_forward()
        elif section_counts["Left"] > 0:
            self.rotate_left(self.strong_rotation)
        elif section_counts["Right"] > 0:
            self.rotate_right(self.strong_rotation)
        elif section_counts["Upper left"] > 0:
            self.rotate_left(self.gentle_rotation)
        elif section_counts["Upper right"] > 0:
            self.rotate_right(self.gentle_rotation)
        elif section_counts["Bottom middle"] > 0:
            self.move_forward()
        else:
            self.stop_fans()

    def collection_control_callback(self, msg):
        """Callback function for collection control messages"""
        command = msg.data
        rospy.loginfo(f"[ Controller ]==========>: {command}")
        
        if command == 'start_collection':
            self.start_collection()
        elif command == 'end_collection':
            self.pause_collection()

    def start_collection(self):
        """Start the collection process"""
        self.is_collecting = True
        self.last_waste_detection_time = time.time()
        self.collection_status_pub.publish('collecting')
        rospy.loginfo("Collection process started")

    def pause_collection(self):
        """Pause the collection process"""
        self.is_collecting = False
        self.last_waste_detection_time = None
        self.collection_status_pub.publish('pause')
        self.stop_fans()
        rospy.loginfo("Collection process paused")

    def check_waste_detection_timeout(self):
        """Check if waste detection has timed out"""
        if not self.is_collecting or self.last_waste_detection_time is None:
            return False
            
        current_time = time.time()
        if current_time - self.last_waste_detection_time > self.waste_detection_timeout:
            rospy.loginfo("No waste detected for 5 seconds, pausing collection")
            self.pause_collection()
            return True
        return False

    def waste_detection_callback(self, msg):
        """Callback function for waste detection messages"""
        if not self.is_collecting:
            return

        # Initialize section counts
        section_counts = {
            "Left": 0,
            "Right": 0,
            "Upper left": 0,
            "Upper right": 0,
            "Middle": 0,
            "Bottom middle": 0
        }

        # Update counts from message
        for section in msg.sections:
            section_counts[section.section] = section.count

        # Check if any waste is detected
        total_waste = sum(section_counts.values())
        if total_waste > 0:
            self.last_waste_detection_time = time.time()
            self.collection_status_pub.publish('collecting')
            # Control boat movement based on waste position
            self.control_boat_movement(section_counts)
        else:
            # Check for timeout only if no waste is detected
            self.check_waste_detection_timeout()

        '''# Print section counts
        rospy.loginfo("\n=== Waste Detection Counts ===")
        for section, count in section_counts.items():
            rospy.loginfo(f"{section}: {count}")
        rospy.loginfo("============================\n")'''

        return section_counts

    def run(self):
        """Main loop"""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.is_collecting:
                self.check_waste_detection_timeout()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = WasteCollectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
