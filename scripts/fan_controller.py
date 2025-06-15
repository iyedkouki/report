#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from gazebo_msgs.srv import BodyRequest
from pynput import keyboard  # Import pynput for keyboard control

class FanController:
    def __init__(self):
        rospy.init_node('fan_controller', anonymous=True)

        # Wait for Gazebo services
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        rospy.wait_for_service('/gazebo/clear_body_wrenches')
        
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.clear_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)

        # Maximum torque limit
        self.max_torque = -0.5
        self.current_droit = 0.0
        self.current_gauche = 0.0

        rospy.loginfo("Fan controller initialized. Use Z/S/Q/D to move the boat.")
        rospy.loginfo("Press 'Esc' to exit.")

        # Start keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def apply_torque(self, link_name, torque):
        """Apply torque to a specific fan, ensuring it stays within limits"""
        try:
            # Clamp torque to max allowed values
            torque = max(min(torque, self.max_torque), -self.max_torque)

            # Clear any previous wrench
            self.clear_wrench(link_name)

            # Apply new torque if non-zero
            if abs(torque) > 0.001:
                req = ApplyBodyWrenchRequest()
                req.body_name = link_name
                req.reference_frame = "world"
                req.wrench.torque.x = torque
                req.duration = rospy.Duration(-1)  # Continuous application
                self.apply_wrench(req)
                rospy.loginfo(f"Applied torque {torque:.2f} Nm to {link_name}")
            else:
                rospy.loginfo(f"Stopped torque on {link_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to apply torque on {link_name}: {e}")

    def on_press(self, key):
        """Handle key press events"""
        try:
            if key.char == 's':  # Move forward
                self.apply_torque("boatcleaningc::fandroit", 0.30)
                self.apply_torque("boatcleaningc::fangauche", 0.30)
            elif key.char == 'z':  # Move backward
                self.apply_torque("boatcleaningc::fandroit", -2.30)
                self.apply_torque("boatcleaningc::fangauche", -2.30)
            elif key.char == 'd':  # Turn left (20% left, 80% right)
                self.apply_torque("boatcleaningc::fandroit", -01.8)
                self.apply_torque("boatcleaningc::fangauche", 0.5)
            elif key.char == 'q':  # Turn right (80% left, 20% right)
                self.apply_torque("boatcleaningc::fandroit", 0.5)
                self.apply_torque("boatcleaningc::fangauche", -01.8)
        except AttributeError:
            pass  # Ignore non-character keys

    def on_release(self, key):
        """Stop movement when keys are released"""
        self.apply_torque("boatcleaningc::fandroit", 0.0)
        self.apply_torque("boatcleaningc::fangauche", 0.0)

        # Exit program when 'Esc' is pressed
        if key == keyboard.Key.esc:
            rospy.signal_shutdown("User exited")
            return False

    def control_loop(self):
        """Keep the ROS node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = FanController()
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass

