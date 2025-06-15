#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from firebase_admin import db, credentials
import firebase_admin
import json
import time
import yaml  # Add this import/  pip install pyyaml 
from std_msgs.msg import String  # Import String message type
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest
from actionlib_msgs.msg import GoalStatus

class FirebaseNavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node')
        
        # Get parameters
        self.robot_id = rospy.get_param('~robot_id', 'default_robot')
        self.firebase_url = rospy.get_param('~firebase_url', 'https://oceancleaner-eb432-default-rtdb.firebaseio.com')
        self.firebase_cred_path = rospy.get_param('~firebase_cred', '/home/iyed71/asv_ws/credinales/auth.json')
        self.map_yaml_path = rospy.get_param('~map_yaml', '/home/iyed71/asv_ws/src/asv_wave_sim/asv_wave_sim_gazebo/maps/map.yaml')  # Add param for yaml path

        # Load map origin offsets from YAML
        self.origin_offset_x, self.origin_offset_y = self.load_origin_offsets(self.map_yaml_path)
        
        # Initialize Firebase
        self.init_firebase()
        
        # MoveBase client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Publisher for navigation status
        self.navigation_status_pub = rospy.Publisher('/navigation_status', String, queue_size=10)
        # Subscribe to navigation control topic
        rospy.Subscriber('/navigation_control', String, self.navigation_control_callback)
        self.navigation_control_state = 'pause'  # Default to pause
        # Gazebo wrench services for stopping fans
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        rospy.wait_for_service('/gazebo/clear_body_wrenches')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.clear_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
        
        # this 2 line addded if tehre is any command still going to the gazebo {didint test it }
        self.stop_fans()
        self.stop_fans()

        # Main loop
        self.navigation_loop()

    def load_origin_offsets(self, yaml_path):
        """Load origin offsets from a YAML map file"""
        try:
            with open(yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
            origin = map_data.get('origin')
            return abs(float(origin[0])), abs(float(origin[1]))
        except Exception as e:
            rospy.logwarn(f"Failed to load map YAML: {e}. Using default offsets.")
            return 75.0, 9.0  # Default values

    def init_firebase(self):
        """Initialize Firebase connection"""
        try:
            cred = credentials.Certificate(self.firebase_cred_path)
            firebase_admin.initialize_app(cred, {
                'databaseURL': self.firebase_url
            })
            rospy.loginfo("Firebase initialized successfully")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Firebase: {str(e)}")
            raise

    def get_navigation_points(self):
        """Retrieve navigation points from Firebase"""
        try:
            # Get all coverage_path_planning entries
            cpp_ref = db.reference('navigation/coverage_path_planning')
            cpp_data = cpp_ref.get()
            if not cpp_data or not isinstance(cpp_data, dict):
                rospy.logwarn("No coverage_path_planning entries found in Firebase")
                return None

            # Get the latest key (assuming keys are sortable by recency)
            latest_key = max(cpp_data.keys())
            rospy.loginfo(f"Using latest coverage_path_planning key: {latest_key}")
            ref_path = f'navigation/coverage_path_planning/{latest_key}/waypoints'
            ref = db.reference(ref_path)
            points_data = ref.get()

            if not points_data:
                rospy.logwarn("No navigation points found in Firebase")
                return None

            # Check if points_data is a list or dictionary
            points = []
            if isinstance(points_data, dict):
                for point_id, point_data in sorted(points_data.items(), key=lambda x: int(x[0])):
                    try:
                        x = float(point_data.get('x', 0))
                        y = float(point_data.get('y', 0))
                        points.append((x, y))
                    except (ValueError, AttributeError) as e:
                        rospy.logwarn(f"Invalid point data for {point_id}: {point_data}")
            elif isinstance(points_data, list):
                for idx, point_data in enumerate(points_data):
                    try:
                        x = float(point_data.get('x', 0))
                        y = float(point_data.get('y', 0))
                        points.append((x, y))
                    except (ValueError, AttributeError) as e:
                        rospy.logwarn(f"Invalid point data at index {idx}: {point_data}")
            else:
                rospy.logerr("Unexpected data format for navigation points")
                return None

            # Print the first 5 points
            rospy.loginfo(f"First 5 points loaded: {points[:5]}")
            return points if points else None

        except Exception as e:
            rospy.logerr(f"Error retrieving navigation points: {str(e)}")
            return None

    def send_goal(self, x, y):
        """Send navigation goal to move_base"""
        # Transform coordinates using loaded offsets
        x_transformed = x - self.origin_offset_x
        y_transformed = y - self.origin_offset_y
       
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set position (assuming fixed orientation for simplicity)
        goal.target_pose.pose.position = Point(x_transformed, y_transformed, 0)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)  # Facing forward
        
        self.move_base_client.send_goal(goal)
        rospy.loginfo(f"Sent goal to position ({x_transformed}, {y_transformed})")
        
        # Wait for result with timeout
        wait = self.move_base_client.wait_for_result(rospy.Duration(60))
        
        if not wait:
            rospy.logwarn("Timed out waiting for move_base result")
            self.move_base_client.cancel_goal()
            return False
        
        return self.move_base_client.get_result()

    def navigation_control_callback(self, msg):
        if msg.data.lower() == 'pause':
            self.navigation_control_state = 'pause'
            # Only cancel if goal is active
            state = self.move_base_client.get_state()
            if state not in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.LOST]:
                self.move_base_client.cancel_goal()
            self.stop_fans()  # Immediately stop the boat
            rospy.loginfo('Navigation paused: move_base goal cancelled and boat stopped.')
        elif msg.data.lower() == 'navigate':
            self.navigation_control_state = 'navigate'
        else:
            rospy.logwarn(f'Unknown navigation control command: {msg.data}')

    def stop_fans(self):
        """Stop the boat by clearing and setting both fan torques to 0 using Gazebo services."""
        try:
            # Clear and set torque to 0 for both fans
            self.clear_wrench("boatcleaningc::fandroit")
            self.clear_wrench("boatcleaningc::fangauche")
            req_right = ApplyBodyWrenchRequest()
            req_right.body_name = "boatcleaningc::fandroit"
            req_right.reference_frame = "world"
            req_right.wrench.torque.x = 0.0
            req_right.duration = rospy.Duration(-1)
            self.apply_wrench(req_right)
            req_left = ApplyBodyWrenchRequest()
            req_left.body_name = "boatcleaningc::fangauche"
            req_left.reference_frame = "world"
            req_left.wrench.torque.x = 0.0
            req_left.duration = rospy.Duration(-1)
            self.apply_wrench(req_left)
            rospy.loginfo("[Navigation] Boat stopped: fan torques set to 0.")
        except rospy.ServiceException as e:
            rospy.logerr(f"[Navigation] Failed to stop fans: {e}")

    def navigation_loop(self):
        """Main navigation loop"""
        while not rospy.is_shutdown():
            # Always check navigation control state before proceeding
            while self.navigation_control_state == 'pause' and not rospy.is_shutdown():
                self.navigation_status_pub.publish('paused')
                self.stop_fans()  # Stop the boat if paused
                rospy.loginfo('Navigation is paused.')
                rospy.sleep(1)
            self.navigation_status_pub.publish('navigating')
            points = self.get_navigation_points()
            if not points:
                rospy.loginfo("No points to navigate to. Waiting...")
                rospy.sleep(5)
                continue
            for idx, (x, y) in enumerate(points, 1):
                if rospy.is_shutdown():
                    return
                # Check for pause before each goal
                while self.navigation_control_state != 'navigate' and not rospy.is_shutdown():
                    self.navigation_status_pub.publish('paused')
                    self.stop_fans()  # Stop the boat if paused
                    rospy.loginfo('Navigation paused before sending goal.')
                    rospy.sleep(1)
                self.navigation_status_pub.publish('navigating')
                rospy.loginfo(f"Navigating to point {idx}/{len(points)}: ({x}, {y})")
                result = self.send_goal(x, y)
                if result:
                    rospy.loginfo(f"Successfully reached point {idx}")
                else:
                    rospy.logwarn(f"Failed to reach point {idx}")
                rospy.loginfo(f"Current point being processed: ({x}, {y})")
                rospy.sleep(1)

            rospy.loginfo("Completed all navigation points. Waiting for new points...")
            rospy.sleep(10)  # Wait before checking for new points again

if __name__ == '__main__':
    try:
        node = FirebaseNavigationNode()
    except rospy.ROSInterruptException:
        pass
