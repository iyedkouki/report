#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from collections import defaultdict
from asv_wave_sim_gazebo.msg import WasteSection, WasteDetection

class WasteTrackerNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('waste_tracker_node', anonymous=True)
        
        # Initialize YOLOv11 model
        self.model = YOLO("/home/iyed71/asv_ws/src/asv_wave_sim/asv_wave_sim_gazebo/scripts/camera/light_best.onnx")  # Replace with your model path if using a custom model
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize track history
        self.track_history = defaultdict(lambda: [])
        
        # Define waste classes (modify based on model class names or IDs)
        self.waste_classes = ["waste"]  # Replace with actual waste class names or IDs
        # Example: self.waste_classes = ["bottle"] or self.waste_classes = [5]
        
        # Image resolution
        self.target_width = 640
        self.target_height = 480
        
        # Define section boundaries (will be updated after first frame)
        self.left_width = 0
        self.right_width = 0
        self.right_x_start = 0
        self.central_width = 0
        self.upper_height = 0
        self.bottom_middle_y_start = 0
        self.frame_width = 0
        self.frame_height = 0
        
        # Publishers
        self.waste_info_pub = rospy.Publisher('/waste_info', String, queue_size=10)
        self.waste_detected_pub = rospy.Publisher('/waste_detected', Bool, queue_size=10)
        self.waste_detection_pub = rospy.Publisher('/waste_detection', WasteDetection, queue_size=10)
        
        # Subscriber
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # Flag for printing class names
        self.class_names_printed = False
        
        rospy.loginfo("Waste Tracker Node initialized")

    def update_section_boundaries(self):
        # Define section boundaries based on frame size
        self.left_width = int(self.frame_width * 0.15)  # 15% for Left
        self.right_width = int(self.frame_width * 0.15)  # 15% for Right
        self.right_x_start = self.frame_width - self.right_width  # Start of Right
        
        # Central area width
        self.central_width = self.frame_width - self.left_width - self.right_width
        
        # Heights
        self.upper_height = int(self.frame_height * 0.8)  # 80% for upper sections
        self.bottom_middle_y_start = self.upper_height  # Start of Bottom Middle
        
        # Vertical divisions for upper central area (25% - 50% - 25%)
        self.upper_left_x_end = self.left_width + int(self.central_width * 0.25)
        self.upper_middle_x_end = self.left_width + int(self.central_width * 0.75)
        # upper_middle: de upper_right_x_end à right_x_start
        
        # Print boundaries for debugging
        rospy.loginfo(f"Frame size: {self.frame_width}x{self.frame_height}")
        rospy.loginfo(f"Left: 0-{self.left_width}, Right: {self.right_x_start}-{self.frame_width}")
        rospy.loginfo(f"Upper left: {self.left_width}-{self.upper_left_x_end}, Upper right: {self.upper_left_x_end}-{self.upper_middle_x_end}, Upper middle: {self.upper_middle_x_end}-{self.right_x_start}, Height: 0-{self.upper_height}")
        rospy.loginfo(f"Bottom middle: {self.left_width}-{self.right_x_start}, Height: {self.upper_height}-{self.frame_height}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize image to 640x480
            frame = cv2.resize(frame, (self.target_width, self.target_height))
            
            # Update frame dimensions
            self.frame_width = frame.shape[1]
            self.frame_height = frame.shape[0]
            
            # Update section boundaries (only once or if resolution changes)
            if self.left_width == 0:
                self.update_section_boundaries()
            
            # Run YOLOv11 tracking
            results = self.model.track(frame, persist=True, tracker="bytetrack.yaml",conf=0.25)
            
            # Print class names once
            if not self.class_names_printed:
                rospy.loginfo(f"Model class names: {results[0].names}")
                self.class_names_printed = True
            
            # Initialize section data with the 6 exact sections
            section_data = {
                "Left": {"count": 0, "track_ids": [], "coords": []},
                "Right": {"count": 0, "track_ids": [], "coords": []},
                "Upper left": {"count": 0, "track_ids": [], "coords": []},
                "Upper right": {"count": 0, "track_ids": [], "coords": []},
                "Middle": {"count": 0, "track_ids": [], "coords": []},
                "Bottom middle": {"count": 0, "track_ids": [], "coords": []}
            }
            
            # Flag for waste detection
            waste_detected = False
            
            # Process detections
            if results[0].boxes is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                track_ids = results[0].boxes.id.int().cpu().tolist() if results[0].boxes.id is not None else []
                confidences = results[0].boxes.conf.cpu().numpy()
                class_ids = results[0].boxes.cls.int().cpu().numpy()
                
                # Debug detected classes
                detected_classes = [results[0].names[cls_id] for cls_id in class_ids]
                rospy.loginfo(f"Detected classes: {detected_classes}")
                
                # Process each detected object
                for box, track_id, conf, cls_id in zip(boxes, track_ids, confidences, class_ids):
                    x1, y1, x2, y2 = map(int, box)
                    class_name = results[0].names[cls_id]
                    
                    # Check if object is waste
                    is_waste = class_name in self.waste_classes or cls_id in self.waste_classes
                    if is_waste:
                        waste_detected = True
                    
                    # Set color: Green for waste, Blue for non-waste
                    box_color = (0, 255, 0) if is_waste else (255, 0, 0)
                    
                    # Calculate center for section detection
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # Determine section (only for waste)
                    section = "Unknown"
                    if is_waste:
                        if center_x < self.left_width:
                            section = "Left"
                        elif center_x >= self.right_x_start:
                            section = "Right"
                        elif center_y > self.upper_height:
                            section = "Bottom middle"
                        else:
                            if center_x < self.upper_left_x_end:
                                section = "Upper left"
                            elif center_x < self.upper_middle_x_end:
                                section = "Middle"
                            else:
                                section = "Upper right"
                        
                        # Update section data
                        section_data[section]["count"] += 1
                        section_data[section]["track_ids"].append(track_id)
                        section_data[section]["coords"].append((x1, y1, x2, y2))
                        
                        # Print waste details
                        rospy.loginfo(f"Waste Detected! Track ID: {track_id}, Class: {class_name}, "
                                      f"Confidence: {conf:.2f}, Coordinates: ({x1}, {y1}, {x2}, {y2}), "
                                      f"Section: {section}")
                    
                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                    
                    # Draw label
                    label = f"ID: {track_id} | {class_name}" + (f" | {section}" if is_waste else "")
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                    
                    # Display coordinates
                    coord_text = f"({x1}, {y1}), ({x2}, {y2})"
                    cv2.putText(frame, coord_text, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                    
                    # Update track history and draw tracking line (only for waste)
                    if is_waste:
                        track = self.track_history[track_id]
                        track.append((float(center_x), float(center_y)))
                        if len(track) > 30:
                            track.pop(0)
                        
                        if len(track) > 1:
                            points = np.array(track, dtype=np.int32).reshape((-1, 1, 2))
                            cv2.polylines(frame, [points], isClosed=False, color=(230, 230, 230), thickness=2)
            
            # Publish section data as a string
            for section, data in section_data.items():
                info_str = (f"Section: {section}, Count: {data['count']}, "
                           f"Track IDs: {data['track_ids']}, "
                           f"Coordinates: {data['coords']}")
                msg = String()
                msg.data = info_str
                self.waste_info_pub.publish(msg)
            
            # Publish waste detection status
            waste_detected_msg = Bool()
            waste_detected_msg.data = waste_detected
            self.waste_detected_pub.publish(waste_detected_msg)
            
            # Publish structured waste detection message
            waste_detection_msg = WasteDetection()
            for section, data in section_data.items():
                ws = WasteSection()
                ws.section = section
                ws.count = data['count']
                ws.track_ids = data['track_ids']
                # ws.coords removed: WasteSection no longer has coords
                waste_detection_msg.sections.append(ws)
            self.waste_detection_pub.publish(waste_detection_msg)
            
            # Draw section boundaries
            cv2.rectangle(frame, (0, 0), (self.left_width, self.frame_height), (0, 0, 255), 1)
            cv2.putText(frame, "Left", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.rectangle(frame, (self.right_x_start, 0), (self.frame_width, self.frame_height), (0, 0, 255), 1)
            cv2.putText(frame, "Right", (self.right_x_start + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            # Upper left
            cv2.rectangle(frame, (self.left_width, 0), (self.upper_left_x_end, self.upper_height), (255, 0, 0), 1)
            cv2.putText(frame, "Upper left", (self.left_width + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            # Middle (plus large)
            cv2.rectangle(frame, (self.upper_left_x_end, 0), (self.upper_middle_x_end, self.upper_height), (255, 0, 0), 1)
            cv2.putText(frame, "Middle", (self.upper_left_x_end + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            # Upper right
            cv2.rectangle(frame, (self.upper_middle_x_end, 0), (self.right_x_start, self.upper_height), (255, 0, 0), 1)
            cv2.putText(frame, "Upper right", (self.upper_middle_x_end + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            # Bottom middle
            cv2.rectangle(frame, (self.left_width, self.upper_height), (self.right_x_start, self.frame_height), (255, 0, 0), 1)
            cv2.putText(frame, "Bottom middle", (self.left_width + 10, self.upper_height + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Display frame
            cv2.imshow("YOLOv11 Waste Tracking", frame)
            cv2.waitKey(1)
        
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def shutdown(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = WasteTrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        node.shutdown()







