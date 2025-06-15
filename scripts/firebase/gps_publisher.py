#!/usr/bin/env python3
import rospy
import random
import firebase_admin
from firebase_admin import credentials, db
from dotenv import load_dotenv
import os
import time

class GPSPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('gps_publisher', anonymous=True)
        rospy.loginfo("Starting GPS Publisher node")

        # Load environment variables from .env file in the same directory
        env_path = os.path.join(os.path.dirname(__file__), '.env')
        load_dotenv(env_path)
        rospy.loginfo(f"Loaded .env file from {env_path}")

        # Get the path to the service account key and database URL
        cred_path = os.getenv('FIREBASE_CREDENTIALS_PATH')
        database_url = os.getenv('FIREBASE_DATABASE_URL')

        # Validate credentials
        if not cred_path or not database_url:
            rospy.logerr("Missing FIREBASE_CREDENTIALS_PATH or FIREBASE_DATABASE_URL in .env file")
            raise ValueError("Invalid Firebase configuration")
        rospy.loginfo(f"Using credentials: {cred_path}")
        rospy.loginfo(f"Using database URL: {database_url}")

        # Initialize Firebase with the credentials
        max_attempts = 5  # Retry up to 5 times for initialization
        attempt = 1
        while attempt <= max_attempts:
            try:
                if not firebase_admin._apps:
                    cred = credentials.Certificate(cred_path)
                    firebase_admin.initialize_app(cred, {
                        'databaseURL': database_url
                    })
                    rospy.loginfo("Firebase initialized successfully")
                else:
                    rospy.loginfo("Firebase app already initialized")
                break
            except Exception as e:
                rospy.logwarn(f"Firebase initialization attempt {attempt}/{max_attempts} failed: {e}")
                attempt += 1
                time.sleep(1)  # Wait 1 second before retrying
                if attempt > max_attempts:
                    rospy.logerr("Failed to initialize Firebase after retries")
                    raise ValueError("Firebase initialization failed")

        # Reference to the Firebase database path
        self.gps_ref = db.reference('/sensors/gps/values')
        rospy.loginfo("Connected to Firebase database at /sensors/gps/values")

        # Use a timer to update the database every second
        self.timer = rospy.Timer(rospy.Duration(1.0), self.update_gps)

    def update_gps(self, event):
        # Generate random GPS coordinates
        longitude = random.uniform(-180, 180)
        latitude = random.uniform(-90, 90)
        try:
            self.gps_ref.update({
                'long': longitude,
                'lati': latitude
            })
            rospy.loginfo(f"Updated GPS: long={longitude:.6f}, lati={latitude:.6f}")
        except Exception as e:
            rospy.logerr(f"Failed to update GPS data: {e}")

    def shutdown(self):
        # Cleanup on shutdown
        rospy.loginfo("Shutting down GPS Publisher node")
        self.timer.shutdown()

if __name__ == '__main__':
    try:
        publisher = GPSPublisher()
        # Register shutdown hook
        rospy.on_shutdown(publisher.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"GPS Publisher node failed: {e}")