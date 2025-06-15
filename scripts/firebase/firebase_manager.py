#!/usr/bin/env python3
from dotenv import load_dotenv
import os
import firebase_admin
from firebase_admin import credentials, db
import json
import rospy

class FirebaseManager:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('firebase_manager', anonymous=True)
        rospy.loginfo("Starting Firebase Manager node")

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
        try:
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred, {
                'databaseURL': database_url
            })
            rospy.loginfo("Firebase initialized successfully")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Firebase: {e}")
            raise

        # Load the initial database structure from JSON file
        json_path = os.path.join(os.path.dirname(__file__), 'initial_structure.json')
        try:
            with open(json_path, 'r') as f:
                initial_data = json.load(f)
            rospy.loginfo(f"Loaded initial structure from {json_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load initial_structure.json: {e}")
            raise

        # Get a reference to the root of the database
        ref = db.reference('/')

        # Check if the database is empty
        try:
            data = ref.get()
            if data is None:
                rospy.loginfo("Database is empty, setting initial structure")
                ref.set(initial_data)
                rospy.loginfo("Initial database structure set successfully")
            else:
                rospy.loginfo("Database already contains data, skipping structure setup")
        except Exception as e:
            rospy.logerr(f"Failed to access or set database: {e}")
            raise

    def run(self):
        # Keep the node running
        rospy.loginfo("Firebase Manager node is running and waiting for shutdown")
        rospy.spin()

    def shutdown(self):
        # Log on shutdown
        rospy.loginfo("Shutting down Firebase Manager node")

if __name__ == '__main__':
    try:
        manager = FirebaseManager()
        # Register shutdown hook
        rospy.on_shutdown(manager.shutdown)
        manager.run()
    except (rospy.ROSInterruptException, Exception) as e:
        rospy.logerr(f"Firebase Manager node failed: {e}")