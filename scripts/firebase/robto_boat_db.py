#!/usr/bin/env python3
import firebase_admin
from firebase_admin import db

class RobotBoatDB:
    def __init__(self):
        # Ensure Firebase is initialized
        if not firebase_admin._apps:
            raise ValueError("Firebase app not initialized")
        self.ref = db.reference('/robot_boat')

    def get_status(self):
        """Get the current status of the robot boat."""
        return self.ref.child('status').get()

    def update_status(self, status):
        """Update the robot boat status."""
        self.ref.update({'status': status})

    def get_system(self):
        """Get the system state (on/off)."""
        return self.ref.child('system').get()

    def update_system(self, system):
        """Update the system state."""
        self.ref.update({'system': system})

    def get_problem(self):
        """Get the problem status."""
        return self.ref.child('problem').get()

    def update_problem(self, problem):
        """Update the problem status."""
        self.ref.update({'problem': problem})