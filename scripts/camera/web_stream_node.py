#!/usr/bin/env python3
import rospy
from flask import Flask, Response
from flask_cors import CORS
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time
import signal
import sys
import socket

# pip install flask flask-socketio eventlet flask-cors

app = Flask(__name__) 
CORS(app)
bridge = CvBridge()
latest_frame = None
ros_initialized = False
shutdown_flag = False

def get_ip_address():
    """Get the local IP address of the machine"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def image_callback(msg):
    global latest_frame
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        latest_frame = jpeg.tobytes()
    except Exception as e:
        rospy.logerr(f"Image processing error: {e}")

def gen_frames():
    while True:
        if latest_frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        time.sleep(0.033)  # ~30 FPS

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                  mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return f"""
    <html>
      <head>
        <title>ROS Camera Stream</title>
        <style>
            body {{ font-family: Arial, sans-serif; text-align: center; }}
        </style>
      </head>
      <body>
        <h1>Live Camera Feed</h1>
        <p>Stream URL: http://{get_ip_address()}:8080/video_feed</p>
        <img src="/video_feed" width="1280" height="720">
      </body>
    </html>
    """

def ros_spin():
    global ros_initialized
    rospy.Subscriber("/camera/image_raw", Image, image_callback)
    ros_initialized = True
    while not shutdown_flag and not rospy.is_shutdown():
        rospy.sleep(0.1)

def signal_handler(sig, frame):
    global shutdown_flag
    print("\nShutting down server...")
    shutdown_flag = True
    time.sleep(1)  # Give time for cleanup
    sys.exit(0)

if __name__ == '__main__':
    # Initialize ROS in main thread
    rospy.init_node('web_stream_node', anonymous=True)
    
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start ROS spinner in background thread
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # Wait for ROS to initialize
    while not ros_initialized and not rospy.is_shutdown():
        time.sleep(0.1)
    
    # Get network information
    ip_address = get_ip_address()
    port = 8080
    
    # Print access information
    print("\n" + "="*50)
    print("Web streaming server is running!")
    print(f"Local access: http://localhost:{port}")
    print(f"Network access: http://{ip_address}:{port}")
    print("Press Ctrl+C to stop the server")
    print("="*50 + "\n")
    
    try:
        # Start Flask server in main thread
        app.run(host='0.0.0.0', port=port, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        signal_handler(None, None)
    finally:
        shutdown_flag = True
        ros_thread.join()
        print("Server successfully shutdown")
