#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from . import camera_parameters as cam


class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.bridge = CvBridge()
        
        # Initialize publishers
        self.hsv_pub = self.create_publisher(Int32MultiArray, 'hsv_values', 10)
        # self.center_pub = self.create_publisher(Point, 'object_center', 10)
        # self.object_width = self.create_publisher(Float64MultiArray, 'object_width', 10)
        self.pub_tracked_point = self.create_publisher(Float64MultiArray, 'tracked_point', 10)
        
        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image,
            '/bluerov2/camera/image',
            self.image_callback,
            10
        )
        
        # Initialize OpenCV windows
        cv2.namedWindow("Color Tuner")
        cv2.namedWindow("Tracking View")

        # Set your initial HSV values
        self.initial_lower = [0, 95, 135]    # H, S, V
        self.initial_upper = [16, 255, 255]   # H, S, V
        
        # Create HSV sliders
        cv2.createTrackbar("Lower H", "Color Tuner", self.initial_lower[0], 179, lambda x: None)
        cv2.createTrackbar("Lower S", "Color Tuner", self.initial_lower[1], 255, lambda x: None)
        cv2.createTrackbar("Lower V", "Color Tuner", self.initial_lower[2], 255, lambda x: None)
        cv2.createTrackbar("Upper H", "Color Tuner", self.initial_upper[0], 179, lambda x: None)
        cv2.createTrackbar("Upper S", "Color Tuner", self.initial_upper[1], 255, lambda x: None)
        cv2.createTrackbar("Upper V", "Color Tuner", self.initial_upper[2], 255, lambda x: None)
        
        self.get_logger().info("Object Tracker Node Initialized")
        self.current_frame = None

    def image_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def process_frame(self):
        if self.current_frame is None:
            return
            
        # Resize for display
        display_frame = cv2.resize(self.current_frame, (640, 480))
        hsv = cv2.cvtColor(display_frame, cv2.COLOR_BGR2HSV)
        
        # Get current HSV range from sliders
        lh = cv2.getTrackbarPos("Lower H", "Color Tuner")
        ls = cv2.getTrackbarPos("Lower S", "Color Tuner")
        lv = cv2.getTrackbarPos("Lower V", "Color Tuner")
        uh = cv2.getTrackbarPos("Upper H", "Color Tuner")
        us = cv2.getTrackbarPos("Upper S", "Color Tuner")
        uv = cv2.getTrackbarPos("Upper V", "Color Tuner")
        
        lower = np.array([lh, ls, lv])
        upper = np.array([uh, us, uv])
        
        # Publish HSV values
        hsv_msg = Int32MultiArray()
        hsv_msg.data = [lh, ls, lv, uh, us, uv]
        self.hsv_pub.publish(hsv_msg)

        
        # Create mask and find contours
        mask = cv2.inRange(hsv, lower, upper)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 100:  # Minimum area threshold
                M = cv2.moments(largest)
                tracking_data_msg = Float64MultiArray()
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    # Publish center coordinates
                    center_msg = Point()
                    center_msg.x = float(cX)
                    center_msg.y = float(cY)
                    center_msg.z = 0.0  # For 2D tracking
                    center_meter = cam.convertOnePoint2meter((cX, cY))
                    # center_meter_msg = Float64MultiArray(data = center_meter)
                    tracking_data_msg.data = [
                        center_meter[0],
                        center_meter[1],
                        0.0,  # Placeholder for width
                        0.0,   # Placeholder for width difference
                    ]
                    # self.center_pub.publish(center_msg)
                    # self.pub_tracked_point.publish(center_meter_msg)
                    
                    # Visual feedback
                    cv2.circle(display_frame, (cX, cY), 7, (0, 255, 0), -1)
                    # cv2.circle(display_frame, center_meter, 7, (0, 255, 0), -1) 
                    cv2.putText(display_frame, f"({center_meter[0]:.2f}, {center_meter[1]:.2f})", (cX-20, cY-20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(largest)
                
                # Publish center (existing)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Calculate diagonal (Pythagorean theorem)
                # diagonal = int(np.sqrt(w**2 + h**2))  # Convert to integer pixels
                
                # Publish diagonal only
                # width_msg = Float64MultiArray(data = w)
                tracking_data_msg.data[2] = cam.convertWidth2meter(w)
                desired_width = 0.5  # Desired width in meters
                tracking_data_msg.data[3] = tracking_data_msg.data[2] - desired_width
                self.pub_tracked_point.publish(tracking_data_msg)
                # self.object_width.publish(width_msg)
                
                # Draw bounding box (optional)
                cv2.rectangle(display_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        # Display results
        cv2.imshow("Tracking View", display_frame)
        cv2.imshow("Mask View", mask)
        key = cv2.waitKey(1)
        
        if key == ord('q'):
            self.get_logger().info("Shutting down...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
