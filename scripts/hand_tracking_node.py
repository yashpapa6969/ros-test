#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import cv2
import mediapipe as mp
import numpy as np

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_tracking_node')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/hand_markers', 10)
        
        # Parameters
        self.declare_parameter('camera_index', 0)
        self.camera_index = self.get_parameter('camera_index').value
        
        # Initialize video capture
        self.cap = cv2.VideoCapture(self.camera_index)
        
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        # Create timer
        self.timer = self.create_timer(0.033, self.timer_callback)  # 30fps

    def create_hand_marker(self, landmarks, marker_id):
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        # Convert landmarks to points
        for landmark in landmarks.landmark:
            point = Point()
            point.x = landmark.x
            point.y = landmark.y
            point.z = landmark.z
            marker.points.append(point)
            
        return marker

    def timer_callback(self):
        success, image = self.cap.read()
        if not success:
            self.get_logger().warn('Failed to capture frame')
            return

        # Convert image to RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image)

        # Convert back to BGR for OpenCV
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.multi_hand_landmarks:
            marker_array = MarkerArray()
            
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # Draw landmarks on image
                self.mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )
                
                # Create and publish markers
                marker = self.create_hand_marker(hand_landmarks, idx)
                marker_array.markers.append(marker)
            
            self.marker_pub.publish(marker_array)

        # Display image
        cv2.imshow('Hand Tracking', image)
        cv2.waitKey(1)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()