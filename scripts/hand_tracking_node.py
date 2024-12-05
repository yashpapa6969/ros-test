#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import cv2
import mediapipe as mp
import numpy as np

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_tracking_node')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
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

        # Define joint names that match your URDF
        self.joint_names = [
            'thumb_joint', 'thumb_proximal_joint', 'thumb_distal_joint',
            'index_proximal_joint', 'index_middle_joint', 'index_distal_joint',
            'middle_proximal_joint', 'middle_middle_joint', 'middle_distal_joint',
            'ring_proximal_joint', 'ring_middle_joint', 'ring_distal_joint',
            'little_proximal_joint', 'little_middle_joint', 'little_distal_joint'
        ]
        
        self.timer = self.create_timer(0.033, self.timer_callback)

    def calculate_finger_angles(self, hand_landmarks):
        # Get all landmark positions
        points = []
        for landmark in hand_landmarks.landmark:
            points.append([landmark.x, landmark.y, landmark.z])
        points = np.array(points)
        
        # Calculate joint angles for each finger
        joint_angles = []
        
        # Thumb (3 joints)
        thumb_angles = self.calculate_finger_joint_angles(points, [1, 2, 3, 4])
        joint_angles.extend(thumb_angles)
        
        # Index finger (3 joints)
        index_angles = self.calculate_finger_joint_angles(points, [5, 6, 7, 8])
        joint_angles.extend(index_angles)
        
        # Middle finger (3 joints)
        middle_angles = self.calculate_finger_joint_angles(points, [9, 10, 11, 12])
        joint_angles.extend(middle_angles)
        
        # Ring finger (3 joints)
        ring_angles = self.calculate_finger_joint_angles(points, [13, 14, 15, 16])
        joint_angles.extend(ring_angles)
        
        # Little finger (3 joints)
        little_angles = self.calculate_finger_joint_angles(points, [17, 18, 19, 20])
        joint_angles.extend(little_angles)
        
        return joint_angles

    def calculate_finger_joint_angles(self, points, indices):
        angles = []
        for i in range(len(indices)-2):
            p1 = points[indices[i]]
            p2 = points[indices[i+1]]
            p3 = points[indices[i+2]]
            
            # Calculate vectors
            v1 = p2 - p1
            v2 = p3 - p2
            
            # Calculate angle between vectors
            angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
            angles.append(angle)
        
        return angles

    def timer_callback(self):
        success, image = self.cap.read()
        if not success:
            self.get_logger().warn('Failed to capture frame')
            return

        # Convert image to RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image)
        
        # Convert back to BGR for display
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Prepare joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]  # Get first hand
            
            # Calculate joint angles
            joint_angles = self.calculate_finger_angles(hand_landmarks)
            joint_state.position = joint_angles
            
            # Draw landmarks on image
            self.mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS
            )

        # Publish joint states
        self.joint_pub.publish(joint_state)

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