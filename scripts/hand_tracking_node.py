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
        self.publisher_ = self.create_publisher(JointState, '/hand_tracking/joint_states', 10)
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

        # Define joint names that correspond to your URDF
        self.joint_names = [
            'index_mcp_joint', 'index_pip_joint', 'index_dip_joint',
            'middle_mcp_joint', 'middle_pip_joint', 'middle_dip_joint',
            'ring_mcp_joint', 'ring_pip_joint', 'ring_dip_joint',
            'pinky_mcp_joint', 'pinky_pip_joint', 'pinky_dip_joint',
            'thumb_cmc_joint', 'thumb_mcp_joint', 'thumb_ip_joint'
        ]
        
        # Create timer for processing
        self.timer = self.create_timer(0.033, self.timer_callback)  # 30fps

    def calculate_joint_angles(self, hand_landmarks):
        positions = []
        for landmark in hand_landmarks.landmark:
            positions.append([landmark.x, landmark.y, landmark.z])
        positions = np.array(positions)
        
        # Calculate joint angles (simplified example)
        joint_angles = []
        
        # Index finger joints (3 angles)
        for i in range(5, 8):
            angle = np.arctan2(positions[i+1][1] - positions[i][1],
                             positions[i+1][0] - positions[i][0])
            joint_angles.append(angle)
        
        # Middle finger joints (3 angles)
        for i in range(9, 12):
            angle = np.arctan2(positions[i+1][1] - positions[i][1],
                             positions[i+1][0] - positions[i][0])
            joint_angles.append(angle)
        
        # Ring finger joints (3 angles)
        for i in range(13, 16):
            angle = np.arctan2(positions[i+1][1] - positions[i][1],
                             positions[i+1][0] - positions[i][0])
            joint_angles.append(angle)
        
        # Pinky joints (3 angles)
        for i in range(17, 20):
            angle = np.arctan2(positions[i+1][1] - positions[i][1],
                             positions[i+1][0] - positions[i][0])
            joint_angles.append(angle)
        
        # Thumb joints (3 angles)
        for i in range(1, 4):
            angle = np.arctan2(positions[i+1][1] - positions[i][1],
                             positions[i+1][0] - positions[i][0])
            joint_angles.append(angle)
        
        return joint_angles

    def timer_callback(self):
        success, image = self.cap.read()
        if not success:
            self.get_logger().warn('Failed to capture frame')
            return

        # Convert the BGR image to RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image)

        # Prepare joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]  # Get first hand
            
            # Calculate joint angles from landmarks
            joint_angles = self.calculate_joint_angles(hand_landmarks)
            
            # Publish joint states
            joint_state.position = joint_angles
            
            # Draw hand landmarks for visualization
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            self.mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS
            )
            
            cv2.imshow('Hand Tracking', image)
            cv2.waitKey(1)

        self.publisher_.publish(joint_state)

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