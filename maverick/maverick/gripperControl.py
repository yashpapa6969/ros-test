#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import cv2
import mediapipe as mp
import numpy as np
import threading

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control')
        
        # Initialize publishers for both grippers
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, 
            '/left_open_manipulator_controller/commands',
            10
        )
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, 
            '/right_open_manipulator_controller/commands',
            10
        )

        # Initialize MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )

        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")
            return

        # Create timer for processing frames
        self.timer = self.create_timer(0.03, self.process_frame)  # ~30fps

        # Gripper limits from URDF
        self.gripper_min = -0.01
        self.gripper_max = 0.019

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read camera frame!")
            return

        # Flip frame horizontally for more intuitive control
        frame = cv2.flip(frame, 1)
        
        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # Draw hand landmarks
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                # Get thumb and index finger positions
                thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
                index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

                # Calculate distance between thumb and index
                distance = np.sqrt(
                    (thumb_tip.x - index_tip.x)**2 + 
                    (thumb_tip.y - index_tip.y)**2
                )

                # Map distance to gripper position
                gripper_pos = np.interp(
                    distance,
                    [0.02, 0.2],  # Input range (distance)
                    [self.gripper_min, self.gripper_max]  # Output range (gripper position)
                )

                # Determine if left or right hand
                if results.multi_handedness[idx].classification[0].label == "Left":
                    self.publish_gripper_command(gripper_pos, is_left=True)
                else:
                    self.publish_gripper_command(gripper_pos, is_left=False)

        # Display frame
        cv2.imshow('Gripper Control', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cleanup()

    def publish_gripper_command(self, position, is_left=True):
        msg = Float64MultiArray()
        msg.data = [position]
        
        if is_left:
            self.left_gripper_pub.publish(msg)
        else:
            self.right_gripper_pub.publish(msg)

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()