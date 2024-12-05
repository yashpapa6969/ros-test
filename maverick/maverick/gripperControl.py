#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import cv2
import mediapipe as mp
import numpy as np

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control')
        
        # Initialize MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,  # Track both hands
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
        # Publishers for gripper commands
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/left_open_manipulator_controller/commands',
            10)
            
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/right_open_manipulator_controller/commands',
            10)
            
        # Create timer for camera processing
        self.create_timer(0.03, self.process_frame)  # ~30fps
        
        # State variables
        self.last_left_position = 0.0
        self.last_right_position = 0.0
        self.command_threshold = 0.15  # Threshold for grip/release detection
        
        # Gripper limits from URDF
        self.gripper_min = -0.01
        self.gripper_max = 0.019

    def process_frame(self):
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warn('Failed to read camera frame')
            return

        # Flip frame horizontally for more intuitive control
        frame = cv2.flip(frame, 1)
        
        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process hand landmarks
        results = self.hands.process(frame_rgb)
        
        if results.multi_hand_landmarks:
            for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # Draw landmarks
                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )
                
                # Calculate distance between thumb and index finger
                thumb_tip = hand_landmarks.landmark[4]
                index_tip = hand_landmarks.landmark[8]
                
                distance = np.sqrt(
                    (thumb_tip.x - index_tip.x)**2 +
                    (thumb_tip.y - index_tip.y)**2
                )
                
                # Normalize distance to gripper position
                gripper_position = np.interp(
                    distance,
                    [0.05, 0.3],  # Input range for hand distance
                    [self.gripper_max, self.gripper_min]  # Output range for gripper
                )
                
                # Determine if left or right hand
                if results.multi_handedness[idx].classification[0].label == "Left":
                    if abs(gripper_position - self.last_left_position) > 0.001:
                        self.publish_gripper_command(gripper_position, is_left=True)
                        self.last_left_position = gripper_position
                        self.get_logger().info(f'Left gripper position: {gripper_position}')
                else:  # Right hand
                    if abs(gripper_position - self.last_right_position) > 0.001:
                        self.publish_gripper_command(gripper_position, is_left=False)
                        self.last_right_position = gripper_position
                        self.get_logger().info(f'Right gripper position: {gripper_position}')
        
        # Display frame
        cv2.imshow('Hand Tracking', frame)
        cv2.waitKey(1)

    def publish_gripper_command(self, position, is_left=True):
        msg = Float64MultiArray()
        msg.data = [position]
        if is_left:
            self.left_gripper_pub.publish(msg)
        else:
            self.right_gripper_pub.publish(msg)

    def __del__(self):
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()