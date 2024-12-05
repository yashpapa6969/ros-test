#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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
            max_num_hands=1,  # Only track one hand
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
        # Publisher for gripper commands
        self.gripper_pub = self.create_publisher(
            String,
            '/gripper/command',
            10)
            
        # Create timer for camera processing
        self.create_timer(0.03, self.process_frame)  # ~30fps
        
        # State variables
        self.last_command = None
        self.command_threshold = 0.15  # Threshold for grip/release detection

    def process_frame(self):
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warn('Failed to read camera frame')
            return

        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process hand landmarks
        results = self.hands.process(frame_rgb)
        
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]  # Get first hand
            
            # Draw landmarks on frame
            self.mp_draw.draw_landmarks(
                frame,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS
            )
            
            # Calculate distance between thumb and index finger
            thumb_tip = hand_landmarks.landmark[4]  # Thumb tip
            index_tip = hand_landmarks.landmark[8]  # Index finger tip
            
            distance = np.sqrt(
                (thumb_tip.x - index_tip.x)**2 +
                (thumb_tip.y - index_tip.y)**2
            )
            
            # Determine gripper command based on gesture
            command = None
            if distance < self.command_threshold:
                command = 'close'  # Close fingers together
            else:
                command = 'open'   # Fingers apart
                
            # Only publish if command changes
            if command != self.last_command:
                self.publish_gripper_command(command)
                self.last_command = command
                self.get_logger().info(f'Sending gripper command: {command}')
        
        # Display frame
        cv2.imshow('Hand Tracking', frame)
        cv2.waitKey(1)

    def publish_gripper_command(self, command):
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

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