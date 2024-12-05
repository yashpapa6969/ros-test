#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import cv2
import mediapipe as mp
import numpy as np

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control')
        self.declare_parameter('side', 'right')
        self.side = self.get_parameter('side').value
        
        # Initialize MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
        # Publisher for gripper joint state
        self.gripper_pub = self.create_publisher(
            JointState,
            f'/{self.side}_gripper_joint',
            10)
            
        # Create timer for camera processing
        self.create_timer(0.03, self.process_frame)  # ~30fps

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
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw landmarks on frame
                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )
                
                # Calculate gripper position based on thumb and index finger distance
                thumb_tip = hand_landmarks.landmark[4]  # Thumb tip
                index_tip = hand_landmarks.landmark[8]  # Index finger tip
                
                # Calculate distance between thumb and index finger
                distance = np.sqrt(
                    (thumb_tip.x - index_tip.x)**2 +
                    (thumb_tip.y - index_tip.y)**2
                )
                
                # Normalize distance to gripper position (0-1)
                # You may need to adjust these thresholds
                gripper_pos = np.clip((distance - 0.1) / 0.2, 0.0, 1.0)
                
                # Publish gripper position
                self.publish_gripper_position(gripper_pos)
        
        # Display frame
        cv2.imshow(f'{self.side} Hand Tracking', frame)
        cv2.waitKey(1)

    def publish_gripper_position(self, position):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [f'{self.side}_gripper_joint']
        joint_state.position = [position]
        self.gripper_pub.publish(joint_state)

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