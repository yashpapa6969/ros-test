import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PoseLandmark
from geometry_msgs.msg import Point

class PoseDetectionPublisher(Node):
    def __init__(self):
        super().__init__('pose_detection_publisher')
        self.publisher_ = self.create_publisher(PoseLandmark, 'pose_landmarks', 10)
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, model_complexity=2, smooth_landmarks=True,
                                      enable_segmentation=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.drawing_utils = mp.solutions.drawing_utils

    def run_pose_detection(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Cannot open webcam")
            return

        try:
            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    self.get_logger().warn("Ignoring empty camera frame.")
                    continue

                image = self.process_image(image)
                cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
                if cv2.waitKey(5) & 0xFF == 27:  # Exit loop if 'ESC' is pressed
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()

    def process_image(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)

        if results.pose_landmarks:
            self.draw_landmarks(image, results.pose_landmarks)
            self.plot_landmarks_and_publish(results.pose_world_landmarks)
        return image

    def draw_landmarks(self, image, landmarks):
        self.drawing_utils.draw_landmarks(
            image, landmarks, self.mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=self.drawing_utils.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
            connection_drawing_spec=self.drawing_utils.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2))

    def plot_landmarks_and_publish(self, landmarks):
        landmarks_labels = {
            # Left arm
            11: "left_shoulder", 
            13: "left_elbow",
            15: "left_wrist",
            # Right arm  
            12: "right_shoulder",
            14: "right_elbow", 
            16: "right_wrist",
            # Reference points for normalization
            23: "left_hip",
            24: "right_hip",
        }
        
        pose_landmark_msg = PoseLandmark()
        
        # Get reference points for normalization
        left_shoulder = landmarks.landmark[11]
        right_shoulder = landmarks.landmark[12]
        left_hip = landmarks.landmark[23] 
        right_hip = landmarks.landmark[24]
        
        # Calculate torso center as origin
        origin_x = (left_shoulder.x + right_shoulder.x + left_hip.x + right_hip.x) / 4
        origin_y = (left_shoulder.y + right_shoulder.y + left_hip.y + right_hip.y) / 4
        origin_z = (left_shoulder.z + right_shoulder.z + left_hip.z + right_hip.z) / 4
        
        # Calculate scale factor based on shoulder width
        shoulder_width = ((right_shoulder.x - left_shoulder.x)**2 + 
                         (right_shoulder.y - left_shoulder.y)**2 + 
                         (right_shoulder.z - left_shoulder.z)**2)**0.5
        
        # Process each landmark
        for idx, landmark in enumerate(landmarks.landmark):
            if idx in landmarks_labels:
                # Normalize coordinates relative to torso center and shoulder width
                norm_x = (landmark.x - origin_x) / shoulder_width
                norm_y = (landmark.y - origin_y) / shoulder_width  
                norm_z = (landmark.z - origin_z) / shoulder_width
                
                label = landmarks_labels[idx]
                pose_landmark_msg.label.append(label)
                pose_landmark_msg.point.append(Point(
                    x=norm_x,
                    y=norm_y, 
                    z=norm_z
                ))

        self.publisher_.publish(pose_landmark_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseDetectionPublisher()
    node.run_pose_detection()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
