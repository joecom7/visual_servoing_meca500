import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO


class PersonDetector(Node):

    def __init__(self):
        super().__init__("person_detector")

        # Lightweight YOLO model for CPU
        self.model = YOLO("yolov8n.pt")  # nano model

        # ROS subscriptions and publishers
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.listener_callback, 10
        )
        self.publisher = self.create_publisher(PoseArray, "/target_poses", 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        results = self.model(img, verbose=False)
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_frame"

        for r in results:
            for box in r.boxes:
                if int(box.cls) != 0:  # Only person class
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().detach().numpy()
                cx = float((x1 + x2) / 2)
                cy = float((y1 + y2) / 2)

                pose = Pose()
                pose.position.x = cx
                pose.position.y = cy
                pose.position.z = 0.0
                pose_array.poses.append(pose)

        if pose_array.poses:
            self.publisher.publish(pose_array)
            # self.get_logger().info(f"Published {len(pose_array.poses)} person centers")

        # Optional visualization
        annotated_frame = results[0].plot()
        cv2.imshow("Person Detection", annotated_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
