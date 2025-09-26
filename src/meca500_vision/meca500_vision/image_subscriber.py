import numpy as np
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

        # Declare launch parameter
        self.declare_parameter("performance_mode", "high")
        mode = self.get_parameter("performance_mode").value

        # Map performance modes to YOLO image sizes
        mode_to_imgsz = {
            "low": 160,
            "medium": 320,
            "high": None,  # or None for full resolution
        }
        self.imgsz = mode_to_imgsz.get(mode, 640)

        self.declare_parameter("image_width_pixels", 1280)
        self.image_width_pixels = self.get_parameter("image_width_pixels").value

        self.declare_parameter("image_height_pixels", 720)
        self.image_height_pixels = self.get_parameter("image_height_pixels").value

        self.model = YOLO("yolov8n.pt")

        # ROS subscriptions and publishers
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.listener_callback, 10
        )
        self.create_subscription(Image, "/camera/depth_image", self.depth_callback, 10)

        self.publisher = self.create_publisher(PoseArray, "/target_poses", 10)
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = np.array(depth_img, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"Could not convert depth image: {e}")

    def listener_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        if self.latest_depth is None:
            self.get_logger().warn("No depth image received yet.")
            return

        if self.imgsz is not None:
            results = self.model(img, imgsz=self.imgsz, verbose=False)

        else:
            results = self.model(img, verbose=False)

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_frame"

        for r in results:
            for box in r.boxes:
                if int(box.cls) != 0:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().detach().numpy()
                cx = float((x1 + x2) / 2)
                cy = float((y1 + y2) / 2)
                if (
                    0 <= cy < self.latest_depth.shape[0]
                    and 0 <= cx < self.latest_depth.shape[1]
                ):
                    depth_value = float(self.latest_depth[int(cy), int(cx)])
                else:
                    depth_value = float("nan")

                pose = Pose()
                pose.position.x = cx - self.image_width_pixels/2
                pose.position.y = cy - self.image_height_pixels/2
                pose.position.z = depth_value
                pose_array.poses.append(pose)

        if pose_array.poses:
            self.publisher.publish(pose_array)

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
