import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import time


class RandomWalkPublisher(Node):
    def __init__(self):
        super().__init__("random_walk_publisher")

        # Parameters
        self.declare_parameter("refresh_rate", 10.0)  # Hz
        self.declare_parameter("num_targets", 5)
        self.declare_parameter("image_width_pixels", 1280)
        self.declare_parameter("image_height_pixels", 720)
        self.declare_parameter("max_depth", 5000.0)
        self.declare_parameter("step_size", 50.0)  # max pixel/depth change per frame

        self.refresh_rate = float(self.get_parameter("refresh_rate").value)
        self.num_targets = int(self.get_parameter("num_targets").value)
        self.image_width = int(self.get_parameter("image_width_pixels").value)
        self.image_height = int(self.get_parameter("image_height_pixels").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.step_size = float(self.get_parameter("step_size").value)

        # Initialize random target positions
        self.positions = np.zeros((self.num_targets, 3), dtype=np.float32)
        self._initialize_targets()

        # ROS publisher
        self.publisher = self.create_publisher(PoseArray, "/target_poses", 10)

        # Timer
        timer_period = 1.0 / self.refresh_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"RandomWalkPublisher started at {self.refresh_rate:.1f} Hz with "
            f"{self.num_targets} targets (step_size={self.step_size})."
        )

    def _initialize_targets(self):
        """Initialize positions randomly within the image and depth bounds."""
        self.positions[:, 0] = np.random.uniform(0, self.image_width, self.num_targets)
        self.positions[:, 1] = np.random.uniform(0, self.image_height, self.num_targets)
        self.positions[:, 2] = np.random.uniform(0.1, self.max_depth, self.num_targets)

    def timer_callback(self):
        """Update positions with random displacements and publish PoseArray."""
        # Apply random movement (random walk)
        deltas = np.random.uniform(-self.step_size, self.step_size, (self.num_targets, 3))
        self.positions += deltas

        # Clamp to valid range
        self.positions[:, 0] = np.clip(self.positions[:, 0], 0, self.image_width)
        self.positions[:, 1] = np.clip(self.positions[:, 1], 0, self.image_height)
        self.positions[:, 2] = np.clip(self.positions[:, 2], 0.1, self.max_depth)

        # Build PoseArray
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_frame"

        for (x, y, z) in self.positions:
            pose = Pose()
            pose.position.x = float(x - self.image_width / 2)
            pose.position.y = float(y - self.image_height / 2)
            pose.position.z = float(z)
            pose_array.poses.append(pose)

        self.publisher.publish(pose_array)
        self.get_logger().debug(f"Published {self.num_targets} random poses at {time.time():.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
