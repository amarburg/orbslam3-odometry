import rclpy
from rclpy.node import Node
from pathlib import Path
import yaml
from sensor_msgs.msg import Image, CameraInfo


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")

        self.declare_parameter("camera_info_file", "")

        # Load camera info
        camera_info_path = Path(
            self.get_parameter("camera_info_file").get_parameter_value().string_value
        )

        if not camera_info_path.exists():
            self.get_logger().error(
                f'Camera info file "{camera_info_path}" doesn\'t exist!'
            )
            exit(-1)

        self.camera_info_msg = self.load_camera_info(camera_info_path)

        self.image_publisher = self.create_publisher(Image, "output/image_raw", 1)
        self.info_publisher = self.create_publisher(CameraInfo, "output/camera_info", 1)

        self.image_subscriber = self.create_subscription(
            Image, "input/image_raw", self.image_callback, 1
        )

    def load_camera_info(self, yaml_file_path):
        self.get_logger().info(f"Loading camera info from {yaml_file_path}")

        with open(yaml_file_path, "r") as file:
            calib_data = yaml.safe_load(file)

        camera_info = CameraInfo()
        try:
            camera_info.height = calib_data["image_height"]
            camera_info.width = calib_data["image_width"]
            camera_info.distortion_model = calib_data["distortion_model"]
            camera_info.d = calib_data["distortion_coefficients"]["data"]
            camera_info.k = calib_data["camera_matrix"]["data"]
            camera_info.r = calib_data["rectification_matrix"]["data"]
            camera_info.p = calib_data["projection_matrix"]["data"]
        except KeyError:
            self.get_logger().error("Error!")

        return camera_info

    def image_callback(self, img):
        self.camera_info_msg.header.stamp = img.header.stamp
        self.camera_info_msg.header.frame_id = img.header.frame_id
        self.info_publisher.publish(self.camera_info_msg)
        self.image_publisher.publish(img)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
