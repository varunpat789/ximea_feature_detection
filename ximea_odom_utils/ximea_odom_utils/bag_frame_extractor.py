import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

import cv2
import os


class BagFrameExtractor(Node):
    def __init__(self):
        super().__init__("bag_frame_extractor")

        self.image_topic = "/image_raw"
        self.frame_number = 250
        self.output_path = "./src/ximea_feature_detection/frame15.png"

        self.bridge = CvBridge()

        self.frame_count = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile
        )

        self.get_logger().info(
            f"Waiting to extract frame {self.frame_number} from {self.image_topic}:"
        )

    def image_callback(self, msg):
        self.frame_count += 1
        self.get_logger().info(
                    f"At frame {self.frame_count}"
                )

        if self.frame_count == self.frame_number:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                output_dir = os.path.dirname(self.output_path)
                if output_dir and not os.path.exists(output_dir):
                    os.makedirs(output_dir)

                cv2.imwrite(self.output_path, cv_image)
                self.get_logger().info(
                    f"Successfully saved frame {self.frame_number} to {self.output_path}"
                )

                raise SystemExit

            except CvBridgeError as e:
                self.get_logger().error(f"CV Bridge error: {str(e)}")
            except Exception as e:
                self.get_logger().error(f"Error saving image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    frame_extractor = BagFrameExtractor()

    try:
        rclpy.spin(frame_extractor)
    except Exception:
        frame_extractor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
