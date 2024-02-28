#!/usr/bin/env python3
import cv2
import rclpy

from .gaze_reader import GazeReader

from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from sensor_msgs.msg import Image


class ImageView(Node):
    def __init__(self):
        super().__init__("image_view")

        self._init_parmas()
        self.gaze_reader = GazeReader() if self.show_gaze else None

        self._init_pubsub()

        self.image = Image()
        self.bridge = CvBridge()
        self.manctrl_status = "enabled"

        self.gaze_x, self.gaze_y = (0, 0)

    def _init_parmas(self):
        self.declare_parameter("draw_manctrl", "True")
        self.declare_parameter("draw_gaze", "False")
        self.declare_parameter("draw_outline", "False")
        self.declare_parameter("draw_route", "False")

        self.show_man_ctrl = bool(self.get_parameter("draw_manctrl").value)
        self.show_gaze = bool(self.get_parameter("draw_gaze").value)
        self.show_outline = bool(self.get_parameter("draw_outline").value)
        self.show_route = bool(self.get_parameter("draw_route").value)

    def _init_pubsub(self):
        self.img_pub = self.create_publisher(Image, "/driver_img_view", 10)

        self.create_subscription(
            Image, "/carla/ego_vehicle/rgb_front/image", self._on_view_img, 10
        )
        self.create_subscription(
            Bool,
            "/carla/ego_vehicle/vehicle_control_manual_override",
            self._set_manctrl_status,
            10,
        )

    def _on_view_img(self, data):
        self._update_gaze()
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")

        # Draw manctrl status
        if self.show_man_ctrl:
            self._draw_manctrl_status(cv_image)
        # Draw gaze
        if self.show_gaze:
            self._draw_gaze(cv_image)
        # Draw boxes
        # Draw route

        self.image = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        self.img_pub.publish(self.image)

    def _set_manctrl_status(self, data):
        self.manctrl_status = data.data

    def _draw_gaze(self, image):
        point_color = (0, 0, 255)

        x = int(self.gaze_x * image.shape[1])
        y = int(self.gaze_y * image.shape[0])

        cv2.drawMarker(
            image, (x, y), point_color, thickness=7, markerType=cv2.MARKER_CROSS
        )

    def _draw_manctrl_status(self, image):
        text = "M" if self.manctrl_status else "A"
        color = (255, 0, 0) if self.manctrl_status else (0, 255, 0)

        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)

        image_height, image_width, _ = image.shape
        c = (image_width // 2, image_height - 150)

        cv2.circle(image, c, 50, color, -1)

        cv2.putText(
            image,
            text,
            (c[0] - tw + 2, c[1] + th),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (0, 0, 0),
            2,
            lineType=cv2.LINE_AA,
        )

    def _update_gaze(self):
        gaze_x, gaze_y = (
            self.gaze_reader.get_gaze() if self.gaze_reader is not None else (0.0, 0.0)
        )

        if gaze_x <= 0.0 or gaze_y <= 0.0:
            return

        self.gaze_x = gaze_x
        self.gaze_y = gaze_y


def main(args=None):
    rclpy.init(args=args)
    img_view = ImageView()

    try:
        rclpy.spin(img_view)
    except KeyboardInterrupt:
        pass

    img_view.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
