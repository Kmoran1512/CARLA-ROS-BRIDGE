#!/usr/bin/env python3
import cv2
import rclpy

from cv_bridge import CvBridge
from std_msgs.msg import Bool
from sensor_msgs.msg import Image


class ImageView:
    def __init__(self, node, show_man_ctrl=True, show_gaze=False):
        self.show_man_ctrl = show_man_ctrl
        self.show_gaze = show_gaze

        self._init_pubsub(node)

        self.image = Image()
        self.bridge = CvBridge()
        self.manctrl_status = "enabled"

        self.gaze_x, self.gaze_y = (0, 0)

    def _init_pubsub(self, node):
        self.img_pub = node.create_publisher(Image, "/driver_img_view", 25)

        node.create_subscription(
            Image, "/carla/ego_vehicle/rgb_front/image", self._on_view_img, 25
        )
        node.create_subscription(
            Bool,
            "/carla/ego_vehicle/vehicle_control_manual_override",
            self._set_manctrl_status,
            10,
        )
        node.create_subscription(
            Image,
            "/carla/ego_vehicle/semantic_segmentation_front/image",
            self._placeholder,
            10,
        )

    def update_gaze(self, x, y):
        self.gaze_x = x
        self.gaze_y = y

    def _on_view_img(self, data):
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

    def _placeholder(self, _):
        pass

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
