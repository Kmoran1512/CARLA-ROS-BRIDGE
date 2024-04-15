#!/usr/bin/env python3
import cv2
import rclpy

from .gaze_reader import GazeReader

from carla_msgs.msg import CarlaBoundingBox, CarlaBoundingBoxArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from typing import List
from rclpy.node import Node


class ImageView(Node):
    LABELS = {13: "Child", 30: "Police", 38: "Terrorist", 101: "Bike"}

    def __init__(self):
        super().__init__("image_view")

        self._init_params()
        self.gaze_reader = GazeReader() if self.show_gaze else None

        self._init_pubsub()

        self.bridge = CvBridge()
        self.manctrl_status = "enabled"

        self.gaze_x, self.gaze_y = (0, 0)
        self.boxes: List[CarlaBoundingBox] = []

    def _init_params(self):
        self.declare_parameter("draw_manctrl", "True")
        self.declare_parameter("draw_gaze", "False")
        self.declare_parameter("draw_outline", "False")
        self.declare_parameter("draw_route", "False")
        self.declare_parameter("labels", [])

        self.show_man_ctrl = bool(self.get_parameter("draw_manctrl").value)
        self.show_gaze = bool(self.get_parameter("draw_gaze").value)
        self.show_outline = bool(self.get_parameter("draw_outline").value)
        self.show_route = bool(self.get_parameter("draw_route").value)

    def _init_pubsub(self):
        self.img_pub = self.create_publisher(Image, "/driver_img_view", 10)

        self.create_subscription(
            Image, "/carla/ego_vehicle/rgb_front/image", self.on_view_img, 10
        )
        self.create_subscription(
            Bool,
            "/carla/ego_vehicle/vehicle_control_manual_override",
            self._set_manctrl_status,
            10,
        )
        self.create_subscription(
            CarlaBoundingBoxArray, "/carla/bounding_boxes", self._update_outlines, 10
        )

    def on_view_img(self, data):
        self._update_gaze()
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")

        if self.show_man_ctrl:
            self._draw_manctrl_status(cv_image)
        if self.show_gaze:
            self._draw_gaze(cv_image)
        if self.show_outline:
            self._draw_outlines(cv_image)

        self._draw_labels(cv_image)

        display_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        self.img_pub.publish(display_image)

    def _set_manctrl_status(self, data):
        self.manctrl_status = data.data

    def _draw_gaze(self, image):
        point_color = (0, 0, 255)

        x = int(self.gaze_x * image.shape[1])
        y = int(self.gaze_y * image.shape[0])

        cv2.drawMarker(
            image, (x, y), point_color, thickness=7, markerType=cv2.MARKER_CROSS
        )

    def _draw_labels(self, img):
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (0, 0, 255)

        for bbox in self.boxes:
            if bbox.size.x <= 5:
                continue

            label = self.LABELS[bbox.type]

            text_size = cv2.getTextSize(label, font, 1, 1)[0]

            cx = int(bbox.center.x - text_size[0] // 2)
            cy = int(bbox.center.y - bbox.size.y // 2)

            cv2.rectangle(
                img,
                (cx, cy - text_size[1]),
                (cx + text_size[0], cy + text_size[1] // 2),
                (255, 255, 0),
                -1,
            )
            cv2.putText(img, label, (cx, cy), font, 1, color, 1)

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

    def _draw_outlines(self, img):
        for bbox in self.boxes:
            half_w = bbox.size.x // 2
            half_h = bbox.size.y // 2

            x0 = int(bbox.center.x - half_w)
            y0 = int(bbox.center.y - half_h)
            x1 = int(bbox.center.x + half_w)
            y1 = int(bbox.center.y + half_h)

            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 0, 255), 2)

    def _update_gaze(self):
        gaze_x, gaze_y = (
            self.gaze_reader.get_gaze() if self.gaze_reader is not None else (0.0, 0.0)
        )

        if gaze_x <= 0.0 or gaze_y <= 0.0:
            return

        self.gaze_x = gaze_x
        self.gaze_y = gaze_y

    def _update_outlines(self, data: CarlaBoundingBoxArray):
        self.boxes = []

        for bbox in data.boxes:
            self.boxes.append(bbox)


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
