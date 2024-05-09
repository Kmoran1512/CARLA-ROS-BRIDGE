#!/usr/bin/env python3
import random
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
    LABELS = {
        1: "Soldier",
        30: "Police",
        12: "Orphan",
        15: "Terrorist",
        16: "Rapist",
        26: "Pedophile",
        20: "Judge",
        23: "Billionaire",
        25: "Celebrity",
    }

    def __init__(self, node: Node, show_gaze):
        self.node = node
        self._init_params()
        self.show_gaze = show_gaze
        self.gaze_reader: GazeReader = node.gaze_reader if self.show_gaze else None

        self._init_pubsub()

        self.bridge = CvBridge()
        self.manctrl_status = "enabled"
        self.labels_visible = False
        self.gaze_x, self.gaze_y = (0, 0)
        self.boxes: List[CarlaBoundingBox] = []
        self.offset_i = random.choice([0, 1])

    def _init_params(self):

        self.show_outline = False

    def _init_pubsub(self):
        self.img_pub = self.node.create_publisher(Image, "/driver_img_view", 10)

        self.node.create_subscription(
            Image, "/carla/ego_vehicle/rgb_front/image", self.on_view_img, 10
        )
        self.node.create_subscription(
            Bool, "/pedestrians_moving", self._update_ped_status, 1
        )

    def on_view_img(self, data):
        self._update_gaze()
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")

        self._draw_manctrl_status(cv_image)

        if self.show_gaze:
            self._draw_gaze(cv_image)
        if self.show_outline and self.labels_visible:
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

        has_child = any([bbox.type == 12 for bbox in self.boxes])

        for i, bbox in enumerate(self.boxes):
            if not self.labels_visible:
                continue

            label = self.LABELS[bbox.type]

            if bbox.type == 12:
                self.has_child = True

            text_size = cv2.getTextSize(label, font, 1, 1)[0]

            offset = 2 * text_size[1] if not has_child and i == self.offset_i else 0

            cx = int(bbox.center.x - text_size[0] // 2)
            cy = int(bbox.center.y - bbox.size.y // 2) - offset

            cv2.rectangle(
                img,
                (cx, cy - text_size[1] - 10),
                (cx + text_size[0], cy + text_size[1] // 2 - 10),
                (255, 255, 0),
                -1,
            )
            cv2.putText(img, label, (cx, cy - 10), font, 1, color, 1)

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

    def _update_ped_status(self, data: Bool):
        self.labels_visible = data.data

    def _update_outlines(self, data: CarlaBoundingBoxArray):
        self.boxes = []

        for bbox in data.boxes:
            self.boxes.append(bbox)