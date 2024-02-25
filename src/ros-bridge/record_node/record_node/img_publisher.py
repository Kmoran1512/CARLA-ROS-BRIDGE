#!/usr/bin/env python3
import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image


class ImageView:
    def __init__(self, node):
        self._init_pubsub(node)

        self.image = Image()
        self.bridge = CvBridge()
        self.manctrl_status = "enabled"

        self.gaze_x, self.gaze_y = (0, 0)

    def _init_pubsub(self, node):
        self.img_pub = node.create_publisher(Image, "/driver_img_view", 10)

        node.create_subscription(
            Image, "/carla/ego_vehicle/rgb_front/image", self._on_view_img, 10
        )
        node.create_subscription(
            Bool,
            "/carla/ego_vehicle/vehicle_control_manual_override",
            self._placeholder,
            10,
        )
        node.create_subscription(
            Image,
            "/carla/ego_vehicle/semantic_segmentation_front/image",
            self._placeholder,
            10,
        )

    def update_gaze(self, gaze_coord):
        self.gaze_x, self.gaze_y = gaze_coord

    def _on_view_img(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")

        # Draw manctrl status
        # Draw gaze
        # Draw boxes
        # Draw route

        self.image = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        self.img_pub.publish(self.image)

    def _placeholder(self, data):
        pass
