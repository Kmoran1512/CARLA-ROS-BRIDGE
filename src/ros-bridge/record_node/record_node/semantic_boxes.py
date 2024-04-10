import cv2
import json
import os
import numpy as np
import rclpy

from ament_index_python.packages import get_package_share_directory
from carla_msgs.msg import CarlaBoundingBox, CarlaBoundingBoxArray
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


semantic_dict_path = os.path.join(
    get_package_share_directory("record_node"), "config", "semantic_map.json"
)


class SemanticBoxes(Node):
    err = 0

    def __init__(self):
        super().__init__("semantic_boxes")

        self.bridge = CvBridge()
        self._init_pubsub()

        with open(semantic_dict_path, "r") as f:
            self.semantic_dict = json.load(f)

    def _init_pubsub(self):
        self.box_pub = self.create_publisher(
            CarlaBoundingBoxArray, "/carla/bounding_boxes", 10
        )

        self.create_subscription(
            Image,
            "/carla/ego_vehicle/semantic_segmentation_front/image",
            self.image_callback,
            10,
        )

    def image_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
        self._find_boxes(img)

    def _find_boxes(self, img: np.ndarray):
        ped_color = np.array(self.semantic_dict["Pedestrian"])
        mask = cv2.inRange(img, ped_color, ped_color)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return

        max_w = max([cv2.boundingRect(contour)[2] for contour in contours])
        max_h = max([cv2.boundingRect(contour)[3] for contour in contours])

        boxes = CarlaBoundingBoxArray()
        boxes.height = img.shape[0]
        boxes.width = img.shape[1]
        for contour in contours:
            single_box = CarlaBoundingBox()
            x, y, w, h = cv2.boundingRect(contour)

            if w + h < (max_w + max_h) // 2:
                continue

            single_box.center.x = float(x + w // 2)
            single_box.center.y = float(y + h // 2)

            single_box.size.x = float(w)
            single_box.size.y = float(h)

            boxes.boxes.append(single_box)

        self.box_pub.publish(boxes)


def main(args=None):
    rclpy.init(args=args)
    semantic_boxes = SemanticBoxes()

    try:
        rclpy.spin(semantic_boxes)
    except KeyboardInterrupt:
        pass

    semantic_boxes.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
