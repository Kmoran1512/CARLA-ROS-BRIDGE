import carla
import cv2
import numpy as np
import rclpy

from carla_msgs.msg import CarlaBoundingBox, CarlaBoundingBoxArray
from cv_bridge import CvBridge
from derived_object_msgs.msg import Object, ObjectArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class SemanticBoxes(Node):
    err = 0

    def __init__(self):
        super().__init__("semantic_boxes")
        self.default_camera_info()

        self.bridge = CvBridge()
        self._init_carla()
        self._init_pubsub()

    def _init_carla(self):
        client = carla.Client("localhost", 2000)
        client.set_timeout(5.0)
        self.world = client.get_world()

    def _init_pubsub(self):
        self.box_pub = self.create_publisher(
            CarlaBoundingBoxArray, "/carla/bounding_boxes", 10
        )

        self.create_subscription(
            ObjectArray, "/transformed_pedestrians", self._tf_callback, 10
        )

        self.create_subscription(
            CameraInfo,
            f"/carla/ego_vehicle/rgb_front/camera_info",
            self._set_matrices,
            1,
        )

    def _set_matrices(self, data: CameraInfo):
        self.D = np.array(data.d)
        self.K = np.reshape(data.k, (3, 3))
        self.R = np.reshape(data.r, (3, 3))
        self.T = np.zeros((1, 3))

        self.height = data.height
        self.width = data.width

    def _tf_callback(self, data: ObjectArray):
        arr_msg = CarlaBoundingBoxArray(height=self.height, width=self.width)
        for obj in data.objects:
            bbox = self.compute_bounding_box(obj)
            arr_msg.boxes.append(bbox)

        self.box_pub.publish(arr_msg)

    def compute_bounding_box(self, obj: Object):
        type_id = int(self.world.get_actor(int(obj.id)).type_id[-2:])

        bbox = CarlaBoundingBox(id=obj.id, type=type_id)
        x, y, z = obj.pose.position.x, obj.pose.position.y, obj.pose.position.z

        bbox.center.x, bbox.center.y = self.project(x, y, z)
        dim = obj.shape.dimensions
        corner_x, corner_y = self.project(x + dim[0], y + dim[2], z + dim[1])

        bbox.size.x = corner_x - bbox.center.x
        bbox.size.y = corner_y - bbox.center.y

        return bbox

    def project(self, x, y, z):
        points = np.array([x, y, z])

        projection, _ = cv2.projectPoints(points, self.R, self.T, self.K, self.D)
        return projection[0][0]

    def default_camera_info(self):
        self.width = 1920
        self.height = 1080

        fov = 110
        fl = self.width / (2.0 * np.tan(fov * np.pi / 360.0))

        self.D = np.zeros(5)
        self.K = np.array(
            [[fl, 0, self.width / 2], [0, fl, self.height / 2], [0, 0, 1]]
        )
        self.R = np.zeros(3)
        self.T = np.zeros((1, 3))


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
