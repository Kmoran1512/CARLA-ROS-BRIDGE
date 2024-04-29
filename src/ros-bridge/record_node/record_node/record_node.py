#!/usr/bin/env python3
import carla
import csv
import os
import time, datetime
import math
import rclpy


from ament_index_python.packages import get_package_share_directory
from carla_msgs.msg import (
    CarlaBoundingBox,
    CarlaBoundingBoxArray,
    CarlaEgoVehicleStatus,
)
from derived_object_msgs.msg import ObjectArray, Object
from geometry_msgs.msg import PoseArray
from pygame.locals import K_e, K_r, K_z
from rclpy.node import Node
from ros_g29_force_feedback.msg import ForceControl, ForceFeedback
from sensor_msgs.msg import CameraInfo
from signal import SIGINT
from std_msgs.msg import Bool, Float32, Int8
from subprocess import Popen
from transforms3d.euler import quat2euler
from typing import List, Tuple

from .gaze_reader import GazeReader


class RecordingOrchestrator(Node):
    def __init__(self):
        super().__init__("recording_orchestrator")

        self.headers = {}
        self._init_params()
        self.gaze_reader = GazeReader() if self.record_gaze else None
        self.gaze_x = self.gaze_y = 0.0

        self.all_data = []
        self.next_row = [0.0] * len(self.headers)

        self._init_pub_sub()
        self._init_carla()

        self.start = None

    def _init_params(self):
        self.declare_parameter("record_gaze", "False")
        self.declare_parameter("participant_number", "1")
        self.declare_parameter("scenario_number", "1")

        self.record_gaze = bool(self.get_parameter("record_gaze").value)
        self.participant_number = int(self.get_parameter("participant_number").value)
        self.scenario_number = int(self.get_parameter("scenario_number").value)

        header_file = os.path.join(
            get_package_share_directory("record_node"), "config", "column_headers.txt"
        )

        with open(header_file, "r") as f:
            for i, line in enumerate(f.read().splitlines()):
                self.headers[line] = i

    def _init_pub_sub(self):
        self.create_subscription(Int8, "/key_press", self._on_key_press, 10)

        # Records velocity, steering, throttle, brake
        self.create_subscription(
            CarlaEgoVehicleStatus,
            "/carla/ego_vehicle/vehicle_status",
            self._record_vehicle_status,
            10,
        )

        # Records car and pedestrian xyz
        self.create_subscription(
            ObjectArray, "/carla/objects", self._record_object_status, qos_profile=10
        )

        self.create_subscription(
            ForceControl, "/force_control", self._record_torque, 10
        )
        self.create_subscription(
            ForceFeedback, "/ff_target", self._record_target_steer, 10
        )
        self.create_subscription(
            Bool,
            "/carla/ego_vehicle/vehicle_control_manual_override",
            self._record_auton_status,
            10,
        )
        self.create_subscription(
            PoseArray, "/carla/ego_vehicle/next_50", self._record_next_waypoint, 10
        )
        self.create_subscription(Float32, "/wheel_value", self._record_ts_number, 10)
        self.create_subscription(
            Float32, "/to_simulator_steer", self._record_sim_number, 10
        )
        self.create_subscription(
            CarlaBoundingBoxArray,
            "/carla/bounding_boxes",
            self._record_ped_2d_transform,
            10,
        )
        self.create_subscription(
            CameraInfo, f"/carla/ego_vehicle/rgb_front/camera_info", self._record_fov, 1
        )

    def _init_carla(self):
        client = carla.Client("localhost", 2000)
        client.set_timeout(5.0)
        self.world = client.get_world()

    def write(self):
        if self.start is None:
            return

        self._get_gaze()
        self.next_row[0] = time.time() - self.start
        self.all_data.append(self.next_row[:])

    def complete(self):
        if self.start is None:
            return

        home_dir = os.path.expanduser("~")
        target_dir = ["Documents", "MATLAB", "test_data"]
        filename = "p{:02}_n{:02}-{}.csv".format(
            self.participant_number,
            self.scenario_number,
            datetime.datetime.now().strftime("%m_%d"),
        )
        filename = os.path.join(home_dir, *target_dir, filename)

        with open(filename, "w") as f:
            w = csv.writer(f)

            w.writerow(self.headers.keys())
            w.writerows(self.all_data)

        if self.record_gaze:
            self.gaze_reader.close()

    def _get_gaze(self):
        gaze_x, gaze_y = (
            self.gaze_reader.get_gaze() if self.gaze_reader is not None else (0.0, 0.0)
        )

        if gaze_x <= 0.0 or gaze_y <= 0.0:
            return

        self.gaze_x = gaze_x
        self.gaze_y = gaze_y

        self.next_row[self.headers["gaze_x"]] = self.gaze_x
        self.next_row[self.headers["gaze_y"]] = self.gaze_y

    def _on_key_press(self, data: Int8):
        if self.start is None and data.data == K_r:

            self.get_logger().info("\n Recording Begins \n")
            self.start = time.time()

            run_command = create_bag_run()
            self.node_pid = Popen([run_command[0], *run_command[1]]).pid

        elif self.start and (data.data == K_z or data.data == K_e):
            self.get_logger().info("\n End Recording \n")
            self.scenario_number += 1
            self.complete()
            self.start = None
            os.kill(self.node_pid, SIGINT)

    def _record_next_waypoint(self, data):
        self.next_row[self.headers["next_waypoint_x (m)"]] = data.poses[0].position.x
        self.next_row[self.headers["next_waypoint_y (m)"]] = data.poses[0].position.y

    def _record_object_status(self, data: ObjectArray):
        self.ped_locations: List[Tuple[float]] = []
        for obj in data.objects:
            if obj.classification == Object.CLASSIFICATION_CAR:
                self._record_vehicle_object(obj)
            elif obj.classification == Object.CLASSIFICATION_PEDESTRIAN:
                self._store_ped_object(obj)

        self._record_ped_object()

    def _record_vehicle_object(self, obj: Object):
        self.next_row[self.headers["car_x (m)"]] = obj.pose.position.x
        self.next_row[self.headers["car_y (m)"]] = -obj.pose.position.y

        self.next_row[self.headers["car_yaw (degrees)"]] = get_yaw(obj)
        self.next_row[self.headers["car_w (m/s)"]] = obj.twist.angular.z

    def _store_ped_object(self, obj: Object):
        i = 0

        x = obj.pose.position.x
        y = obj.pose.position.y
        type_id = self.world.get_actor(int(obj.id)).type_id[-2:]
        vel = math.sqrt(obj.twist.linear.x ** 2 + obj.twist.linear.y ** 2)

        if len(self.ped_locations) > 0:
            for i in range(len(self.ped_locations) - 1, -1, -1):
                # TODO-KM: This is a placeholder comparison. The best way is to just transform based on id
                if self.ped_locations[i][1] < obj.pose.position.y:
                    i += 1
                    break

        self.ped_locations.insert(i, (x, y, vel, get_yaw(obj), type_id))

    def _record_ped_object(self):
        for i, loc in enumerate(self.ped_locations):
            self._ensure_ped_headers(i)

            self.next_row[self.headers[f"ped{i}_x (m)"]] = loc[0]
            self.next_row[self.headers[f"ped{i}_y (m)"]] = loc[1]
            self.next_row[self.headers[f"ped{i}_v (m/s)"]] = loc[2]
            self.next_row[self.headers[f"ped{i}_yaw (degrees)"]] = loc[3]
            self.next_row[self.headers[f"ped{i}_id"]] = loc[4]
            self.next_row[self.headers[f"ped{i}_val"]] = ID_TO_VAL[loc[4]]

    def _record_vehicle_status(self, data):
        self.next_row[self.headers["car_v (m/s)"]] = data.velocity
        self.next_row[
            self.headers["response_theta (±turn % max 200)"]
        ] = data.control.steer
        self.next_row[self.headers["throttle (%)"]] = data.control.throttle
        self.next_row[self.headers["break (%)"]] = data.control.brake

    def _record_auton_status(self, data):
        self.next_row[self.headers["is_autonomous"]] = float(not data.data)

    def _record_fov(self, data: CameraInfo):
        # FOCAL_LENGTH = IMAGE_WIDTH / (2.0 * np.tan(FIELD_OF_VIEW * np.pi / 360.0))

        self.next_row[self.headers["cam_flx"]] = data.k[0]
        self.next_row[self.headers["cam_fly"]] = data.k[4]

    def _record_torque(self, data):
        self.next_row[self.headers["torque (N)"]] = data.torque

    def _record_target_steer(self, data):
        self.next_row[
            self.headers["command_theta  (±turn % max 100)"]
        ] = data.position  # command theta

    def _record_sim_number(self, data):
        self.next_row[
            self.headers["to_sim_theta  (±turn % max 200)"]
        ] = data.data  # sent to simulator

    def _record_ts_number(self, data):
        self.next_row[
            self.headers["controller_value_theta (±turn % max 100)"]
        ] = data.data  # true steer

    def _record_ped_2d_transform(self, data: CarlaBoundingBoxArray):
        sorted_objects: List[CarlaBoundingBox] = sorted(
            data.boxes, key=lambda bbox: bbox.center.x
        )

        for i, bbox in enumerate(sorted_objects):
            self._ensure_ped_headers(i)

            self.next_row[self.headers[f"ped{i}_cx"]] = bbox.center.x / data.width
            self.next_row[self.headers[f"ped{i}_cy"]] = bbox.center.y / data.height

    def _ensure_ped_headers(self, i):
        pedestrian_headers = [
            f"ped{i}_x (m)",
            f"ped{i}_y (m)",
            f"ped{i}_v (m/s)",
            f"ped{i}_yaw (degrees)",
            f"ped{i}_cx",
            f"ped{i}_cy",
            f"ped{i}_id",
            f"ped{i}_val",
        ]

        existing_headers = set(self.headers.keys())
        for header in pedestrian_headers:
            if header not in existing_headers:
                self.headers[header] = len(self.next_row)
                self.next_row.append(0.0)


def get_yaw(obj: Object):
    orient = obj.pose.orientation
    quaternion = (orient.w, orient.x, orient.y, orient.z)
    _, _, yaw = quat2euler(quaternion)
    return math.degrees(yaw)


ID_TO_VAL = {
    1: (6000.0, 15000.0, 1000000.0), # Soldier
    12: (2000.0, 10000.0, 85000.0), # Orphan
    15: (4000.0, 10000.0, 100000.0), # Police
    16: (0.0, 0.0, 100.0), # Terrorist
    20: (0.0, 0.0, 100.0), # Pedophile
    23: (0.0, 0.0, 11.25), # Rapist
    25: (1000.0, 5000.0, 10000.0), # Judge
    26: (1000.0, 4000.0, 50000.0), # Billionaire
    30: (1000.0, 3000.0, 20000.0), # Celebrity
}


def create_bag_run():
    return (
        "ros2",
        [
            "bag",
            "record",
            "/resized_semantic",
            "/resized_rgb",
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
        ],
    )


def main(args=None):
    rclpy.init(args=args)

    orchestrator = RecordingOrchestrator()

    try:
        orchestrator.create_timer(0.1, lambda _=None: orchestrator.write())

        rclpy.spin(orchestrator, orchestrator.executor)
    except KeyboardInterrupt:
        pass

    os.kill(orchestrator.node_pid, SIGINT)
    orchestrator.complete()
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
