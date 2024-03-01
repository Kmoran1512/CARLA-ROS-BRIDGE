#!/usr/bin/env python3
import csv
import os
import time, datetime
import math
import rclpy

from rclpy.node import Node
from transforms3d.euler import quat2euler
from ament_index_python.packages import get_package_share_directory

from .gaze_reader import GazeReader

from carla_msgs.msg import (
    CarlaBoundingBox,
    CarlaBoundingBoxArray,
    CarlaEgoVehicleStatus,
)
from derived_object_msgs.msg import ObjectArray, Object
from geometry_msgs.msg import PoseArray
from ros_g29_force_feedback.msg import ForceControl, ForceFeedback
from std_msgs.msg import Bool, Float32


class RecordingOrchestrator(Node):
    def __init__(self):
        super().__init__("recording_orchestrator")

        self.headers = {}
        self._init_parmas()
        self.gaze_reader = GazeReader() if self.record_gaze else None
        self.gaze_x = self.gaze_y = 0.0

        self.all_data = []
        self.next_row = [0.0] * len(self.headers)

        self._init_pub_sub()

        self.start = None

    def _init_parmas(self):
        self.declare_parameter("record_gaze", "False")
        self.declare_parameter("participant_number", "1")
        self.declare_parameter("test_number", "1")

        self.record_gaze = bool(self.get_parameter("record_gaze").value)
        self.participant_number = int(self.get_parameter("participant_number").value)
        self.test_number = int(self.get_parameter("test_number").value)

        header_file = os.path.join(
            get_package_share_directory("record_node"), "config", "column_headers.txt"
        )

        with open(header_file, "r") as f:
            for i, line in enumerate(f.read().splitlines()):
                self.headers[line] = i

    def _init_pub_sub(self):
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
        self.create_subscription(Float32, "/ts_pub", self._record_ts_number, 10)
        self.create_subscription(Float32, "/sim_pub", self._record_sim_number, 10)
        self.create_subscription(
            CarlaBoundingBoxArray,
            "/carla/bounding_boxes",
            self._record_ped_2d_transform,
            10,
        )

    def write(self):
        if self.start is None:
            return

        self._get_gaze()
        self.next_row[0] = time.time() - self.start
        self.all_data.append(self.next_row[:])

    def complete(self):
        home_dir = os.path.expanduser("~")
        target_dir = ["Documents", "MATLAB", "test_data"]
        filename = "p{:02}_n{:02}-{}.csv".format(
            self.participant_number,
            self.test_number,
            datetime.datetime.now().strftime("%m_%d"),
        )
        filename = os.path.join(home_dir, *target_dir, filename)

        with open(filename, "w") as f:
            w = csv.writer(f)

            w.writerow(self.headers.keys())
            w.writerows(self.all_data)

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

    def _record_next_waypoint(self, data):
        self.next_row[self.headers["next_waypoint_x (m)"]] = data.poses[0].position.x
        self.next_row[self.headers["next_waypoint_y (m)"]] = data.poses[0].position.y

    def _record_object_status(self, data):
        ped_i = 0
        for obj in data.objects:
            if obj.classification == Object.CLASSIFICATION_CAR:
                if self.start is None:
                    self.start = time.time()
                self._record_vehicle_object(obj)
            elif obj.classification == Object.CLASSIFICATION_PEDESTRIAN:
                self._record_ped_object(obj, ped_i)
                ped_i += 1

    def _record_vehicle_object(self, obj):
        self.next_row[self.headers["car_x (m)"]] = obj.pose.position.x
        self.next_row[self.headers["car_y (m)"]] = -obj.pose.position.y

        quaternion = (
            obj.pose.orientation.w,
            obj.pose.orientation.x,
            obj.pose.orientation.y,
            obj.pose.orientation.z,
        )
        _, _, yaw = quat2euler(quaternion)
        self.next_row[self.headers["car_yaw (degrees)"]] = math.degrees(yaw)
        self.next_row[self.headers["car_w (m/s)"]] = obj.twist.angular.z

    def _record_ped_object(self, obj, n):
        self.next_row[self.headers[f"ped{n}_x (m)"]] = obj.pose.position.x
        self.next_row[self.headers[f"ped{n}_y (m)"]] = obj.pose.position.y

    def _record_vehicle_status(self, data):
        self.next_row[self.headers["car_v (m/s)"]] = data.velocity
        self.next_row[
            self.headers["response_theta (±turn % max 200)"]
        ] = data.control.steer
        self.next_row[self.headers["throttle (%)"]] = data.control.throttle
        self.next_row[self.headers["break (%)"]] = data.control.brake

    def _record_auton_status(self, data):
        self.next_row[self.headers["is_autonomous"]] = float(not data.data)

    def _record_torque(self, data):
        self.next_row[self.headers["torque (N)"]] = data.torque

    def _record_target_steer(self, data):
        self.next_row[
            self.headers["command_theta  (±turn % max 100)"]
        ] = data.position  # command theta

    def _record_sim_number(self, data):
        self.next_row[
            self.headers["sim_theta  (±turn % max 200)"]
        ] = data.data  # sent to simulator

    def _record_ts_number(self, data):
        self.next_row[
            self.headers["true_theta (±turn % max 100)"]
        ] = data.data  # true steer

    def _record_ped_2d_transform(self, data: CarlaBoundingBoxArray):
        ped_id = 0
        for bbox in data.boxes:
            bbox: CarlaBoundingBox
            self.next_row[self.headers[f"ped{ped_id}_cx"]] = bbox.center.x
            self.next_row[self.headers[f"ped{ped_id}_cy"]] = bbox.center.y
            ped_id += 1


def main(args=None):
    rclpy.init(args=args)

    orchestrator = RecordingOrchestrator()

    try:
        orchestrator.create_timer(0.1, lambda _=None: orchestrator.write())

        rclpy.spin(orchestrator, orchestrator.executor)
    except KeyboardInterrupt:
        pass

    orchestrator.complete()
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
