#!/usr/bin/env python3
import csv
import os
import time
import datetime

import rclpy
from rclpy.node import Node

from carla_msgs.msg import Gazedata
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaCollisionEvent
from derived_object_msgs.msg import ObjectArray


class RecordingOrchestrator(Node):
    def __init__(self):
        super().__init__("recording_orchestrator")

        # Vehile recorder
        self.vehicle_recorder = VehicleWriter(self)
        # Pedestrain recorder
        # Gaze recorder

        self.create_timer(0.1, self.write)

    def write(self):
        self.vehicle_recorder.write_row()


class BaseWriter:
    row = []

    def __init__(self, name, header):
        self.file = open(self._filename(name), mode="a", newline="")
        self.csv_writer = csv.writer(self.file)

        self.create_file(header)

    def create_file(self, header):
        self.csv_writer.writerow(header)

    def write_row(self):
        self.csv_writer.writerow(self.row)

    def close_file(self):
        self.file.close()

    def _filename(self, name):
        home_dir = os.path.expanduser("~")
        target_dir = ("expiremental_recordings", name)
        return os.path.join(
            home_dir, *target_dir, datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        )


class GazeWriter(BaseWriter):
    def __init__(self):
        super().__init__("gaze_recording", ["timestamp", "x", "y", "v", "d"])

        self.carla_gaze_subscriber = self.node.new_subscription(
            Gazedata, "/gaze_publisher", self.record_row, qos_profile=10
        )

    def record_row(self, data):
        self.write_row([data.pog_x, data.pog_y, data.pog_v, data.pog_d])


class VehicleWriter(BaseWriter):
    data = {}

    def __init__(self, node):
        super().__init__(
            "vehicle_recording",
            [
                "timestamp",
                "x",
                "y",
                "z",
                "velocity",
                "steering",
                "throttle",
                "brake",
                "reverse",
                "collision",
                "lane_crossing",
            ],
        )
        self.node = node

        self._reset()

        self.carla_vehicle_status_subscriber = self.node.create_subscription(
            CarlaEgoVehicleStatus,
            f"/carla/{'ego_vehicle'}/vehicle_status",
            self._record_vehicle_status,
            qos_profile=10,
        )
        self.carla_vehicle_status_subscriber = self.node.create_subscription(
            ObjectArray, "/carla/objects", self._record_object_status, qos_profile=10
        )
        self.carla_vehicle_status_subscriber = self.node.create_subscription(
            CarlaCollisionEvent,
            f"/carla/{'ego_vehicle'}/collision",
            self._record_collision,
            qos_profile=10,
        )

    def write_row(self):
        required_keys = ["x", "y", "z"]

        if any(self.data.get(key) is None for key in required_keys):
            return

        self.row = [
            time.time(),
            self.data["x"],
            self.data["y"],
            self.data["z"],
            self.data["velocity"],
            self.data["steering"],
            self.data["throttle"],
            self.data["brake"],
            self.data["reverse"],
            self.data["collision"],
        ]
        super().write_row()
        self._reset()

    def _reset(self):
        self.data["collision"] = 0

    def _record_vehicle_status(self, data):
        self.data["velocity"] = data.velocity
        self.data["steering"] = data.control.steer
        self.data["throttle"] = data.control.throttle
        self.data["brake"] = data.control.brake
        self.data["reverse"] = data.control.gear == -1

    def _record_object_status(self, data):
        for obj in data.objects:
            if not obj.classification == 6:
                continue

            self.data["x"] = obj.pose.position.x
            self.data["y"] = obj.pose.position.y
            self.data["z"] = obj.pose.position.z

    def _record_collision(self, data):
        i = data.normal_impulse
        self.data["collision"] = (i.x * i.x + i.y * i.y + i.z * i.z) ** 0.5


def main(args=None):
    rclpy.init(args=args)

    orchestrator = RecordingOrchestrator()

    rclpy.spin(orchestrator)

    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
