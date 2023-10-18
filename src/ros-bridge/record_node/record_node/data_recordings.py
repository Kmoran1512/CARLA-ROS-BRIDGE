#!/usr/bin/env python3
import csv
import os
import time
import datetime

import rclpy
from rclpy.node import Node

from record_node.gaze_publisher import GazePublisher

from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaCollisionEvent
from derived_object_msgs.msg import ObjectArray


class RecordingOrchestrator(Node):
    def __init__(self):
        super().__init__("recording_orchestrator")

        self.vehicle_recorder = VehicleWriter(self)
        #self.gaze_recorder = GazeWriter(self)
        self.ped_recorder = PedestrianWriter(self)

        self.create_timer(0.1, self.write)

    def write(self):
        #self.gaze_recorder.write_row()
        self.vehicle_recorder.write_row()
        self.ped_recorder.write_row()


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
    gaze_data = []

    def __init__(self, node):
        super().__init__("gaze_recording", ["timestamp", "x", "y", "v", "d"])
        self.node = node

        self.gaze_publisher = GazePublisher()
        self.node.create_timer(0.1, self.get_gaze)


    def get_gaze(self):
        gaze_data = self.gaze_publisher.get_gaze()
        if gaze_data:
            self.gaze_data = gaze_data

    def write_row(self):
        if not self.gaze_data: return

        self.row = [time.time(), *self.gaze_data]

        super().write_row()



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


class PedestrianWriter(BaseWriter):
    data = {}

    def __init__(self, node):
        super().__init__(
            "ped_recording",
            [
                "timestamp",
                "pwx",
                "pwy",
                "pwz",
                "p1x",
                "p1y",
                "p1z",
                "p2x",
                "p2y",
                "p2z",
                "p3x",
                "p3y",
                "p3z",
            ],
        )
        self.node = node

        self.carla_object_subscriber = self.node.create_subscription(
            ObjectArray, "/carla/objects", self._record_object_status, qos_profile=10
        )

    def write_row(self):
        self.row = [
            time.time(),
            self.data["pwx"],
            self.data["pwy"],
            self.data["pwz"],
            self.data["p1x"],
            self.data["p1y"],
            self.data["p1z"],
            self.data["p2x"],
            self.data["p2y"],
            self.data["p2z"],
            self.data["p3x"],
            self.data["p3y"],
            self.data["p3z"],
        ]

        super().write_row()

    def _record_object_status(self, data):
        for obj in data.objects:
            if not obj.classification == 4:
                continue

            ped_id = 'pw'
            if obj.id == 955 : ped_id = 'p1'
            elif obj.id == 956 : ped_id = 'p2'
            elif obj.id == 957 : ped_id = 'p3'




            self.data[ped_id + "x"] = obj.pose.position.x
            self.data[ped_id + "y"] = obj.pose.position.y
            self.data[ped_id + "z"] = obj.pose.position.z




def main(args=None):
    rclpy.init(args=args)

    orchestrator = RecordingOrchestrator()

    rclpy.spin(orchestrator)

    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
