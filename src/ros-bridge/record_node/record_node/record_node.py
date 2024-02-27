#!/usr/bin/env python3
import csv
import os
import time, datetime
import math

import rclpy
from rclpy.node import Node

from .gaze_reader import GazeReader
from .img_publisher import ImageView

from carla_msgs.msg import CarlaEgoVehicleStatus
from ros_g29_force_feedback.msg import ForceControl, ForceFeedback

from derived_object_msgs.msg import ObjectArray, Object
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, Float32

from transforms3d.euler import quat2euler


class RecordingOrchestrator(Node):
    def __init__(self):
        super().__init__("recording_orchestrator")

        self._init_parmas()
        self.gaze_reader = GazeReader() if self.record_gaze else None
        self.gaze_x = self.gaze_y = 0.0

        # TODO: pass draw params
        self.img = ImageView(self, self.draw_manctrl, self.draw_gaze)

        self.header = [
            "time (ms since start)",  # 0
            "car_x (world frame x)",  # 1
            "car_y (world frame y)",  # 2
            "car_yaw (degrees, 0->180 0->-180)",  # 3
            "car_v (m/s)",  # 4
            "car_w (m/s)",  # 5
            "throttle (depressed %)",  # 6
            "break (depressed %)",  # 7
            "torque (N on wheel motor)",  # 8
            "response_theta (±turn % max 200)",  # 9
            "true_theta (±turn % max 100)",  # 10
            "command_theta  (±turn % max 100)",  # 11
            "sim_theta  (±turn % max 200)",  # 12
            "is_autonomous (1 is auton, 0 is manual)",  # 13
            "gaze_x (0 is left)",  # 14
            "gaze_y (0 is top)",  # 15
            "next_waypoint_x (in world frame)",  # 16
            "next_waypoint_y (in world frame)",  # 17
            "ped0_x (in world frame)",  # 18
            "ped0_y (in world frame)",  # 19
            "ped1_x (in world frame)",  # 20
            "ped1_y (in world frame)",  # 21
            "ped2_x (in world frame)",  # 22
            "ped2_y (in world frame)",  # 23
            "ped3_x (in world frame)",  # 24
            "ped3_y (in world frame)",  # 25
        ]
        self.all_data = []
        self.next_row = [0.0] * len(self.header)

        self._init_pub_sub()

        self.start = None

    def _init_parmas(self):
        self.declare_parameter("record_gaze", "False")
        self.declare_parameter("participant_number", "1")
        self.declare_parameter("test_number", "1")

        self.declare_parameter("draw_manctrl", "True")
        self.declare_parameter("draw_gaze", "False")
        self.declare_parameter("draw_outline", "False")
        self.declare_parameter("draw_route", "False")

        self.record_gaze = bool(self.get_parameter("record_gaze").value)
        self.participant_number = int(self.get_parameter("participant_number").value)
        self.test_number = int(self.get_parameter("test_number").value)

        self.draw_manctrl = bool(self.get_parameter("draw_manctrl").value)
        self.draw_gaze = bool(self.get_parameter("draw_gaze").value)
        self.draw_outline = bool(self.get_parameter("draw_outline").value)
        self.draw_route = bool(self.get_parameter("draw_route").value)

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

            w.writerow(self.header)
            w.writerows(self.all_data)

    def _get_gaze(self):
        gaze_x, gaze_y = (
            self.gaze_reader.get_gaze() if self.gaze_reader is not None else (0.0, 0.0)
        )

        if gaze_x > 0.0 or gaze_y > 0.0:
            self.gaze_x = gaze_x
            self.gaze_y = gaze_y

            self.img.update_gaze(gaze_x, gaze_y)

        if gaze_x and gaze_y:
            self.next_row[14] = self.gaze_x
            self.next_row[15] = self.gaze_y

    def _record_next_waypoint(self, data):
        self.next_row[16] = data.poses[0].position.x
        self.next_row[17] = data.poses[0].position.y

    def _record_object_status(self, data):
        ped_i = 0
        for obj in data.objects:
            if obj.classification == Object.CLASSIFICATION_CAR:
                self.start = time.time()

                self.next_row[1] = obj.pose.position.x
                self.next_row[2] = -obj.pose.position.y

                quaternion = (
                    obj.pose.orientation.w,
                    obj.pose.orientation.x,
                    obj.pose.orientation.y,
                    obj.pose.orientation.z,
                )
                _, _, yaw = quat2euler(quaternion)
                self.next_row[3] = math.degrees(yaw)

                self.next_row[5] = obj.twist.angular.z

            elif obj.classification == Object.CLASSIFICATION_PEDESTRIAN:
                self.next_row[18 + ped_i * 2] = obj.pose.position.x
                self.next_row[19 + ped_i * 2] = obj.pose.position.y
                ped_i += 1

    def _record_vehicle_status(self, data):
        self.next_row[4] = data.velocity
        self.next_row[9] = data.control.steer  # return theta
        self.next_row[6] = data.control.throttle
        self.next_row[7] = data.control.brake

    def _record_auton_status(self, data):
        self.next_row[13] = float(not data.data)

    def _record_torque(self, data):
        self.next_row[8] = data.torque

    def _record_target_steer(self, data):
        self.next_row[11] = data.position  # command theta

    def _record_sim_number(self, data):
        self.next_row[12] = data.data  # sent to simulator

    def _record_ts_number(self, data):
        self.next_row[10] = data.data  # true steer


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
