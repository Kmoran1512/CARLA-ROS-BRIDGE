#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
A basic AD agent using CARLA waypoints
"""

import copy
import sys
import time
import threading

import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_ad_agent.agent import Agent, AgentState

from carla_msgs.msg import (
    CarlaEgoVehicleStatus,
    CarlaTrafficLightStatusList,
    CarlaTrafficLightInfoList,
)
from derived_object_msgs.msg import ObjectArray, Object
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64  # pylint: disable=import-error


class CarlaAdAgent(Agent):
    """
    A basic AD agent using CARLA waypoints
    """

    def __init__(self):
        """
        Constructor
        """
        super(CarlaAdAgent, self).__init__("ad_agent")

        role_name = self.get_param("role_name", "ego_vehicle")
        self._avoid_risk = self.get_param("avoid_risk", True)
        self._avoid_pedestrians = self.get_param("avoid_pedestrians", True)

        self.data_lock = threading.Lock()

        self._ego_vehicle_pose = None
        self._rotatation_direction = None
        self._objects = {}
        self._lights_status = {}
        self._lights_info = {}
        self._target_speed = 0.0
        self._next_50 = []

        self.speed_command_publisher = self.new_publisher(
            Float64,
            "/carla/{}/speed_command".format(role_name),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self._odometry_subscriber = self.new_subscription(
            Odometry,
            "/carla/{}/odometry".format(role_name),
            self.odometry_cb,
            qos_profile=10,
        )

        self._target_speed_subscriber = self.new_subscription(
            Float64,
            "/carla/{}/target_speed".format(role_name),
            self.target_speed_cb,
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        self._objects_subscriber = self.new_subscription(
            ObjectArray,
            "/carla/{}/objects".format(role_name),
            self.objects_cb,
            qos_profile=10,
        )

        if self._avoid_risk:
            self._traffic_light_status_subscriber = self.new_subscription(
                CarlaTrafficLightStatusList,
                "/carla/traffic_lights/status",
                self.traffic_light_status_cb,
                qos_profile=QoSProfile(
                    depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
                ),
            )
            self._traffic_light_info_subscriber = self.new_subscription(
                CarlaTrafficLightInfoList,
                "/carla/traffic_lights/info",
                self.traffic_light_info_cb,
                qos_profile=QoSProfile(
                    depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
                ),
            )

            self.vehicle_status_subscriber = self.new_subscription(
                CarlaEgoVehicleStatus,
                "/carla/{}/vehicle_status".format(role_name),
                self.vehicle_status_updated,
                qos_profile=10,
            )
            self.way_sub = self.new_subscription(
                PoseArray,
                "/carla/{}/next_50".format(role_name),
                self.waypoints_updated,
                qos_profile=10,
            )

    def odometry_cb(self, odometry_msg):
        with self.data_lock:
            self._ego_vehicle_pose = odometry_msg.pose.pose

    def target_speed_cb(self, target_speed_msg):
        with self.data_lock:
            self._target_speed = (
                target_speed_msg.data * 3.6
            )  # target speed from scenario is in m/s

    def objects_cb(self, objects_msg):
        objects = {}
        for obj in objects_msg.objects:
            objects[obj.id] = obj

        with self.data_lock:
            self._objects = objects

    def traffic_light_status_cb(self, traffic_light_status_msg):
        lights_status = {}
        for tl_status in traffic_light_status_msg.traffic_lights:
            lights_status[tl_status.id] = tl_status

        with self.data_lock:
            self._lights_status = lights_status

    def traffic_light_info_cb(self, traffic_light_info_msg):
        lights_info = {}
        for tl_info in traffic_light_info_msg.traffic_lights:
            lights_info[tl_info.id] = tl_info

        with self.data_lock:
            self._lights_info = lights_info

    def vehicle_status_updated(self, vehicle_status_msg):
        steer_angle = vehicle_status_msg.control.steer
        with self.data_lock:
            self._rotatation_direction = -steer_angle

    def waypoints_updated(self, path):
        with self.data_lock:
            self._next_50 = path.poses

    def emergency_stop(self):
        stopping_speed = Float64()
        stopping_speed.data = 0.0
        self.speed_command_publisher.publish(stopping_speed)

    def run_step(self):
        """
        Executes one step of navigation.
        """

        # is there an obstacle in front of us?
        hazard_detected = False

        with self.data_lock:
            # retrieve relevant elements for safe navigation, i.e.: traffic lights and other vehicles.
            ego_vehicle_pose = copy.deepcopy(self._ego_vehicle_pose)
            objects = copy.deepcopy(self._objects)
            lights_info = copy.deepcopy(self._lights_info)
            lights_status = copy.deepcopy(self._lights_status)
            target_speed = copy.deepcopy(self._target_speed)

        if ego_vehicle_pose is None:
            self.loginfo("Waiting for ego vehicle pose")
            return

        # ensure we have received all the status/info data of traffic lights.
        if set(lights_info.keys()) != set(lights_status.keys()):
            self.logwarn("Missing traffic light information")
            return

        if self._avoid_risk:
            # check possible obstacles
            vehicle_state, vehicle = self._is_vehicle_hazard(ego_vehicle_pose, objects)
            if vehicle_state:
                self._state = AgentState.BLOCKED_BY_VEHICLE
                hazard_detected = True

            if self._avoid_pedestrians:
                pedestrian_state, pedestrian = self._is_pedestrian_hazard(
                    ego_vehicle_pose, self._rotatation_direction, objects, self._next_50
                )
                if pedestrian_state:
                    self._state = AgentState.BLOCKED_BY_PEDESTRIAN
                    hazard_detected = True

            # check for the state of the traffic lights
            light_state, traffic_light = self._is_light_red(
                ego_vehicle_pose, lights_status, lights_info
            )
            if light_state:
                self._state = AgentState.BLOCKED_RED_LIGHT
                hazard_detected = True

        self.get_ped_transform(objects)

        speed_command = Float64()
        if hazard_detected:
            speed_command.data = 0.0
        else:
            self._state = AgentState.NAVIGATING
            speed_command.data = target_speed

        self.speed_command_publisher.publish(speed_command)


def main(args=None):
    """

    main function

    :return:
    """
    roscomp.init("ad_agent", args=args)
    controller = None
    try:
        executor = roscomp.executors.MultiThreadedExecutor()
        controller = CarlaAdAgent()
        executor.add_node(controller)

        roscomp.on_shutdown(controller.emergency_stop)

        update_timer = controller.new_timer(
            0.05, lambda timer_event=None: controller.run_step()
        )

        controller.spin()

    except (ROSInterruptException, ROSException) as e:
        if roscomp.ok():
            roscomp.logwarn("ROS Error during exection: {}".format(e))
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
