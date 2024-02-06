#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Base class for agent
"""

import enum
import math
import numpy as np

import carla

import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.callback_groups import MutuallyExclusiveCallbackGroup
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from agents.navigation.global_route_planner import GlobalRoutePlanner

from carla_ad_agent.misc import (
    is_within_distance_ahead,
    distance_vehicle,
)  # pylint: disable=relative-import

from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaTrafficLightStatus
from carla_waypoint_types.srv import GetWaypoint
from derived_object_msgs.msg import Object
from std_msgs.msg import Float64


class AgentState(enum.Enum):
    """
    AGENT_STATE represents the possible states of a roaming agent
    """

    NAVIGATING = 1
    BLOCKED_BY_VEHICLE = 2
    BLOCKED_RED_LIGHT = 3
    BLOCKED_BY_PEDESTRIAN = 4


class Agent(CompatibleNode):
    """
    Base class for agent
    """

    OBJECT_VEHICLE_CLASSIFICATION = [
        # Object.CLASSIFICATION_PEDESTRIAN,
        Object.CLASSIFICATION_CAR,
        Object.CLASSIFICATION_BIKE,
        Object.CLASSIFICATION_MOTORCYCLE,
        Object.CLASSIFICATION_TRUCK,
        Object.CLASSIFICATION_OTHER_VEHICLE,
    ]

    def __init__(self, name="agent"):
        super(Agent, self).__init__(name)

        self._proximity_tlight_threshold = 10.0  # meters
        self._proximity_vehicle_threshold = 12.0  # meters

        role_name = self.get_param("role_name", "ego_vehicle")
        self.loginfo("Waiting for vehicle_info...")
        vehicle_info = self.wait_for_message(
            "/carla/{}/vehicle_info".format(role_name),
            CarlaEgoVehicleInfo,
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )
        self.loginfo("Vehicle info received.")
        self._ego_vehicle_id = vehicle_info.id

        self._get_waypoint_client = self.new_client(
            GetWaypoint,
            "/carla_waypoint_publisher/{}/get_waypoint".format(role_name),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._ped_dist_publisher = self.new_publisher(
            Float64, "/carla/{}/hazard_distance".format(role_name), 10
        )
        self.desired_gap = 40.0

    def get_waypoint(self, location):
        """
        Helper method to get a waypoint for a location.

        :param location: location request
        :type location: geometry_msgs/Point
        :return: waypoint of the requested location
        :rtype: carla_msgs/Waypoint
        """
        if not roscomp.ok():
            return None
        try:
            request = roscomp.get_service_request(GetWaypoint)
            request.location = location
            response = self.call_service(self._get_waypoint_client, request)
            return response.waypoint
        except ServiceException as e:
            if roscomp.ok():
                self.logwarn("Service call 'get_waypoint' failed: {}".format(str(e)))

    def run_step():
        """
        Executes one step of navigation.
        """
        raise NotImplementedError

    def _is_vehicle_hazard(self, ego_vehicle_pose, objects):
        """
        Checks whether there is a vehicle hazard.

        This method only takes into account vehicles. Pedestrians are other types of obstacles are
        ignored.

        :param ego_vehicle_pose: current ego vehicle pose
        :type ego_vehicle_pose: geometry_msgs/Pose
        :param objects: list of objects
        :type objects: derived_object_msgs/ObjectArray
        :return: a tuple given by (bool_flag, vehicle), where
                 - bool_flag is True if there is a vehicle ahead blocking us
                   and False otherwise
                 - vehicle is the blocker vehicle id
        """

        ego_vehicle_waypoint = self.get_waypoint(ego_vehicle_pose.position)
        ego_vehicle_transform = trans.ros_pose_to_carla_transform(ego_vehicle_pose)

        for target_vehicle_id, target_vehicle_object in objects.items():
            # take into account only vehicles
            if (
                target_vehicle_object.classification
                not in self.OBJECT_VEHICLE_CLASSIFICATION
            ):
                continue

            # do not account for the ego vehicle
            if target_vehicle_id == self._ego_vehicle_id:
                continue

            target_vehicle_waypoint = self.get_waypoint(
                target_vehicle_object.pose.position
            )
            if target_vehicle_waypoint is None:
                continue
            target_vehicle_transform = trans.ros_pose_to_carla_transform(
                target_vehicle_object.pose
            )

            # if the object is not in our lane it's not an obstacle
            if (
                target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id
                or target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id
            ):
                continue

            if is_within_distance_ahead(
                target_vehicle_transform,
                ego_vehicle_transform,
                self._proximity_vehicle_threshold,
            ):
                return (True, target_vehicle_id)

        return (False, None)

    def _is_pedestrian_hazard(self, ego_vehicle_pose, objects):
        ego_vehicle_waypoint = self.get_waypoint(ego_vehicle_pose.position)

        closest_ped = Float64()
        closest_ped.data = 0.0

        pedestrian_status = (False, None)

        for target_pedestrian_id, target_pedestrian_obj in objects.items():
            if target_pedestrian_obj.classification != Object.CLASSIFICATION_PEDESTRIAN:
                continue

            target_waypoint = self.get_waypoint(target_pedestrian_obj.pose.position)

            if (
                target_waypoint.road_id
                != ego_vehicle_waypoint.road_id
                # or target_waypoint.lane_id != ego_vehicle_waypoint.lane_id
            ):
                continue

            distance_ahead = target_waypoint.s_val - ego_vehicle_waypoint.s_val
            absolute_distance = distance_vehicle(
                target_pedestrian_obj.pose, ego_vehicle_pose.position
            )

            hazard_distance = min(distance_ahead, self.desired_gap) - self.desired_gap

            is_in_lane = (
                distance_vehicle(
                    target_waypoint.pose, target_pedestrian_obj.pose.position
                )
                < target_waypoint.lane_width / 2
            )

            target_transform = trans.ros_pose_to_carla_transform(
                target_pedestrian_obj.pose
            )
            target_waypoint_transform = trans.ros_pose_to_carla_transform(
                target_waypoint.pose
            )

            is_crossing_lane = is_within_distance_ahead(
                target_waypoint_transform, target_transform, 2.0
            )

            if is_in_lane or is_crossing_lane:
                if absolute_distance < 5.0 and distance_ahead > -3:
                    pedestrian_status = (True, target_pedestrian_id)
                    break
                else:
                    closest_ped.data = (
                        hazard_distance
                        if hazard_distance < closest_ped.data
                        else closest_ped.data
                    )

        self._ped_dist_publisher.publish(closest_ped)
        return pedestrian_status

    def _is_light_red(self, ego_vehicle_pose, lights_status, lights_info):
        """
        Checks if there is a red light affecting us. This version of the method is compatible with
        both European and US style traffic lights.

        :param ego_vehicle_pose: current ego vehicle pose
        :type ego_vehicle_pose: geometry_msgs/Pose

        :param lights_status: list containing all traffic light status.
        :type lights_status: carla_msgs/CarlaTrafficLightStatusList

        :param lights_info: list containing all traffic light info.
        :type lights_info: lis.

        :return: a tuple given by (bool_flag, traffic_light), where
                 - bool_flag is True if there is a traffic light in RED
                   affecting us and False otherwise
                 - traffic_light is the traffic light id or None if there is no
                   red traffic light affecting us.
        """
        ego_vehicle_waypoint = self.get_waypoint(ego_vehicle_pose.position)
        ego_vehicle_transform = trans.ros_pose_to_carla_transform(ego_vehicle_pose)

        for light_id in lights_status.keys():
            object_location = self._get_trafficlight_trigger_location(
                lights_info[light_id]
            )
            object_location = trans.carla_location_to_ros_point(object_location)
            object_waypoint = self.get_waypoint(object_location)
            if object_waypoint is None:
                continue
            object_transform = trans.ros_pose_to_carla_transform(object_waypoint.pose)

            if object_waypoint.road_id != ego_vehicle_waypoint.road_id:
                continue

            ve_dir = ego_vehicle_transform.get_forward_vector()
            wp_dir = object_transform.get_forward_vector()

            dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

            if dot_ve_wp < 0:
                continue

            if is_within_distance_ahead(
                object_transform,
                ego_vehicle_transform,
                self._proximity_tlight_threshold,
            ):
                if (
                    lights_status[light_id].state == CarlaTrafficLightStatus.RED
                    or lights_status[light_id].state == CarlaTrafficLightStatus.YELLOW
                ):
                    return (True, light_id)

        return (False, None)

    def _get_trafficlight_trigger_location(
        self, light_info
    ):  # pylint: disable=no-self-use
        def rotate_point(point, radians):
            """
            rotate a given point by a given angle
            """
            rotated_x = math.cos(radians) * point.x - math.sin(radians) * point.y
            rotated_y = math.sin(radians) * point.x + math.cos(radians) * point.y

            return carla.Vector3D(rotated_x, rotated_y, point.z)

        base_transform = trans.ros_pose_to_carla_transform(light_info.transform)
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(
            trans.ros_point_to_carla_location(light_info.trigger_volume.center)
        )
        area_ext = light_info.trigger_volume.size

        point = rotate_point(
            carla.Vector3D(0, 0, area_ext.z / 2.0), math.radians(base_rot)
        )
        point_location = area_loc + carla.Location(x=point.x, y=point.y)

        return carla.Location(point_location.x, point_location.y, point_location.z)
