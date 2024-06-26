#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""

import carla_common.transforms as trans

from carla_ros_bridge.traffic_participant import TrafficParticipant

from carla import VehicleControl
from carla_msgs.msg import CarlaEgoVehicleControl
from derived_object_msgs.msg import Object
from std_msgs.msg import ColorRGBA


class Vehicle(TrafficParticipant):

    """
    Actor implementation details for vehicles
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        """
        self.classification = Object.CLASSIFICATION_CAR
        if "object_type" in carla_actor.attributes:
            if carla_actor.attributes["object_type"] == "car":
                self.classification = Object.CLASSIFICATION_CAR
            elif carla_actor.attributes["object_type"] == "bike" or name.startswith(
                "bike"
            ):
                self.classification = Object.CLASSIFICATION_BIKE
            elif carla_actor.attributes["object_type"] == "motorcycle":
                self.classification = Object.CLASSIFICATION_MOTORCYCLE
            elif carla_actor.attributes["object_type"] == "truck":
                self.classification = Object.CLASSIFICATION_TRUCK
            elif carla_actor.attributes["object_type"] == "other":
                self.classification = Object.CLASSIFICATION_OTHER_VEHICLE

        super(Vehicle, self).__init__(
            uid=uid, name=name, parent=parent, node=node, carla_actor=carla_actor
        )

        if name not in self.node.parameters["ego_vehicle"]["role_name"]:
            self.node.new_subscription(
                CarlaEgoVehicleControl,
                self.get_topic_prefix() + "/secondary_vehicle_control",
                self.control_command_updated,
                10,
            )

    def control_command_updated(self, data: CarlaEgoVehicleControl):
        vehicle_control = VehicleControl()
        vehicle_control.throttle = data.throttle
        vehicle_control.steer = data.steer
        vehicle_control.brake = data.brake
        vehicle_control.hand_brake = data.hand_brake
        vehicle_control.reverse = data.reverse
        vehicle_control.manual_gear_shift = data.manual_gear_shift
        vehicle_control.gear = data.gear
        self.carla_actor.apply_control(vehicle_control)

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: the color used by a vehicle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 255.0
        color.g = 0.0
        color.b = 0.0
        return color

    def get_marker_pose(self):
        """
        Function to return the pose for vehicles.

        :return: the pose of the vehicle
        :rtype: geometry_msgs.msg.Pose
        """
        # Moving pivot point from the bottom (CARLA) to the center (ROS) of the bounding box.
        extent = self.carla_actor.bounding_box.extent
        marker_transform = self.carla_actor.get_transform()
        marker_transform.location += marker_transform.get_up_vector() * extent.z
        return trans.carla_transform_to_ros_pose(marker_transform)

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return self.classification
