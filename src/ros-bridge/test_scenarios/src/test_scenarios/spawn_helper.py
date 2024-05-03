import math
import random

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaWalkerControl
from carla_msgs.srv import SpawnObject
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose
from .pedestrian_actions import Pedestrian
from rclpy import node, publisher
from transforms3d.euler import euler2quat, quat2euler
from typing import Tuple


def build_pedestrian(location, n, kind, speed=16):
    LABELS = {
        "soldier": 1,
        "police officer": 30,
        "orphan": 12,
        "terrorist": 15,
        "rapist": 16,
        "pedophile": 26,
        "judge": 20,
        "billionaire": 23,
        "celebrity": 25,
    }
    RT = 2.5

    pedestrian = {
        "actions": [
            {"yaw": -95.0 if n == 0 else 90.0},
            {"speed": 0.0, "tdelay": RT - 0.2},
        ],
        "yaw": -90.0 if n == 0 else 90.0,
    }

    if location[0] == "r":
        pedestrian["spawn"] = "far_left_margin"
    else:
        pedestrian["spawn"] = "near_left_margin"

    if location == "rs":
        pedestrian["actions"][0]["speed"] = 2.4 if n == 0 else 2.2
    if location == "rt":
        pedestrian["actions"][0]["speed"] = 2.4 if n == 0 else 1.4
    elif location == "ls":
        pedestrian["actions"][0]["speed"] = 1.2 if n == 0 else 1.4
    elif location == "lt":
        pedestrian["actions"][0]["speed"] = 1.0

    if n == 0:
        pedestrian["spawn"] = "right"

    pedestrian["actions"].append(
        {"speed": 1.0, "yaw": -pedestrian["actions"][0]["yaw"], "tdelay": 0.2}
    )

    pedestrian["actions"][0]["mdelay"] = RT * speed
    pedestrian["blueprint"] = LABELS[kind]

    return pedestrian


def calculate_position(goal_pose: Pose) -> Tuple[float, float, float]:
    o = goal_pose.orientation
    _, _, yaw = quat2euler((o.x, o.y, o.z, o.w))

    x = goal_pose.position.x + 2 * math.cos(yaw)
    y = goal_pose.position.y + 2 * math.sin(yaw)

    return (x, y, yaw)


def create_request(n, i, s):
    type_id = f"walker.pedestrian.{n:04}"
    id = f"walker{i:04}"
    if n >= 100:
        type_id = "vehicle.bh.crossbike"
        id = f"bike{i:04}"

    request = SpawnObject.Request(type=type_id, id=id, transform=s)

    if n >= 100:
        request.attributes = [KeyValue(key="role_name", value=f"bike{i:04}")]
    else:
        request.attributes = [KeyValue(key="is_invincible", value="false")]

    return request


def get_pedestrian_labels(delta):
    value_arr = []
    if delta == 2:
        rand = random.choice([0, 2])
        value_arr = [rand, 2 - rand]
    elif delta == 0:
        rand = random.choice(range(2))
        value_arr = [rand] * 2
    else:
        rand = random.choice(range(2))
        if rand == 2:
            value_arr = [2, 1]
        elif rand == 0:
            value_arr = [0, 1]
        else:
            rand = random.choice([0, 2])
            value_arr = [1, rand]

    return [get_pedestrian(value) for value in value_arr]


def get_pedestrian(value):
    if value == 2:
        return get_high_value_pedestrian()
    elif value == 1:
        return get_mid_value_pedestrian()
    else:
        return get_low_value_pedestrian()


def get_high_value_pedestrian():
    return random.choice(["soldier", "police officer", "orphan"])


def get_low_value_pedestrian():
    return random.choice(["terrorist", "rapist", "pedophile"])


def get_mid_value_pedestrian():
    return random.choice(["judge", "billionaire", "celebrity"])


def map_control_publisher(node: node.Node, n, ped: Pedestrian) -> publisher.Publisher:
    msg_type = CarlaWalkerControl
    topic = f"/carla/walker{n:04}/walker_control_cmd"
    if ped.bp >= 100:
        msg_type = CarlaEgoVehicleControl
        topic = f"/carla/bike{n:04}/secondary_vehicle_control"

    return node.create_publisher(msg_type, topic, 10)


def spawn_obstacle(
    spawn_distance, waypts, offset=2, side="right", bp="container", yaw=0.0
):
    s: Pose = waypts[spawn_distance - offset].pose
    s.position.y += Pedestrian.OFFSETS[side] - 1
    s.orientation.x, s.orientation.y, s.orientation.z, s.orientation.w = euler2quat(
        math.radians(yaw), math.pi, 0
    )
    type_id = "static.prop." + bp
    id = f"obstacle"

    return SpawnObject.Request(type=type_id, id=id, transform=s)
