import math

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaWalkerControl
from carla_msgs.srv import SpawnObject
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose
from .pedestrian_actions import Pedestrian
from rclpy import node, publisher
from transforms3d.euler import euler2quat, quat2euler
from typing import Tuple


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
