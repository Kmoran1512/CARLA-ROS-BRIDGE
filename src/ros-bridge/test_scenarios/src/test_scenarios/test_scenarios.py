import json
import math
import os
import time
import rclpy

from ament_index_python import get_package_share_directory
from carla_msgs.msg import CarlaWalkerControl
from carla_msgs.srv import DestroyObject, SpawnObject
from derived_object_msgs.msg import ObjectArray, Object
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
)
from nav_msgs.msg import Path
from pygame.locals import K_s
from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.task import Future
from transforms3d.euler import euler2quat, quat2euler
from typing import List

from .carla_node import CarlaNode

SPAWN_DISTANCE = 70


class TestScenarios(Node):
    def __init__(self):
        super(TestScenarios, self).__init__("TestScenarios")

        self.start = None
        self.actions: List[List[PedestrianAction]] = []

        self.carla = CarlaNode(self)

        self._init_params()
        self._init_pub_sub()

    def _init_params(self):
        self.declare_parameter("scenario_config", "")
        filename = self.get_parameter("scenario_config").value
        if filename:
            file_path = os.path.join(
                get_package_share_directory("test_scenarios"), "scenarios", filename
            )
            with open(file_path) as f:
                self.config = json.load(f)

        else:
            self.config = None

        if self.config is None:
            raise NotImplementedError("You must provide a config file")
        else:
            self._set_params_from_config_file()

        self.v_x = 0.0
        self.v_y = 0.0

        self.pedestrian_ids = [-1] * sum(self.peds)

    def _init_pub_sub(self):
        self.spawn_actors_service = self.create_client(
            SpawnObject, "/carla/spawn_object"
        )
        self.destroy_actors_service = self.create_client(
            DestroyObject, "/carla/destroy_object"
        )

        self.create_subscription(Int8, "/key_press", self._on_key_press, 10)
        self.create_subscription(
            Path, "/carla/ego_vehicle/waypoints", self._set_lanes, 10
        )
        self.create_subscription(ObjectArray, "/carla/objects", self._update_obj, 10)

        self.control_publishers: List[Publisher] = []
        for n in range(sum(self.peds)):
            self.control_publishers.append(
                self.create_publisher(
                    CarlaWalkerControl, f"/carla/walker{n:04}/walker_control_cmd", 10
                )
            )

    def run_step(self):
        if self.start is None:
            return

        for n, actions in enumerate(self.actions):
            if not actions or not actions[0].should_run((self.v_x, self.v_y), self):
                continue

            self.publish_action(actions[0], n)

            actions.pop(0)
            if not actions:
                continue

            actions[0].set_previous_time(time.time())

    def publish_action(self, action, n):
        msg = CarlaWalkerControl()
        msg.direction.x = action.x_dir
        msg.direction.y = action.y_dir
        msg.speed = action.speed
        self.control_publishers[n].publish(msg)

    def spawn_pedestrians(self):
        if not self.spawn_actors_service.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service spawn_actors not available after waiting")
            return

        self.requests: List[Future] = []
        lanes = [self.left, self.center, self.right]

        n_spawned = 0
        for i, lane in enumerate(lanes):
            self.position_orchestrator(self.peds[i], lane, n_spawned)
            n_spawned += self.peds[i]

    def position_orchestrator(self, num, lane, spawned):
        offsets = [(self.pedestrian_x + num // 3, lane)]

        for n in range(1, num):
            prev_x, prev_y = offsets[n - (((n - 1) % 3) + 1)]
            off_x = prev_x - (1 if n % 3 else 2.5)
            off_y = prev_y + (1 if n % 3 == 2 else -1 if n % 3 == 1 else 0)
            offsets.append((off_x, off_y))

        for n in range(num):
            total = spawned + n
            bp = self.bps[total] if self.bps else self.bp
            direction = self.dirs[total] if self.dirs else self.direction

            x, y = offsets[n]
            self.spawn_call(total, bp, x, y, direction)

    def spawn_call(self, i, ped_num, x, y, yaw):
        a, b, c, d = euler2quat(math.radians(yaw) - self.point_yaw, math.pi, 0)
        s = Pose(
            position=Point(x=x, y=y, z=1.0), orientation=Quaternion(x=a, y=b, z=c, w=d)
        )

        walker_request = SpawnObject.Request(
            type=f"walker.pedestrian.{ped_num:04}", id=f"walker{i:04}", transform=s
        )
        walker_request.attributes = [KeyValue(key="is_invincible", value="false")]
        self.requests.append(self.spawn_actors_service.call_async(walker_request))

    def _set_params_from_config_file(self):
        self.peds = [0, 0, 0]
        n = self.config.get("num", 0)

        self.bps = [0] * n
        self.dirs = [0.0] * n

        for i, p in enumerate(self.config.get("pedestrians")):
            if p.get("spawn") == "left":
                self.peds[0] += 1
            elif p.get("spawn") == "right":
                self.peds[2] += 1
            else:
                self.peds[1] += 1

            self.bps[i] = p.get("blueprint", 0)
            self.dirs[i] = p.get("yaw", 0.0)

            self.actions.append([PedestrianAction(act) for act in p.get("actions", [])])

    def _on_key_press(self, data: Int8):
        if self.start is None and data.data == K_s:
            self.get_logger().info("\n Playing Scenario \n")
            self.start = time.time()

            for actions in self.actions:
                if not actions:
                    continue

                actions[0].set_previous_time(self.start)

    def _update_obj(self, data: ObjectArray):
        for obj in data.objects:
            obj: Object
            if obj.classification == Object.CLASSIFICATION_CAR:
                self.v_x, self.v_y = obj.pose.position.x, obj.pose.position.y

    def _set_lanes(self, data: Path):
        if len(data.poses) < 90:
            raise Exception("The path for the vehicle needs to be longer")

        pedestrian_pose: Pose = data.poses[SPAWN_DISTANCE].pose
        self.pedestrian_x, self.center, self.point_yaw = calculate_position(
            pedestrian_pose
        )

        self.left = self.center - 3.5
        self.right = self.center + 4.2

        for actions in self.actions:
            action = actions[0]
            action.set_dist(data.poses)

        self.spawn_pedestrians()


class PedestrianAction:
    def __init__(self, action):
        self.speed = action.get("speed", 0.0)

        yaw = math.radians(action.get("yaw", 0.0))
        self.x_dir, self.y_dir = math.cos(yaw), math.sin(yaw)

        self.mdelay = action.get("mdelay", 0.0)
        self.tdelay = action.get("tdelay", 0.0)

    def set_previous_time(self, prev_time):
        self.start_time = prev_time

    def set_dist(self, waypoints: List[PoseStamped]):
        if not self.mdelay:
            return

        begin_loc = waypoints[SPAWN_DISTANCE - int(self.mdelay)].pose.position
        next_loc = waypoints[SPAWN_DISTANCE - int(self.mdelay) + 1].pose.position
        self.begin_x, self.begin_y = begin_loc.x, begin_loc.y
        self.next_x, self.next_y = next_loc.x, next_loc.y

    def should_run(self, v_loc, node):
        return (
            self._is_in_triggering_distance(v_loc, node)
            or self._is_in_triggering_time()
        )

    def _is_in_triggering_distance(self, v_loc, node: Node) -> bool:
        if not self.mdelay:
            return False

        v_x, v_y = v_loc

        dist_to_trigger = math.sqrt(
            (v_x - self.begin_x) ** 2 + (v_y - self.begin_y) ** 2
        )
        dist_after_trigger = math.sqrt(
            (v_x - self.next_x) ** 2 + (v_y - self.next_y) ** 2
        )

        return dist_to_trigger > dist_after_trigger

    def _is_in_triggering_time(self) -> bool:
        if not self.tdelay:
            return False

        time_passed = time.time() - self.start_time
        return time_passed > self.tdelay


def string_to_pose(input: str) -> PoseWithCovarianceStamped:
    p = list(map(float, input.split(",")))

    a, b, c, d = euler2quat(
        math.radians(p[3]) + math.pi, math.radians(p[4]), math.radians(p[5])
    )

    s = Pose(
        position=Point(x=p[0], y=p[1], z=p[2]),
        orientation=Quaternion(x=a, y=b, z=c, w=d),
    )

    out = PoseWithCovarianceStamped()
    out.pose.pose = s
    return out


def calculate_position(goal_pose: Pose):
    o = goal_pose.orientation
    _, _, yaw = quat2euler((o.x, o.y, o.z, o.w))

    x = goal_pose.position.x + 2 * math.cos(yaw)
    y = goal_pose.position.y + 2 * math.sin(yaw)

    return (x, y, yaw)


def main():
    rclpy.init()

    ts = TestScenarios()

    try:
        ts.create_timer(0.1, lambda _=None: ts.run_step())

        rclpy.spin(ts, ts.executor)
    except KeyboardInterrupt:
        pass

    ts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
