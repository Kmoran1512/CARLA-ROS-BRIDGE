import json
import math
import os
import time
import rclpy

from ament_index_python import get_package_share_directory
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaWalkerControl
from carla_msgs.srv import DestroyObject, SpawnObject
from derived_object_msgs.msg import ObjectArray, Object
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Path
from pygame.locals import K_s
from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.task import Future
from transforms3d.euler import euler2quat
from typing import Dict, List, Tuple

from .carla_node import CarlaNode
from .pedestrian_actions import PedestrianAction
from .pedestrian_spawn_helper import (
    calculate_position,
    create_request,
    map_control_publisher,
)

SPAWN_DISTANCE = 7


class TestScenarios(Node):
    def __init__(self):
        super(TestScenarios, self).__init__("TestScenarios")

        self.start = None

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
            raise NotImplementedError("You must provide a config file")

        self._set_params_from_config_file()

        self.v_x, self.v_y = 0.0, 0.0
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

        self.control_publishers = [
            map_control_publisher(self, n, self.bps[n]) for n in range(sum(self.peds))
        ]

    def run_step(self):
        if self.start is None:
            return

        for n, actions in enumerate(self.actions):
            if not actions or not actions[0].should_run((self.v_x, self.v_y)):
                continue

            self.publish_action(actions[0], n)

            actions.pop(0)
            if not actions:
                continue

            actions[0].set_previous_time(time.time())

    def publish_action(self, action: PedestrianAction, n):
        msg = None
        if self.bps[n] < 100:
            msg = CarlaWalkerControl()
            msg.direction.x = action.x_dir
            msg.direction.y = action.y_dir
            msg.speed = action.speed
        else:
            msg = CarlaEgoVehicleControl()
            msg.throttle = 1.0 if abs(action.speed) > 0 else 0.0
            msg.brake = 1.0 if action.speed == 0 else 0.0
        self.control_publishers[n].publish(msg)

    def spawn_pedestrians(self):
        if not self.spawn_actors_service.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service spawn_actors not available after waiting")
            return

        self.requests: List[Future] = []

        n_spawned = 0
        for lane_name, lane in lanes.items():

            self._logger.info(f"name ::: {lane_name}")
            self._logger.info(f"lane ::: {lane}")
            self._logger.info(f"bp ::: {self.bps}")
            self._logger.info(f"yaws ::: {self.dirs}")


            self.position_orchestrator(
                lane["number"], self.center + lane["offset"], n_spawned
            )
            n_spawned += lane["number"]

    def position_orchestrator(self, num, lane, spawned):
        offsets = [(self.pedestrian_x + num // 3, lane)]

        for n in range(1, num):
            prev_x, prev_y = offsets[n - (((n - 1) % 3) + 1)]
            off_x = prev_x - (1 if n % 3 else 2.5)
            off_y = prev_y + (1 if n % 3 == 2 else -1 if n % 3 == 1 else 0)
            offsets.append((off_x, off_y))

        for n in range(num):
            total = spawned + n
            bp = self.bps[total]
            direction = self.dirs[total]

            x, y = offsets[n]
            self.spawn_call(total, bp, x, y, direction)

    def spawn_call(self, i, ped_num, x, y, yaw):
        a, b, c, d = euler2quat(math.radians(yaw) - self.point_yaw, math.pi, 0)
        s = Pose(
            position=Point(x=x, y=y, z=1.0), orientation=Quaternion(x=a, y=b, z=c, w=d)
        )


        request = create_request(ped_num, i, s)
        self.requests.append(self.spawn_actors_service.call_async(request))

    def _set_params_from_config_file(self):
        self.peds = [0] * len(lanes)
        n = self.config.get("num", 0)

        self.bps = [0] * n
        self.dirs = [0.0] * n
        self.actions: List[List[PedestrianAction]] = [0] * n


        for i, p in enumerate(self.config.get("pedestrians")):
            spawn = p.get("spawn", None)
            if spawn is None:
                continue

            lanes[spawn]["number"] += 1

            self.bps[i] = p.get("blueprint", 0)
            self.dirs[i] = p.get("yaw", 0.0)

            self.actions[i] =    [PedestrianAction(SPAWN_DISTANCE, act) for act in p.get("actions", [])]
            

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
            if not obj.classification == Object.CLASSIFICATION_CAR:
                continue

            self.v_x, self.v_y = obj.pose.position.x, obj.pose.position.y

    def _set_lanes(self, data: Path):
        if len(data.poses) < SPAWN_DISTANCE + 10:
            raise Exception("The path for the vehicle needs to be longer")

        pedestrian_pose: Pose = data.poses[SPAWN_DISTANCE].pose
        self.pedestrian_x, self.center, self.point_yaw = calculate_position(
            pedestrian_pose
        )

        for actions in self.actions:
            action = actions[0]
            action.set_dist(data.poses)

        self.spawn_pedestrians()


lanes = {
    "center": {"number": 0, "offset": 0.0},
    "right": {"number": 0, "offset": 4.2},
    "left": {"number": 0, "offset": -3.5},
    "far_right": {"number": 0, "offset": 3.5},
    "far_left": {"number": 0, "offset": -5.7},
    "near_left_margin": {"number": 0, "offset": -2.2},
    "far_left_margin": {"number": 0, "offset": -5.7},
}


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
