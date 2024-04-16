import json
import os
import time
import rclpy

from ament_index_python import get_package_share_directory
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaWalkerControl
from carla_msgs.srv import DestroyObject, SpawnObject
from derived_object_msgs.msg import ObjectArray, Object
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from pygame.locals import K_s
from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.task import Future
from typing import List

from .carla_node import CarlaNode
from .pedestrian_actions import Pedestrian, PedestrianAction
from .pedestrian_spawn_helper import (
    calculate_position,
    create_request,
    map_control_publisher,
)


from diagnostic_msgs.msg import KeyValue

SPAWN_DISTANCE = 70


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
        self.requests: List[Future] = []

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
            map_control_publisher(self, n, p) for n, p in enumerate(self.pedestrians)
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
        if self.pedestrians[n].bp < 100:
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

        for i, ped in enumerate(self.pedestrians):
            s = ped.get_spawn(self.pedestrian_x, self.center, self.point_yaw)

            request = create_request(ped.bp, i, s)
            self.requests.append(self.spawn_actors_service.call_async(request))

    def _set_params_from_config_file(self):
        n = self.config.get("num", 0)

        self.pedestrians: List[Pedestrian] = [Pedestrian()] * n
        self.actions: List[List[PedestrianAction]] = [0] * n

        for i, p in enumerate(self.config.get("pedestrians")):
            spawn = p.get("spawn", None)
            if spawn is None:
                continue

            self.pedestrians[i] = Pedestrian(
                p.get("blueprint", 0), p.get("yaw", 0.0), spawn, lanes[spawn]
            )
            self.actions[i] = [
                PedestrianAction(SPAWN_DISTANCE, act) for act in p.get("actions", [])
            ]

            lanes[spawn] += 1

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

        self._spawn_obstacle(data.poses)

        for actions in self.actions:
            action = actions[0]
            action.set_dist(data.poses)

        self.spawn_pedestrians()

    def _spawn_obstacle(self, waypts):
        s: Pose = waypts[SPAWN_DISTANCE - 2].pose
        s.position.y += Pedestrian.OFFSETS["right"] - 1
        type_id = "static.prop.container"
        id = f"obstacle"

        request = SpawnObject.Request(type=type_id, id=id, transform=s)
        self.requests.append(self.spawn_actors_service.call_async(request))


lanes = {
    "center": 0,
    "right": 0,
    "left": 0,
    "far_right": 0,
    "far_left": 0,
    "near_left_margin": 0,
    "near_left_bike": 0,
    "far_left_margin": 0,
    "far_right_margin": 0,
    "near_right_margin": 0,
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
