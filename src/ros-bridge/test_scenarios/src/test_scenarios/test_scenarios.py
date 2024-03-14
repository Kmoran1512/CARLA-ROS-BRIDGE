import json
import math
import os
import time
import rclpy

from ament_index_python import get_package_share_directory
from carla_msgs.msg import CarlaWalkerControl
from carla_msgs.srv import DestroyObject, SpawnObject
from derived_object_msgs.msg import ObjectArray, Object
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Pose, Quaternion
from pygame.locals import K_s, K_z
from std_msgs.msg import Int8
from rclpy.node import Node, Parameter
from rclpy.publisher import Publisher
from rclpy.task import Future
from transforms3d.euler import euler2quat
from typing import List


RIGHT = -191.0
CENTER = -195.2
LEFT = -198.7
PEDESTRIAN = 222.0
VEHICLE = 312.0


class TestScenarios(Node):
    def __init__(self):
        super(TestScenarios, self).__init__("TestScenarios")

        self.start = None
        self.actions: List[List[PedestrianAction]] = []

        self._init_params()
        self._init_pub_sub()

    def _init_params(self):
        self.declare_parameter("spawn_point", "")
        self.spawn_point = string_to_pose(self.get_parameter("spawn_point").value)

        self.declare_parameter("peds", "[0,1,0]")
        self.declare_parameter("bp", "1")
        self.declare_parameter("bps", "")
        self.declare_parameter("direction", "0.0")
        self.declare_parameter("directions", "")
        self.declare_parameter("speed", "0.0")
        self.declare_parameter("speeds", "")
        self.declare_parameter("tdelay", "0.0")
        self.declare_parameter("tdelays", "")
        self.declare_parameter("mdelay", "")
        self.declare_parameter("mdelays", "")
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
            self.peds = get_list(self.get_parameter("peds"))
            self.bp = int(self.get_parameter("bp").value)
            self.bps = get_list(self.get_parameter("bps"))

            self.direction = get_float(self.get_parameter("direction"))
            self.speed = get_float(self.get_parameter("speed"))
            self.tdelay = get_float(self.get_parameter("tdelay"))
            self.mdelay = get_float(self.get_parameter("mdelay"))

            self.dirs: List[float] = get_list(self.get_parameter("directions"))
            self.speeds: List[float] = get_list(self.get_parameter("speeds"))
            self.tdelays: List[float] = get_list(self.get_parameter("tdelays"))
            self.mdelays: List[float] = get_list(self.get_parameter("mdelays"))
        else:
            self._set_params_from_config_file()

        self.v_x = 0.0
        self.v_y = 0.0

        self.pedestrian_ids = [0] * sum(self.peds)
        self.ped_locs = [(0.0, 0.0)] * sum(self.peds)

    def _init_pub_sub(self):
        self.spawn_actors_service = self.create_client(
            SpawnObject, "/carla/spawn_object"
        )
        self.destroy_actors_service = self.create_client(
            DestroyObject, "/carla/destroy_object"
        )

        self.create_subscription(Int8, "/key_press", self._on_key_press, 10)
        self.create_subscription(ObjectArray, "/carla/objects", self._update_obj, 10)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )

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
        elif self.actions:
            return self.run_actions()

        for n, (p_x, _) in enumerate(self.ped_locs):
            mdelay = self.mdelays[n] if self.mdelays else self.mdelay
            tdelay = self.tdelays[n] if self.tdelays else self.tdelay
            direction = self.dirs[n] if self.dirs else self.direction

            distance = self.v_x - p_x  # TODO: This needs to be waypoint dist
            time_passed = time.time() - self.start

            if not mdelay and not tdelay:
                continue
            elif mdelay and abs(distance - mdelay) > 0.5:
                continue
            elif not mdelay and abs(time_passed - tdelay) > 0.1:
                continue

            msg = CarlaWalkerControl()
            msg.direction.x = math.cos(direction)
            msg.direction.y = math.sin(direction)
            msg.speed = self.speed
            self.control_publishers[n].publish(msg)

    def run_actions(self):
        for n, actions in enumerate(self.actions):
            if not actions or not actions[0].should_run((self.v_x, self.v_y)):
                continue

            msg = CarlaWalkerControl()
            msg.direction.x = actions[0].x_dir
            msg.direction.y = actions[0].y_dir
            msg.speed = actions[0].speed
            self.control_publishers[n].publish(msg)

            prev_action = actions.pop(0)
            if not actions:
                continue

            actions[0].set_location((prev_action.x_coord, prev_action.y_coord))
            actions[0].set_previous_time(time.time())

    def clean_up(self):
        for id in self.pedestrian_ids:
            self.destroy_actors_service.call_async(DestroyObject.Request(id=id))

    def spawn_pedestrians(self):
        if not self.spawn_actors_service.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service spawn_actors not available after waiting")
            return

        self.requests: List[Future] = []
        lanes = [LEFT, CENTER, RIGHT]

        n_spawned = 0
        for i, lane in enumerate(lanes):
            self.position_orchestrator(self.peds[i], lane, n_spawned)
            n_spawned += self.peds[i]

        for i, future in enumerate(self.requests):
            rclpy.spin_until_future_complete(self, future)
            walker_id = future.result().id
            if walker_id > 0:
                self.pedestrian_ids[i] = walker_id

    def position_orchestrator(self, num, lane, spawned):
        offsets = [(PEDESTRIAN + num // 3, lane)]

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
        a, b, c, d = euler2quat(math.radians(yaw) + math.pi, math.pi, 0)
        s = Pose(
            position=Point(x=x, y=y, z=1.0), orientation=Quaternion(x=a, y=b, z=c, w=d)
        )

        walker_request = SpawnObject.Request(
            type=f"walker.pedestrian.{ped_num:04}", id=f"walker{i:04}", transform=s
        )
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
        elif data.data == K_z:
            self.pose_pub.publish(self.spawn_point)
            self.clean_up()
            self.spawn_pedestrians()
            self.start = None

    def _update_obj(self, data: ObjectArray):
        for obj in data.objects:
            obj: Object
            x, y = obj.pose.position.x, -obj.pose.position.y

            if obj.classification == Object.CLASSIFICATION_CAR:
                self.v_x, self.v_y = x, y
            elif obj.classification == Object.CLASSIFICATION_PEDESTRIAN:
                i = self.pedestrian_ids.index(obj.id)
                self.ped_locs[i] = (x, y)

                if len(self.actions) > i and self.actions[i]:
                    self.actions[i][0].set_location((x, y))


class PedestrianAction:
    def __init__(self, action):
        self.speed = action.get("speed", 0.0)

        yaw = math.radians(action.get("yaw", 0.0))
        self.x_dir, self.y_dir = math.cos(yaw), math.sin(yaw)

        self.mdelay = action.get("mdelay", 0.0)
        self.tdelay = action.get("tdelay", 0.0)

    def set_previous_time(self, prev_time):
        self.start_time = prev_time

    def set_location(self, loc):
        self.x_coord, self.y_coord = loc

    def should_run(self, v_loc):
        return self._is_in_triggering_distance(v_loc) or self._is_in_triggering_time()

    def _is_in_triggering_distance(self, v_loc) -> bool:
        if not self.mdelay:
            return False

        v_x, _ = v_loc
        distance = v_x - self.x_coord  # TODO: This needs to be waypoint dist
        return distance < self.mdelay

    def _is_in_triggering_time(self) -> bool:
        if not self.tdelay:
            return False

        time_passed = time.time() - self.start_time
        return time_passed > self.tdelay


def get_float(param: Parameter):
    val = param.value

    try:
        return float(val)
    except:
        return None


def get_list(param: Parameter) -> list:
    val = param.value
    if type(val) is list:
        return val
    return eval(val) if not val == "" else []


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


def main():
    rclpy.init()

    ts = TestScenarios()
    ts.spawn_pedestrians()

    try:
        ts.create_timer(0.1, lambda _=None: ts.run_step())

        rclpy.spin(ts, ts.executor)
    except KeyboardInterrupt:
        pass

    ts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
