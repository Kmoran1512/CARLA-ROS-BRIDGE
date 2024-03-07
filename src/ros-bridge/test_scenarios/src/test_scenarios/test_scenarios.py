import math
import time
import rclpy

from carla_msgs.msg import CarlaWalkerControl
from carla_msgs.srv import SpawnObject
from rclpy.node import Node, Parameter
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

        self._init_params()
        self._init_pub_sub()

        self.pedestrian_ids = []

    def _init_params(self):
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

    def _init_pub_sub(self):
        self.spawn_actors_service = self.create_client(
            SpawnObject, "/carla/spawn_object"
        )

    def spawn_pedestrians(self):
        if not self.spawn_actors_service.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service spawn_actors not available after waiting")
            return

        self.requests: List[Future] = []
        lanes = [LEFT, CENTER, RIGHT]

        n_spawned = 0
        for i, lane in enumerate(lanes):
            print(self.peds[i])
            self.position_orchestrator(self.peds[i], lane, n_spawned)
            n_spawned += self.peds[i]

        for future in self.requests:
            rclpy.spin_until_future_complete(self, future)
            walker_id = future.result().id
            if walker_id > 0:
                self.pedestrian_ids.append(walker_id)

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
        walker_request = SpawnObject.Request(
            type=f"walker.pedestrian.{ped_num:04}", id=f"walker{i:04}"
        )
        walker_request.transform.position.x = x
        walker_request.transform.position.y = y
        walker_request.transform.position.z = 1.0

        a, b, c, d = euler2quat(math.radians(yaw) + math.pi, math.pi, 0)

        walker_request.transform.orientation.x = a
        walker_request.transform.orientation.y = b
        walker_request.transform.orientation.z = c
        walker_request.transform.orientation.w = d

        self.requests.append(self.spawn_actors_service.call_async(walker_request))


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


def main():
    rclpy.init()

    ts = TestScenarios()
    ts.spawn_pedestrians()

    try:
        rclpy.spin(ts)
    except KeyboardInterrupt:
        pass

    ts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
