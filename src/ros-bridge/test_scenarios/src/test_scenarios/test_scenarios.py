from typing import List
import rclpy
import re

from carla_msgs.srv import SpawnObject
from rclpy.node import Node, Parameter
from rclpy.task import Future
from transforms3d.euler import euler2quat


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
        bp = int(self.get_parameter("bp").value)
        self.bps = get_list(self.get_parameter("bps")) or [bp]

        direction = get_float(self.get_parameter("direction"))
        speed = get_float(self.get_parameter("speed"))
        tdelay = get_float(self.get_parameter("tdelay"))
        mdelay = get_float(self.get_parameter("mdelay"))

        self.dirs = get_list(self.get_parameter("directions")) or [direction]
        self.speeds = get_list(self.get_parameter("speeds")) or [speed]
        self.tdelays = get_list(self.get_parameter("tdelays")) or [tdelay]
        self.mdelays = get_list(self.get_parameter("mdelays")) or [mdelay]

    def _init_pub_sub(self):
        self.spawn_actors_service = self.create_client(
            SpawnObject, "/carla/spawn_object"
        )

    def spawn_pedestrians(self):
        # if not self.spawn_actors_service.wait_for_service(timeout_sec=10.0):
        #     self.get_logger().error("Service spawn_actors not available after waiting")
        #     return

        self.requests: List[Future] = []
        lanes = [RIGHT, CENTER, LEFT]

        n_spawned = 0
        for i, lane in enumerate(lanes):
            print(self.peds)
            self.position_orchestrator(self.peds[i], lane, n_spawned)
            n_spawned += self.peds[i]

        for future in self.requests:
            rclpy.spin_until_future_complete(self, future)
            walker_id = future.result().id
            if walker_id > 0:
                self.pedestrian_ids.append(walker_id)

    def position_orchestrator(self, num, lane, spawned):
        for n in range(num):
            y_offset = 0.0  # TODO: later
            x_offset = 0.0  # TODO: later

            self.spawn_call(
                self.bps[spawned + n], self.dirs[spawned + n], lane + y_offset, x_offset
            )

    def spawn_call(self, ped_num, yaw, y, x=PEDESTRIAN):
        walker_request = SpawnObject.Request(
            type=f"walker.pedestrian.{ped_num:04}", id=f"walker{ped_num:04}"
        )
        walker_request.transform.position.x = x
        walker_request.transform.position.y = y
        walker_request.transform.position.z = 1.0

        a, b, c, d = euler2quat(0, 0, yaw)

        walker_request.transform.orientation.x = a
        walker_request.transform.orientation.y = b
        walker_request.transform.orientation.z = c
        walker_request.transform.orientation.w = d

        print(walker_request)
        #self.requests.append(self.spawn_actors_service.call_async(walker_request))


def get_float(param: Parameter):
    val = param.value

    try:
        return float(val)
    except:
        return None
    
def get_list(param: Parameter):
    val = param.value
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
