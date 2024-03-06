from typing import List
import rclpy

from carla_msgs.srv import SpawnObject
from rclpy.node import Node
from rclpy.task import Future

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
        self.declare_parameter("scenario_number", 3)

        self.scenario_number = int(self.get_parameter("scenario_number").value)

    def _init_pub_sub(self):
        self.spawn_actors_service = self.create_client(
            SpawnObject, "/carla/spawn_object"
        )

    def spawn_walkers(self):
        if not self.spawn_actors_service.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service spawn_actors not available after waiting")
            return

        requests: List[Future] = []
        ped_number = self.scenario_number % 50

        lanes = [RIGHT, CENTER, LEFT]

        for i, lane in enumerate(lanes):
            lane_status = (ped_number // (3 ** i)) % 3

            if lane_status == 1:
                requests.append(self.spawn_1(ped_number, lane))
            elif lane_status == 2:
                requests.extend(self.spawn_3(ped_number, lane))

        for future in requests:
            rclpy.spin_until_future_complete(self, future)
            walker_id = future.result().id
            if walker_id > 0:
                self.pedestrian_ids.append(walker_id)

    def spawn_3(self, ped_num, y) -> List[Future]:
        spawn_offsets = [(-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)]
        return [
            self.spawn_1(ped_num, y + y_offset, PEDESTRIAN + x_offset)
            for x_offset, y_offset in spawn_offsets
        ]

    def spawn_1(self, ped_num, y, x=PEDESTRIAN) -> Future:
        walker_request = SpawnObject.Request(
            type=f"walker.pedestrian.{ped_num:04}", id=f"walker{ped_num:04}"
        )
        walker_request.transform.position.x = x
        walker_request.transform.position.y = y
        walker_request.transform.position.z = 1.0
        return self.spawn_actors_service.call_async(walker_request)


def main():
    rclpy.init()

    ts = TestScenarios()
    ts.spawn_walkers()

    try:
        rclpy.spin(ts)
    except KeyboardInterrupt:
        pass

    ts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
