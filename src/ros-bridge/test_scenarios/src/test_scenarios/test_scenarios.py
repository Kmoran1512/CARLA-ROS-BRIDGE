from typing import List
import carla
import rclpy

from carla_msgs.srv import SpawnObject
from carla_msgs.msg import CarlaWeatherParameters
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
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
        self._init_carla()
        self._init_pub_sub()

        self.pedestrian_ids = []

    def _init_params(self):
        self.declare_parameter("sun_azimuth", "60.0")
        self.declare_parameter("sun_elevation", "2.0")
        self.declare_parameter("ego_side", "right")
        self.declare_parameter("scenario_number", 3)

        self._azimuth = float(self.get_parameter("sun_azimuth").value)
        self._altitude = float(self.get_parameter("sun_elevation").value)
        self._ego_side = self.get_parameter("ego_side").value
        self.scenario_number = int(self.get_parameter("scenario_number").value)

    def _init_carla(self):
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

    def _init_pub_sub(self):
        self.spawn_actors_service = self.create_client(
            SpawnObject, "/carla/spawn_object"
        )

        self.weather_publisher = self.create_publisher(
            CarlaWeatherParameters, "/carla/weather_control", 10
        )
        self.ego_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

    def clean_up(self):
        self.client.apply_batch(
            [carla.command.DestroyActor(x) for x in self.pedestrian_ids]
        )

    def set_weather(self):
        weather_msg = CarlaWeatherParameters()
        weather_msg.sun_azimuth_angle = self._azimuth
        weather_msg.sun_altitude_angle = self._altitude
        self.weather_publisher.publish(weather_msg)

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
    ts.set_weather()

    try:
        rclpy.spin(ts)
    except KeyboardInterrupt:
        pass

    ts.clean_up()
    ts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
