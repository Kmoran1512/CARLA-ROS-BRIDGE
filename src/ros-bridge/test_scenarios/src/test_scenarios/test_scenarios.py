import carla
import rclpy

from rclpy.node import Node
from carla_msgs.srv import SpawnObject
from carla_msgs.msg import CarlaWeatherParameters
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose


RIGHT = -195.5
LEFT = -192.5
PEDESTRIAN  = 222.0
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
        # TODO: Allow to run on different sides and all
        self.declare_parameter("ego_side", "right")
        self.declare_parameter("single_side", "right")
        self.declare_parameter("has_multi", "False")
        
        self._azimuth = float(self.get_parameter("sun_azimuth").value)
        self._altitude = float(self.get_parameter("sun_elevation").value)
        self._ego_side = self.get_parameter("ego_side").value
        self._single_side = self.get_parameter("single_side").value
        self._has_multi = bool(self.get_parameter("has_multi").value)

        self.scenario_number = 1

    def _init_carla(self):
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

    def _init_pub_sub(self):
        self.spawn_actors_service = self.create_client(
            SpawnObject, "/carla/spawn_object"
        )

        self.weather_publisher = self.create_publisher(
            CarlaWeatherParameters,
            "/carla/weather_control",
            10,
        )
        self.ego_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
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

        walker_requests = []
        if self._has_multi:
            for i in range(3):
                walker_request = SpawnObject.Request()

                walker_request.type = f"walker.pedestrian.{self.scenario_number:04}"
                walker_request.id = f"walker{self.scenario_number:04}"

                walker_pose = Pose()
                walker_pose.position.x = PEDESTRIAN
                walker_pose.position.y = RIGHT if self._single_side == 'left' else LEFT
                if i == 0:
                    walker_pose.position.x -= 1
                elif i == 1:
                    walker_pose.position.x += 1
                    walker_pose.position.y -= 1
                else:
                    walker_pose.position.x += 1
                    walker_pose.position.y += 1

                walker_request.transform = walker_pose
                walker_requests.append(self.spawn_actors_service.call_async(walker_request))

        walker_request = SpawnObject.Request()
        walker_request.type = f"walker.pedestrian.{self.scenario_number:04}"
        walker_request.id = f"walker{self.scenario_number:04}"
        walker_request.transform.position.x = PEDESTRIAN
        walker_request.transform.position.y = LEFT if self._single_side == 'left' else RIGHT
        walker_requests.append(self.spawn_actors_service.call_async(walker_request))


        for i, request in enumerate(walker_requests):
            future = request
            rclpy.spin_until_future_complete(self, future)
            walker_id = future.result().id
            if walker_id > 0:
                self.pedestrian_ids.append(walker_id)


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
