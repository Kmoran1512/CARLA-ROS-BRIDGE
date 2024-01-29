import math
import rclpy

from rclpy.node import Node

from carla_msgs.srv import SpawnObject, DestroyObject
from carla_msgs.msg import CarlaWalkerControl
from geometry_msgs.msg import Pose, Vector3


class TrainingScenario(Node):
    def __init__(self):
        super(TrainingScenario, self).__init__("TrainingScenario")

        self.declare_parameter("role_name", "hero")
        role_name = self.get_parameter("role_name")

        self.spawn_actors_service = self.create_client(
            SpawnObject, "/carla/spawn_object"
        )
        self.destroy_object_service = self.create_client(
            DestroyObject, "/carla/destroy_object"
        )

        self.walkers = []

    def spawn_walkers(self, n=1):
        walker_requests = []

        for i in range(1, n + 1):
            walker_request = SpawnObject.Request()

            number = i % 44

            walker_request.type = f"walker.pedestrian.{number:04}"
            walker_request.id = f"walker{i:04}"
            walker_request.random_pose = True

            walker_requests.append(self.spawn_actors_service.call_async(walker_request))

        for i, request in enumerate(walker_requests):
            future = request
            rclpy.spin_until_future_complete(self, future)
            if future.result().id > 0:
                self.walkers.append(WalkerAgent(i + 1, future.result().id))

        self.get_logger().info(f"spawned {len(self.walkers)} walkers")

    def clean_up(self):
        responses = []

        for walker in self.walkers:
            remove_req = DestroyObject.Request()
            remove_req.id = walker.id
            responses.append(self.destroy_object_service.call_async(remove_req))

        for response in responses:
            rclpy.spin_until_future_complete(self, response)

        self.get_logger().info(f"removed {len(responses)} walkers")


class WalkerAgent(Node):
    MIN_DISTANCE = 0.5

    def __init__(self, number, id):
        super().__init__(f"WalkerAgent{number:04}")

        self.id = id
        self.n = number

        self._waypoints = []
        self._pose = Vector3()

        self.walker_publisher = self.create_publisher(
            CarlaWalkerControl,
            "/carla/{}/walker_control_cmd".format(f"walker{self.n:04}"),
            10,
        )

    def run_step(self):
        # TODO: stay on sidewalk
        if not self._waypoints:
            return

        control = CarlaWalkerControl()
        direction = Vector3()
        direction.x = self._waypoints[-1].position.x - self._pose.position.x
        direction.y = self._waypoints[-1].position.y - self._pose.position.y
        direction_norm = math.sqrt(direction.x**2 + direction.y**2)
        if direction_norm > WalkerAgent.MIN_DISTANCE:
            control.speed = self._target_speed
            control.direction.x = direction.x / direction_norm
            control.direction.y = direction.y / direction_norm
        else:
            self._waypoints.pop()
            if self._waypoints:
                self.loginfo(
                    "next waypoint: {} {}".format(
                        self._waypoints[-1].position.x, self._waypoints[-1].position.y
                    )
                )
            else:
                self._reached_goal()

        self.control_publisher.publish(control)

    def get_position(self, objects_list):
        self._pose.x = 0
        self._pose.y = 0
        pass

    def _reached_goal(self):
        self.get_logger().info("Route finished.")
        pass


def main():
    rclpy.init()

    ts = TrainingScenario()
    ts.spawn_walkers(1)

    try:
        rclpy.spin(ts)
    except KeyboardInterrupt:
        pass

    ts.clean_up()
    ts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
