import math
import random
import carla
import rclpy

from rclpy.node import Node

from carla_msgs.srv import SpawnObject, DestroyObject
from carla_msgs.msg import CarlaWalkerControl
from geometry_msgs.msg import Vector3
from derived_object_msgs.msg import ObjectArray


class TrainingScenario(Node):
    def __init__(self):
        super(TrainingScenario, self).__init__("TrainingScenario")

        self.walkers = {}
        self.wf = self.get_waypoint_finder()

        self.spawn_actors_service = self.create_client(
            SpawnObject, "/carla/spawn_object"
        )
        self.destroy_object_service = self.create_client(
            DestroyObject, "/carla/destroy_object"
        )

        self.carla_vehicle_status_subscriber = self.create_subscription(
            ObjectArray, "/carla/objects", self._update_locations, 10
        )

    def run_step(self):
        for walker in self.walkers.values():
            walker.run_step()

    def get_waypoint_finder(self):
        client = carla.Client("localhost", 2000)
        world = client.get_world()
        map = world.get_map()

        waypoints = []

        for waypoint in map.generate_waypoints(distance=2.0):
            if waypoint.lane_type == carla.libcarla.LaneType.Sidewalk:
                waypoints.append(waypoint)

        return WaypointFinder(waypoints)

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
            walker_id = future.result().id
            if walker_id > 0:
                self.walkers[walker_id] = WalkerAgent(self, i + 1, walker_id, self.wf)

        self.get_logger().info(f"spawned {len(self.walkers)} walkers")

    def clean_up(self):
        responses = []

        for id in self.walkers.keys():
            remove_req = DestroyObject.Request()
            remove_req.id = id
            responses.append(self.destroy_object_service.call_async(remove_req))

        for response in responses:
            rclpy.spin_until_future_complete(self, response)

        self.get_logger().info(f"removed {len(responses)} walkers")

    def _update_locations(self, data):
        if not self.walkers:
            return

        for obj in data.objects:
            if not obj.classification == 4 and obj.id in self.walkers.keys():
                continue

            agent = self.walkers.get(obj.id)

            if agent:
                agent.set_position(obj.pose.position)


class WalkerAgent:
    MIN_DISTANCE = 0.5

    def __init__(self, node, number, id, wf):
        self._node = node

        self.id = id
        self.n = number
        self._target_speed = random.uniform(1, 2)

        self.wf = wf

        self._waypoint = Pose()
        self._pose = Pose()

        self.walker_publisher = self._node.create_publisher(
            CarlaWalkerControl,
            "/carla/{}/walker_control_cmd".format(f"walker{self.n:04}"),
            10,
        )

    def run_step(self):
        # TODO: stay on sidewalk
        if not self._waypoint:
            self._waypoint = self.wf.get_next_waypoint(self._pose)
            return

        self._node.get_logger().info("Walking.")

        control = CarlaWalkerControl()
        direction = Vector3()
        direction.x = self._waypoint.x - self._pose.x
        direction.y = self._waypoint.y - self._pose.y
        direction_norm = math.sqrt(direction.x**2 + direction.y**2)
        if direction_norm > WalkerAgent.MIN_DISTANCE:
            control.speed = self._target_speed
            control.direction.x = direction.x / direction_norm
            control.direction.y = direction.y / direction_norm
        else:
            self._reached_goal()

        self.walker_publisher.publish(control)

    def set_position(self, pose):
        self._pose.x = pose.x
        self._pose.y = pose.y

    def _reached_goal(self):
        self._node.get_logger().info("Route finished.")
        self._waypoint = self.wf.get_next_waypoint(self._pose)


class WaypointFinder:
    TOLERANCE = 1.0

    def __init__(self, waypoints) -> None:
        self._waypoints = waypoints

    def get_next_waypoint(self, current_pose):
        filtered_waypoints = [
            waypoint
            for waypoint in self._waypoints
            if abs(waypoint.transform.location.x - current_pose.x)
            < WaypointFinder.TOLERANCE
            or abs(waypoint.transform.location.y - current_pose.y)
            < WaypointFinder.TOLERANCE
        ]

        if filtered_waypoints:
            random_waypoint = random.choice(filtered_waypoints)
            clean_waypoint = Pose(
                random_waypoint.transform.location.x,
                random_waypoint.transform.location.y,
            )
            return clean_waypoint
        else:
            return None


class Pose:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


def main():
    rclpy.init()

    ts = TrainingScenario()
    ts.spawn_walkers(50)

    rate = ts.create_rate(2)

    try:
        while rclpy.ok():
            ts.run_step()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    ts.clean_up()
    ts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
