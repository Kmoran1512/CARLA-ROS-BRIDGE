import random
import carla
import rclpy

from rclpy.node import Node

from carla_msgs.srv import SpawnObject, DestroyObject
from carla_msgs.msg import CarlaTrafficLightStatusList
from derived_object_msgs.msg import ObjectArray


SYNC = False


class TrainingScenario(Node):
    def __init__(self):
        super(TrainingScenario, self).__init__("TrainingScenario")

        self.declare_parameter("pedestrian_number", "1")
        self.pedestrian_number = int(self.get_parameter("pedestrian_number").value)

        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

        self.walkers = {}
        self.spawn_actors_service = self.create_client(
            SpawnObject, "/carla/spawn_object"
        )

        self._traffic_light_info_subscriber = self.create_subscription(
            CarlaTrafficLightStatusList,
            "/carla/traffic_lights/status",
            self.traffic_light_status,
            10,
        )

    def clean_up(self):
        # Perhaps Save this for walker agents
        pass

    def spawn_walkers(self):
        if not self.spawn_actors_service.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service spawn_actors not available after waiting')
            return


        walker_requests = []

        pedestrians_left_to_spawn = self.pedestrian_number + 1 - len(self.walkers)

        for i in range(1, pedestrians_left_to_spawn):
            walker_request = SpawnObject.Request()

            number = i % 44

            walker_request.type = f"walker.pedestrian.{number:04}"
            walker_request.id = f"walker{i:04}"
            # TODO: Set Pose Here
            walker_request.random_pose = True

            walker_requests.append(self.spawn_actors_service.call_async(walker_request))

        for i, request in enumerate(walker_requests):
            future = request
            rclpy.spin_until_future_complete(self, future)
            walker_id = future.result().id
            if walker_id > 0:
                self.walkers[walker_id] = self.world.get_actor(walker_id)

        self.get_logger().info(f"\n\n\n spawned {len(self.walkers)} walkers \n\n\n")

    def traffic_light_status(self, traffic_light_info_msg):
        lights_on = []
        for tl_info in traffic_light_info_msg.traffic_lights:
            if tl_info.state == 3:
                continue
            lights_on.append(tl_info.id)

        if not lights_on:
            return
        all_actors = self.world.get_actors(lights_on)
        self._turn_off_lights(all_actors)

    def _turn_off_lights(self, light_actors):
        for actor in light_actors:
            if actor.__class__ is not carla.libcarla.TrafficLight:
                continue
            actor.set_state(carla.libcarla.TrafficLightState.Off)
            actor.freeze(True)


def main():
    rclpy.init()

    ts = TrainingScenario()
    ts.spawn_walkers()

    try:
        rclpy.spin(ts)
    except KeyboardInterrupt:
        pass

    ts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# TODO:
# - Get spawn points from the blueprints
# - ~~SpawnActor service to create actors~~
# - ~~Wait for the response~~
# - Attach a walker agent to the actor
# - Give the actor a goal position
# - Walk them around
