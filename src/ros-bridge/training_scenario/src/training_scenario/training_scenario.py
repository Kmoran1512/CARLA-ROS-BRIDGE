import random
import carla
import rclpy

from rclpy.node import Node

import carla_common.transforms as trans
from carla_msgs.srv import SpawnObject, DestroyObject
from carla_msgs.msg import CarlaTrafficLightStatusList
from geometry_msgs.msg import Pose


class TrainingScenario(Node):
    def __init__(self):
        super(TrainingScenario, self).__init__("TrainingScenario")

        self.declare_parameter("pedestrian_number", "1")
        self.pedestrian_number = int(self.get_parameter("pedestrian_number").value)

        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

        self.world.set_pedestrians_cross_factor(0.7)

        self.walkers = {}

        self.walker_agents = {}
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
        self.client.apply_batch(
            [carla.command.DestroyActor(x) for x in self.walker_agents.keys()]
        )

    def run_step(self):
        if not len(self.walker_agents) > 0:
            return

        self.world.wait_for_tick()

    def spawn_walkers(self):
        if not self.spawn_actors_service.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service spawn_actors not available after waiting")
            return

        spawn_points = self._get_spawn_location()
        walker_requests = []

        for i in range(self.pedestrian_number):
            walker_request = SpawnObject.Request()

            number = (i % 44) + 1

            walker_request.type = f"walker.pedestrian.{number:04}"
            walker_request.id = f"walker{i:04}"
            walker_request.transform = spawn_points[i]

            walker_requests.append(self.spawn_actors_service.call_async(walker_request))

        for i, request in enumerate(walker_requests):
            future = request
            rclpy.spin_until_future_complete(self, future)
            walker_id = future.result().id
            if walker_id > 0:
                self.walkers[walker_id] = self.world.get_actor(walker_id)

        self.get_logger().info(f"\n\n\n spawned {len(self.walkers)} walkers \n\n\n")
        self._walk()

    def _walk(self):
        walker_controller_bp = self.world.get_blueprint_library().find(
            "controller.ai.walker"
        )

        batch = []

        self.get_logger().info(f"length_walkers  ::  {len(self.walkers)}")
        for walker_id in self.walkers.keys():
            batch.append(
                carla.command.SpawnActor(
                    walker_controller_bp, carla.Transform(), walker_id
                )
            )

        results = self.client.apply_batch_sync(batch, True)

        for result in results:
            if result.error:
                self.get_logger().error(result.error)
            else:
                self.walker_agents[result.actor_id] = self.world.get_actor(
                    result.actor_id
                )

        for _, agent in self.walker_agents.items():
            agent.start()
            agent.go_to_location(self.world.get_random_location_from_navigation())
            agent.set_max_speed(2.0)

        self.get_logger().info(
            f"\n\n\n Agents spawned {len(self.walker_agents)} \n\n\n"
        )

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

    def _get_spawn_location(self):
        spawn_poses = []

        for _ in range(self.pedestrian_number):
            spawn_pose = Pose()

            spawn_point = self.world.get_random_location_from_navigation()
            spawn_pose.position = trans.carla_location_to_ros_point(spawn_point)

            spawn_poses.append(spawn_pose)

        return spawn_poses

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
        ts.create_timer(0.05, lambda timer_event=None: ts.run_step())

        rclpy.spin(ts, ts.executor)
    except KeyboardInterrupt:
        pass

    ts.clean_up()
    ts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# TODO:
# - ~~Get spawn points from the blueprints~~
# - ~~SpawnActor service to create actors~~
# - ~~Wait for the response~~
# - ~~Ensure the car doesn't **stop** for walkers on the sidewalk~~
# - Attach a walker agent to the actor
# - Give the actor a goal position
# - Walk them around
