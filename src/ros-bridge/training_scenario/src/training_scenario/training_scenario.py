import carla
import logging
import rclpy
import random
import time

from carla_ros_bridge.actor_factory import ActorFactory
from rclpy.node import Node


class TrainingScenario(Node):
    def __init__(self, client):
        super(TrainingScenario, self).__init__("TrainingScenario")

        self.declare_parameter('role_name', 'hero')

        self.client = client
        role_name = self.get_parameter('role_name')

        self.walkers = []
        self.w_id = []

        print(f"in {role_name.value}")

    def spawn_walkers(self, client, world, n=10):
        world = client.get_world()
        bps = world.get_blueprint_library().filter("walker.pedestrian.*")

        walkers_list = []
        all_id = []

        percentagePedestriansRunning = 0.0  # how many pedestrians will run
        percentagePedestriansCrossing = (
            0.9  # how many pedestrians will walk through the road
        )

        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)

        spawn_points = []
        for i in range(n):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if loc != None:
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        batch = []
        walker_speed = []

        for spawn_point in spawn_points:
            bp = random.choice(bps)

            bp.set_attribute("is_invincible", "false")

            # Adjust for running here
            walker_speed.append(bp.get_attribute("speed").recommended_values[1])

            batch.append(carla.command.SpawnActor(bp, spawn_point))

        results = client.apply_batch_sync(batch, True)

        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                self.logger.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2

        batch = []
        walker_controller_bp = world.get_blueprint_library().find("controller.ai.walker")
        for i in range(len(walkers_list)):
            batch.append(
                carla.command.SpawnActor(
                    walker_controller_bp, carla.Transform(), walkers_list[i]["id"]
                )
            )
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                self.logger.error(results[i].error)
            else:
                walkers_list[i]["controller"] = results[i].actor_id

        for walker in walkers_list:
            all_id.append(walker["controller"])
            all_id.append(walker["id"])
        all_actors = world.get_actors(all_id)

        world.tick()

        for i in range(0, len(all_id), 2):
            all_actors[i].start()
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))

        self.logger.info("spawned %d walkers, press Ctrl+C to exit." % (len(walkers_list)))

        self.walkers = all_actors
        self.w_id = all_id

    def clean_up(self, client):
        for i in range(0, len(self.w_id), 2):
            self.walkers[i].stop()

        print("\ndestroying %d walkers" % len(self.walkers))
        client.apply_batch([carla.command.DestroyActor(x) for x in self.w_id])



def main():
    carla_client = carla.Client('localhost', 2000)
    world = carla_client.get_world()

    rclpy.init()

    ts = TrainingScenario(world)
    ts.spawn_walkers(carla_client, world, 200)

    try:
        rclpy.spin(ts)
    except KeyboardInterrupt:
        pass

    ts.clean_up(carla_client)
    ts.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
