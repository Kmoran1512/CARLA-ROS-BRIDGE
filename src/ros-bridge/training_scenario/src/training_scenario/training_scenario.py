import random
import carla
import rclpy

from rclpy.node import Node

SYNC = False


class TrainingScenario(Node):
    def __init__(self):
        super(TrainingScenario, self).__init__("TrainingScenario")

        self.declare_parameter("pedestrian_number", "50")
        pedestrian_number = int(self.get_parameter("pedestrian_number").value)

        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

        self.log = self.get_logger()

        if SYNC:
            settings = self.world.get_settings()
            settings.synchronous_mode = SYNC
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)

        self._turn_off_lights()

        self.walkers = self.ids = []
        self._spawn_walkers(pedestrian_number)

    def _turn_off_lights(self):
        actors = self.world.get_actors()

        for actor in actors:
            if actor.__class__ is carla.libcarla.TrafficLight:
                actor.set_state(carla.libcarla.TrafficLightState.Off)
                actor.freeze(True)

    def _spawn_walkers(self, n=10):
        bps = self.world.get_blueprint_library().filter("walker.pedestrian.*")
        self.world.set_pedestrians_cross_factor(0.7)

        spawn_points = []
        walkers_speeds = []
        spawned_walkers = []

        for _ in range(n):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()

            spawn_point.location = loc
            spawn_points.append(spawn_point)

        batch = []
        for spawn_point in spawn_points:
            bp = random.choice(bps)
            bp.set_attribute("is_invincible", "false")

            # Adjust for running here
            walkers_speeds.append(bp.get_attribute("speed").recommended_values[1])

            batch.append(carla.command.SpawnActor(bp, spawn_point))

        results = self.client.apply_batch_sync(batch, True)

        batch = []
        matching_speeds = []
        for i in range(len(results)):
            if results[i].error:
                self.log.error(results[i].error)
            else:
                spawned_walkers.append({"id": results[i].actor_id})
                matching_speeds.append(walkers_speeds[i])
        walkers_speeds = matching_speeds

        walker_controller_bp = self.world.get_blueprint_library().find(
            "controller.ai.walker"
        )
        for walker in spawned_walkers:
            batch.append(
                carla.command.SpawnActor(
                    walker_controller_bp, carla.Transform(), walker["id"]
                )
            )
        results = self.client.apply_batch_sync(batch, True)

        for i in range(len(results)):
            if results[i].error:
                self.log.error(results[i].error)
                continue

            a_id = results[i].actor_id
            spawned_walkers[i]["controller"] = a_id

        all_ids = []
        for walker in spawned_walkers:
            all_ids.append(walker["controller"])
            all_ids.append(walker["id"])
        all_actors = self.world.get_actors(all_ids)

        if SYNC:
            self.world.tick()

        for i in range(0, len(all_ids), 2):
            all_actors[i].start()
            all_actors[i].go_to_location(
                self.world.get_random_location_from_navigation()
            )
            all_actors[i].set_max_speed(float(walkers_speeds[int(i / 2)]))

        self.log.info(
            "spawned %d walkers, press Ctrl+C to exit." % (len(spawned_walkers))
        )
        self.walkers = all_actors
        self.ids = all_ids

    def clean_up(self):
        if SYNC:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)

        for i in range(0, len(self.ids), 2):
            self.walkers[i].stop()

        self.log.info("\ndestroying %d walkers" % (len(self.walkers) / 2))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.ids])

    def run_step(self):
        if SYNC:
            self.world.tick()
        else:
            self.world.wait_for_tick()


def main():
    rclpy.init()

    ts = TrainingScenario()

    try:
        while True:
            ts.run_step()
    except KeyboardInterrupt:
        pass

    ts.clean_up()
    ts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
