import random
import carla
import rclpy

from rclpy.node import Node

from carla_msgs.msg import (
    CarlaEgoVehicleInfo,
    CarlaActorList,
    CarlaTrafficLightStatusList,
    CarlaTrafficLightInfoList,
)
from derived_object_msgs.msg import ObjectArray


SYNC = False


class TrainingScenario(Node):
    def __init__(self):
        super(TrainingScenario, self).__init__("TrainingScenario")

        self.declare_parameter("pedestrian_number", "50")
        self.pedestrian_number = int(self.get_parameter("pedestrian_number").value)

        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

        self.log = self.get_logger()
        self.walker_actors = self.ids = []

        self.log.warn("hello this is a warning")

        self._traffic_light_info_subscriber = self.create_subscription(
            CarlaTrafficLightStatusList,
            "/carla/traffic_lights/status",
            self.traffic_light_status,
            10,
        )

    def traffic_light_status(self, traffic_light_info_msg):
        lights_on = []
        for tl_info in traffic_light_info_msg.traffic_lights:
            if tl_info.state == 3:
                continue
            lights_on.append(tl_info.id)

        self.log.info(f"lights on:  {len(lights_on)}")
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
# - SpawnActor service to create actors
# - Wait for the response
# - Attach a walker agent to the actor
# - Give the actor a goal position
# - Walk them around

# - Access the stop light actors
# - Disable & Freeze them
