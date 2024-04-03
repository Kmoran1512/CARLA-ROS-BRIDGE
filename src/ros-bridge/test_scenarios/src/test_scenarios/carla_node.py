import carla

from rclpy.node import Node
from carla_msgs.msg import CarlaTrafficLightStatusList


class CarlaNode:
    def __init__(self, node: Node) -> None:
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

        node.create_subscription(
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

        if not lights_on:
            return
        all_actors = self.world.get_actors(lights_on)
        self.green_lights(all_actors)

    def green_lights(self, light_actors):
        for actor in light_actors:
            if actor.__class__ is not carla.libcarla.TrafficLight:
                continue
            actor.set_state(carla.libcarla.TrafficLightState.Green)
            actor.freeze(True)
