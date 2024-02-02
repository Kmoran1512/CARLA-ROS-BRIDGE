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
