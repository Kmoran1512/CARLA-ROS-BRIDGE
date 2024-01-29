import rclpy

from rclpy.node import Node

from carla_msgs.srv import SpawnObject, DestroyObject

class TrainingScenario(Node):
    def __init__(self):
        super(TrainingScenario, self).__init__("TrainingScenario")

        self.declare_parameter('role_name', 'hero')
        role_name = self.get_parameter('role_name')

        self.spawn_actors_service = self.create_client(SpawnObject, '/carla/spawn_object')
        self.destroy_object_service = self.create_client(DestroyObject, "/carla/destroy_object")

        self.walkers = []


    def spawn_walkers(self, n=1):
        walker_requests = []

        for i in range(n):
            walker_request = SpawnObject.Request()

            number = i % 44

            walker_request.type = f'walker.pedestrian.00{number:02}'
            walker_request.id = f'walker{i:02}'
            walker_request.random_pose = True

            walker_requests.append(self.spawn_actors_service.call_async(walker_request))

        for request in walker_requests:
            future = request
            rclpy.spin_until_future_complete(self, future)
            self.walkers.append(future.result().id)
        
        self.walkers = [walker for walker in self.walkers if walker > 0]
        self.get_logger().info(f"spawned {len(self.walkers)} walkers")

    def clean_up(self):
        responses = []

        for id in self.walkers:
            remove_req = DestroyObject.Request()
            remove_req.id = id
            responses.append(self.destroy_object_service.call_async(remove_req))


        for response in responses:
            rclpy.spin_until_future_complete(self, response)

        self.get_logger().info(f"removed {len(responses)} walkers")


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


if __name__ == '__main__':
    main()
