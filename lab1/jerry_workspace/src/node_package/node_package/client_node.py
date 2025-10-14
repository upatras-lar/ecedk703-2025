import random
import rclpy
from rclpy.task import Future
from rclpy.node import Node

from interfaces_package.srv import DistanceFromObstacle

class ClientNode(Node):
    # A ROS2 Node with a Service Client for DistanceFromObstacle

    def __init__(self):
        super().__init__('client_node')

        self.service_client = self.create_client(
            #### ENTER CODE HERE ###########


            ################################
        )

        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"service {self.service_client.srv_name} not available, waiting ...")
        
        self.future : Future = None

        timer_period: float = 0.5
        self.timer = self.create_timer(
            timer_period_sec=timer_period,
            #### ENTER CODE HERE ###########


            ################################
        )
    
    def make_request(self):
        # Method that is periodically called by the timer

        request = DistanceFromObstacle.Request()
        
        # Generate a random obstacle from (-100, -100) to (100, 100)
        request.x = float(random.randint(-100, 100))
        request.y = float(random.randint(-100, 100))

        
        if self.future is not None and not self.future.done():
            self.future.cancel() # Cancel the future. The callback will be called with Future.result == None
            self.get_logger().info(f"Service Future canceled. The Node took too long to process the service call")
        
        self.future = self.service_client.call_async(request)
        self.future.add_done_callback(self.process_response)
    
    def process_response(self, future: Future):
        # Callback for the future, that will be called when it is done
        response = future.result()
        #### ENTER CODE HERE ###########
        # Print the servers response
        # if response is not None:
        #     ###################

        #     ###################
        # else:
        #     ###################

        #     ###################

def main(args=None):

    try:
        rclpy.init(args=args)

        client_node = ClientNode()

        rclpy.spin(client_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()