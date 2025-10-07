import rclpy
import random
from rclpy.node import Node
from interfaces_package.msg import JerryCommand

class CommandPublisher(Node):
    # A ROS2 node that publishes a JerryCommand

    def __init__(self):
        super().__init__('command_publisher_node')

        self.jerry_command_publisher = self.create_publisher(
           #### ENTER CODE HERE ###########


            ################################
        )

        timer_period: float = 2.0
        self.timer = self.create_timer(timer_period, self.publish_command)

        self.incremental_id: int = 0
    
    def publish_command(self):
        # Method that is periodically called by the timer

        jerry_command = JerryCommand()
        #### ENTER CODE HERE ###########


        ################################
        
        self.incremental_id += 1

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but instead by ROS2 to configure
    certain aspects of the Node.
    """

    try:
        rclpy.init(args=args)

        jerry_command_publisher_node = CommandPublisher()

        rclpy.spin(jerry_command_publisher_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()