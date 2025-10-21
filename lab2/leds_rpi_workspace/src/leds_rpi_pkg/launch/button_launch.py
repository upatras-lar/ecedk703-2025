from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
       Node(
            package = 'leds_rpi_pkg',
            executable = 'sub_led_node',
            name = 'sub_led_node',
            output = 'screen',
            emulate_tty = True,
        ),
        ### ENTER CODE #########


        ########################
    ])