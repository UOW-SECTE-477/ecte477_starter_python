#!/usr/bin/env python3
"""
    command_server_node.py

    A ROS node that sends commands based on the current state of the 
    USAR problem. 

    Listens to the start and stop commands on the /cmd topic. These can 
    be sent with:
    rostopic pub -1 /cmd std_msgs/String -- 'start'
    rostopic pub -1 /cmd std_msgs/String -- 'stop'

    Subscribed: /cmd
    Publishes:

    Created: 2020/02/04
    Author: Brendan Halloran
"""

import rclpy
from rclpy.node import Node

from .commands import Commands, RobotState
from std_msgs.msg import String

class command_server_node(Node):
    def __init__(self):
        super().__init__('command_server_node')
        self.state = RobotState.WAITING_TO_START

        self.subscriber_command = self.create_subscription(String, '/cmd', self.callback_command, 5)
        self.publisher_state = self.create_publisher(String, '/state', 1)

        # Publish the current state at 10Hz to make sure other nodes get the correct info
        self.timer = self.create_timer(0.1, self.loop)

    # Run at 10Hz
    def loop(self):
        state_msg = String()
        state_msg.data = self.state.value
        self.publisher_state.publish(state_msg)

    def callback_command(self, data):
        command = Commands(data.data)

        if command is Commands.START:
            self.state = RobotState.EXPLORING
        elif command is Commands.STOP:
            self.state = RobotState.PAUSED
    
def main(args=None):
    
    rclpy.init(args=args)
    print("Starting ROS Command Server Module")
    cs = command_server_node()
    rclpy.spin(cs)
    cs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()