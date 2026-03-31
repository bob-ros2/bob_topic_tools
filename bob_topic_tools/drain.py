#!/usr/bin/env python3
#
# Copyright 2025 BobRos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

from collections import deque

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DrainNode(Node):
    """String topic forwarding node with drain behavior."""

    def __init__(self):
        super().__init__('drain')

        self.message_cache = deque()

        self.declare_parameter('frequency', 1.0,
                               ParameterDescriptor(description='Frequency in seconds.'))

        self.frequency = self.get_parameter('frequency').value
        self.timer = self.create_timer(self.frequency, self.timer_callback)

        self.sub = self.create_subscription(
            String, 'drain_in', self.input_callback, 10)
        self.pub = self.create_publisher(
            String, 'drain_out', 10)

        self.add_on_set_parameters_callback(self.on_parameter_change)

    def on_parameter_change(self, params):
        """Handle dynamic parameter changes."""
        for param in params:
            if param.name == 'frequency':
                self.frequency = param.value
                self.get_logger().info(f'Frequency updated to: {self.frequency}s')
                # Restart timer with new frequency
                self.timer.cancel()
                self.timer = self.create_timer(self.frequency, self.timer_callback)
        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Publish next message if available."""
        if self.message_cache:
            message = self.message_cache.popleft()
            msg = String()
            msg.data = message
            self.pub.publish(msg)

    def input_callback(self, msg: String):
        """Handle incoming message callback."""
        self.message_cache.append(msg.data)


def main():
    rclpy.init(args=None)
    n = DrainNode()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
