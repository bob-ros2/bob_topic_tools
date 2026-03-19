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

import os
import threading
import time

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ValveNode(Node):
    """String topic forwarding node with valve behavior."""

    def __init__(self):
        super().__init__('valve')

        self.running = True
        self.message_cache = []
        self.cache_lock = threading.Lock()

        self.declare_parameter('frequency',
                               float(os.getenv('VALVE_FREQUENCY', '1.0')),
                               ParameterDescriptor(description=(
                                   'Frequency in seconds to forward incoming messages to output. '
                                   'Can also be set via environment variable VALVE_FREQUENCY, '
                                   'default 1.0.')))

        self.sub = self.create_subscription(
            String, 'valve_in', self.input_callback, 1000)
        self.pub = self.create_publisher(
            String, 'valve_out', 1000)

        # Start a separate thread to handle publishing
        self.publish_thread = threading.Thread(
            target=self.publish_messages)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def publish_messages(self):
        """Publish messages from the cache at a controlled frequency."""
        while self.running:
            msg = None
            with self.cache_lock:
                if self.message_cache:
                    message = self.message_cache.pop(0)
                    msg = String()
                    msg.data = message
                    self.pub.publish(msg)
            if msg:
                time.sleep(
                    self.get_parameter('frequency').value)
            else:
                time.sleep(0.01)

    def input_callback(self, msg: String):
        """Handle incoming message callback."""
        with self.cache_lock:
            self.message_cache.append(msg.data)


def main():
    rclpy.init(args=None)
    n = ValveNode()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
