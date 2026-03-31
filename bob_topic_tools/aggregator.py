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


from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AggregatorNode(Node):
    """String Stream aggregator node."""

    def __init__(self):
        super().__init__('aggregator')

        self.sentence = ''
        self.last_text_time = self.get_clock().now()

        self.declare_parameter('delimiters',
                               ['.', ',', '!', '?', '\n'],
                               ParameterDescriptor(description='List of delimiter strings.'))

        self.declare_parameter('auto_flush', 1500,
                               ParameterDescriptor(description='Auto-flush (ms, 0 to disable).'))

        self.sub = self.create_subscription(
            String, 'stream_in', self.input_callback, 1000)
        self.pub = self.create_publisher(
            String, 'stream_out', 1000)

        self.flush_timer = self.create_timer(0.1, self.flush_callback)

    def handle_escape(self, values: list):
        """Handle escape characters for delimiters in a list."""
        return [v.replace('\\n', '\n').replace('\\t', '\t').replace('\\s', ' ') for v in values]

    def input_callback(self, msg: String):
        """Handle incoming message callback."""
        self.get_logger().debug(f'input_callback: got: {msg.data}')
        delims = self.handle_escape(
            self.get_parameter('delimiters').value)

        self.last_text_time = self.get_clock().now()

        for c in msg.data:
            self.sentence += c
            # Check if any delimiter matches the current end of sentence
            if any(self.sentence.endswith(d) for d in delims):
                self.get_logger().debug(
                    f"input_callback: triggered by delimiter match in: '{self.sentence}'")
                self.pub.publish(String(data=self.sentence))
                self.sentence = ''

    def flush_callback(self):
        """Check periodically if the auto_flush timeout has expired."""
        auto_flush = self.get_parameter('auto_flush').value
        if auto_flush <= 0 or not self.sentence:
            return

        now = self.get_clock().now()
        elapsed_ms = (now - self.last_text_time).nanoseconds / 1e6

        if elapsed_ms > auto_flush:
            self.get_logger().debug(f'flush_callback: flushing: {self.sentence}')
            self.pub.publish(String(data=self.sentence))
            self.sentence = ''


def main():
    rclpy.init(args=None)
    n = AggregatorNode()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
