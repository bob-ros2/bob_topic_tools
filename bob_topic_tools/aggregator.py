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

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String


class AggregatorNode(Node):
    """String Stream aggregator node."""

    def __init__(self):
        super().__init__('aggregator')

        self.sentence = ''
        self.last_text_time = self.get_clock().now()

        self.declare_parameter('delimiters',
                               os.getenv('AGGREGATOR_DELIMITERS', '.:,;!?\n'),
                               ParameterDescriptor(description=(
                                   'Delimiter characters to stop aggregating the input and '
                                   'publish the aggregated data to the ouput topic. '
                                   'Real equivalents like \\n and \\t are supported. '
                                   'Can also be set via environment variable '
                                   "AGGREGATOR_DELIMITERS, default '.:,;!?\\n'")))

        self.declare_parameter('auto_flush',
                               int(os.getenv('AGGREGATOR_AUTO_FLUSH', '1500')),
                               ParameterDescriptor(description=(
                                   'Auto-flush timer in milliseconds. If the last token is not '
                                   'a delimiter, the text is published after this timeout. '
                                   'Set to 0 to disable. Default 1500ms. '
                                   'Can also be set via env AGGREGATOR_AUTO_FLUSH.')))

        self.sub = self.create_subscription(
            String, 'stream_in', self.input_callback, 1000)
        self.pub = self.create_publisher(
            String, 'stream_out', 1000)

        self.flush_timer = self.create_timer(0.1, self.flush_callback)

    def handle_escape(self, s: str):
        """Handle escape characters for specific delimiters."""
        return s.replace('\\n', '\n').replace('\\t', '\t')

    def input_callback(self, msg: String):
        """Handle incoming message callback."""
        self.get_logger().debug(f'input_callback: got: {msg.data}')
        delims = self.handle_escape(
            self.get_parameter('delimiters').value)

        self.last_text_time = self.get_clock().now()

        for c in msg.data:
            self.sentence += c
            if c in delims:
                self.get_logger().debug(
                    f"input_callback: char:{c} delims:'{delims}'")
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
