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
import logging
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

class AggregatorNode(Node):
    """String Stream aggregator node.
    """

    def __init__(self):
        super().__init__('aggregator')

        logging.basicConfig(
            level = (logging.DEBUG
                if self.get_logger().get_effective_level() \
                    == LoggingSeverity.DEBUG \
                else logging.INFO),
            format="[%(levelname)s] [%(asctime)s.] [%(name)s]: %(message)s",
            datefmt="%s")

        self.sentence = ''

        self.declare_parameter('delimeters', 
            os.getenv('AGGREGATOR_DELIMETERS', '.:,;!?-*\n\t'),
            ParameterDescriptor(description=
            "Delimeter character list where to stop aggregating the input and "
            "publish the aggregated data to the ouput topic. This parameter "
            "can also be set via environment variable AGGREGATOR_DELIMETERS, "
            "default '.:,;!?-*\\n\\t'"))

        self.sub = self.create_subscription(
            String, 'stream_in', self.input_callback, 1000)
        self.pub = self.create_publisher(
            String, 'stream_out', 1000)

    def input_callback(self, msg: String):
        """Will be called for every incoming message."""
        logging.debug("got: {msg.data}")
        for c in msg.data:
            self.sentence += c
            if c in self.get_parameter('delimeters',).value:
                self.pub.publish(String(data=self.sentence))
                self.sentence = ''
        

def main():
    rclpy.init(args=None)
    n = AggregatorNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
