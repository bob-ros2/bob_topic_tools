#!/usr/bin/env python3
#
# Copyright 2023 BobRos
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
import re

import rclpy
import yaml
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String


class FilterNode(Node):
    """
    String topic filter node.

    Can filter based on black/white list file
    with the additional option to apply a list of substitutions to the output.
    """

    def __init__(self):
        super().__init__('filter')
        self.declare_parameters(
            namespace='',
            parameters=[

                ('white_filter',
                 os.getenv('FILTER_WHITE_FILTER', '').split(',')
                 if os.getenv('FILTER_WHITE_FILTER') else [''],
                 ParameterDescriptor(description=(
                     'String array with white list rules. '
                     'Can also be set via environment variable FILTER_WHITE_FILTER.'))),

                ('black_filter',
                 os.getenv('FILTER_BLACK_FILTER', '').split(',')
                 if os.getenv('FILTER_BLACK_FILTER') else [''],
                 ParameterDescriptor(description=(
                     'String array with blacklist rules. '
                     'Can also be set via environment variable FILTER_BLACK_FILTER.'))),

                ('white_list',
                 os.getenv('FILTER_WHITE_LIST', ''),
                 ParameterDescriptor(description=(
                     'White list file. This overrides parameter white_filter.\n'
                     'Format: Yaml file with a list of strings containing regex rules. '
                     'Can also be set via environment variable FILTER_WHITE_LIST.'))),

                ('black_list',
                 os.getenv('FILTER_BLACK_LIST', ''),
                 ParameterDescriptor(description=(
                     'Black list file. This overrides parameter black_filter.\n'
                     'Format: Yaml file with a list of strings containing regex rules. '
                     'Can also be set via environment variable FILTER_BLACK_LIST.'))),

                ('substitute',
                 os.getenv('FILTER_SUBSTITUTE', '').split(',')
                 if os.getenv('FILTER_SUBSTITUTE') else [''],
                 ParameterDescriptor(description=(
                     'Applies substitutions to the string message similar to python re.sub\n'
                     "Expects an array with pairs: ['pattern1','replace1', ...]. "
                     'Can also be set via environment variable FILTER_SUBSTITUTE.')))
            ])

        self.white_filter = self.get_parameter('white_filter',).value
        self.black_filter = self.get_parameter('black_filter').value
        self.white_list = self.get_parameter('white_list').value
        self.black_list = self.get_parameter('black_list').value
        self.substitute = self.get_parameter('substitute').value

        msg = "Parameter 'substitute' must have even length: ['pattern1', 'replace1', ...]"
        assert len(self.substitute) in [0, 1] or len(self.substitute) % 2 == 0, msg

        if self.white_list:
            self.white_filter = self.load_yaml(self.white_list)
        if self.black_list:
            self.black_filter = self.load_yaml(self.black_list)

        self.sub = self.create_subscription(
            String, 'topic_in', self.chat_input_callback, 1000)
        self.pub = self.create_publisher(
            String, 'topic_out', 1000)
        self.pub_rejected = self.create_publisher(
            String, 'topic_rejected', 1000)

        self.add_on_set_parameters_callback(
            self.parameter_callback)

    def transform(self, s):
        """Substitute a string similar to python re.sub."""
        if len(self.substitute) >= 2 and len(self.substitute) % 2 == 0:
            for a, b in zip(self.substitute[::2], self.substitute[1::2]):
                s = re.sub(a, b, s)
        return s

    def chat_input_callback(self, msg: String):
        """
        Handle incoming message callback.

        It depends from black and white list if the message will be filtered.
        Finally also substitutions are applied if configured.
        """
        is_white = False
        for f in self.white_filter:
            if f and re.search(f, msg.data):
                is_white = True
                break
        if is_white is True \
           or self.white_filter == [''] \
           or len(self.white_filter) < 1:
            for f in self.black_filter:
                if f and re.search(f, msg.data):
                    self.get_logger().debug('Skipping %s' % msg.data)
                    self.pub_rejected.publish(msg)
                    return
            msg.data = self.transform(msg.data)
            self.pub.publish(msg)
            return
        self.pub_rejected.publish(msg)

    def load_yaml(self, filename):
        """Load YAML file and return content. Returns [] if an error occurs."""
        try:
            with open(filename, 'r') as file:
                content = yaml.load(file, Loader=yaml.FullLoader)
                self.get_logger().info('Loaded items: %d' % len(content))
            return content
        except Exception as e:
            self.get_logger().error('Loading %s: %s' % (filename, str(e)))
            return []

    def parameter_callback(self, params):
        """Parameter callback used by Dynamic Reconfigure."""
        for param in params:
            if param.name == 'white_filter':
                self.white_filter = param.value
            elif param.name == 'black_filter':
                self.black_filter = param.value
            elif param.name == 'substitute':
                if len(param.value) not in [0, 1] and len(param.value) % 2 != 0:
                    self.get_logger().error("Parameter 'substitute' has invalid length")
                    return SetParametersResult(successful=False,
                                               reason='substitute length must be even')
                self.substitute = param.value
            elif param.name == 'black_list':
                if self.black_list != param.value:
                    self.get_logger().info('Reload %s' % param.value)
                    self.black_filter = self.load_yaml(param.value)
                self.black_list = param.value
            elif param.name == 'white_list':
                if self.white_list != param.value:
                    self.get_logger().info('Reload %s' % param.value)
                    self.white_filter = self.load_yaml(param.value)
                self.white_list = param.value
        return SetParametersResult(successful=True)


def main():
    rclpy.init(args=None)
    n = FilterNode()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
