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
import string

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml


class FilterNode(Node):
    """String topic filter ROS node with pre-compiled regex performance."""

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
                     'White list file. This overrides parameter white_filter. '
                     'Format: Yaml file with list of strings. '
                     'Can also be set via environment variable FILTER_WHITE_LIST.'))),

                ('black_list',
                 os.getenv('FILTER_BLACK_LIST', ''),
                 ParameterDescriptor(description=(
                     'Black list file. This overrides parameter black_filter. '
                     'Format: Yaml file with list of strings. '
                     'Can also be set via environment variable FILTER_BLACK_LIST.'))),

                ('substitute',
                 os.getenv('FILTER_SUBSTITUTE', '').split(',')
                 if os.getenv('FILTER_SUBSTITUTE') else [''],
                 ParameterDescriptor(description=(
                     'Applies substitutions. Array with pairs: [pattern1, replace1, ...]. '
                     'Can also be set via environment variable FILTER_SUBSTITUTE.'))),

                ('trim_data',
                 os.getenv('FILTER_TRIM_DATA', 'false').lower() == 'true',
                 ParameterDescriptor(description=(
                     'If true, trims whitespace (or trim_chars) from the message. '
                     'Can also be set via environment variable FILTER_TRIM_DATA.'))),

                ('trim_chars',
                 os.getenv('FILTER_TRIM_CHARS', ''),
                 ParameterDescriptor(description=(
                     'Specific characters to trim if trim_data is true. '
                     'Defaults to standard whitespace if empty. '
                     'Can also be set via environment variable FILTER_TRIM_CHARS.'))),

                ('skip_empty',
                 os.getenv('FILTER_SKIP_EMPTY', 'false').lower() == 'true',
                 ParameterDescriptor(description=(
                     'If true, skips messages that are empty after processing. '
                     'Can also be set via environment variable FILTER_SKIP_EMPTY.')))
            ])

        # Current parameter states
        self.white_filter = self.get_parameter('white_filter').value
        self.black_filter = self.get_parameter('black_filter').value
        self.white_list = self.get_parameter('white_list').value
        self.black_list = self.get_parameter('black_list').value
        self.substitute = self.get_parameter('substitute').value
        self.trim_data = self.get_parameter('trim_data').value
        self.trim_chars = self.get_parameter('trim_chars').value
        self.skip_empty = self.get_parameter('skip_empty').value

        # Compiled regex patterns
        self.white_patterns = []
        self.black_patterns = []
        self.sub_patterns = []

        # Initial compilation
        self.compile_regexes()

        self.sub = self.create_subscription(
            String, 'topic_in', self.input_callback, 10)
        self.pub = self.create_publisher(
            String, 'topic_out', 10)
        self.pub_rejected = self.create_publisher(
            String, 'topic_rejected', 10)

        self.get_logger().debug('FilterNode initialized.')

        self.add_on_set_parameters_callback(self.on_parameter_change)

    def load_yaml(self, filename):
        """Load YAML file and return content list. Returns [] on error."""
        if not filename or not os.path.isfile(filename):
            return []
        try:
            with open(filename, 'r') as file:
                content = yaml.safe_load(file)
                if isinstance(content, list):
                    return [str(item) for item in content if item]
        except Exception as e:
            self.get_logger().error(f'Error loading {filename}: {e}')
        return []

    def compile_regexes(self):
        """Pre-compile all regex patterns for performance."""
        # 1. White List
        raw_white = self.load_yaml(self.white_list) if self.white_list else self.white_filter
        self.white_patterns = []
        for p in raw_white:
            if p:
                try:
                    self.white_patterns.append(re.compile(p))
                except re.error as e:
                    self.get_logger().error(f"Invalid white regex '{p}': {e}")

        # 2. Black List
        raw_black = self.load_yaml(self.black_list) if self.black_list else self.black_filter
        self.black_patterns = []
        for p in raw_black:
            if p:
                try:
                    self.black_patterns.append(re.compile(p))
                except re.error as e:
                    self.get_logger().error(f"Invalid black regex '{p}': {e}")

        # 3. Substitutions
        self.sub_patterns = []
        if self.substitute and len(self.substitute) >= 2:
            if len(self.substitute) % 2 != 0:
                self.get_logger().warn('Substitute parameter length must be even.')
            else:
                for i in range(0, len(self.substitute), 2):
                    pattern, replacement = self.substitute[i], self.substitute[i+1]
                    if pattern:
                        try:
                            self.sub_patterns.append((re.compile(pattern), replacement))
                        except re.error as e:
                            self.get_logger().error(f"Invalid sub regex '{pattern}': {e}")

    def on_parameter_change(self, params):
        """Handle dynamic parameter changes and trigger re-compilation."""
        recompile_needed = False
        for param in params:
            if param.name == 'white_filter':
                self.white_filter = param.value
                recompile_needed = True
            elif param.name == 'black_filter':
                self.black_filter = param.value
                recompile_needed = True
            elif param.name == 'white_list':
                self.white_list = param.value
                recompile_needed = True
            elif param.name == 'black_list':
                self.black_list = param.value
                recompile_needed = True
            elif param.name == 'substitute':
                if len(param.value) >= 2 and len(param.value) % 2 != 0:
                    return SetParametersResult(successful=False,
                                               reason='substitute length must be even')
                self.substitute = param.value
                recompile_needed = True
            elif param.name == 'trim_data':
                self.trim_data = bool(param.value)
            elif param.name == 'trim_chars':
                self.trim_chars = str(param.value)
            elif param.name == 'skip_empty':
                self.skip_empty = bool(param.value)

        if recompile_needed:
            self.compile_regexes()

        return SetParametersResult(successful=True)

    def input_callback(self, msg: String):
        """Process incoming messages through filters and substitutions."""
        original_data = msg.data

        # 1. White list check
        if self.white_patterns:
            if not any(p.search(msg.data) for p in self.white_patterns):
                self.pub_rejected.publish(msg)
                return

        # 2. Black list check
        if self.black_patterns:
            if any(p.search(msg.data) for p in self.black_patterns):
                self.pub_rejected.publish(msg)
                return

        # 3. Substitutions
        for pattern, replacement in self.sub_patterns:
            msg.data = pattern.sub(replacement, msg.data)

        # 4. Trimming
        if self.trim_data:
            if self.trim_chars:
                msg.data = msg.data.strip(string.whitespace + self.trim_chars)
            else:
                msg.data = msg.data.strip()

        # 5. Skip empty
        if self.skip_empty and not msg.data:
            self.get_logger().debug(f'MSG SKIP: "{repr(original_data)}"')
            self.pub_rejected.publish(msg)
            return

        self.pub.publish(msg)


def main():
    rclpy.init(args=None)
    node = FilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
