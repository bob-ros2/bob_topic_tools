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

import logging
import os

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import String


class StringStreamBuffer:
    """Buffer for streaming string data with a maximum length."""

    def __init__(self, max_length=0xFFFF):
        self.max_length = max_length
        self.buffer = bytearray()

    def append_text(self, text):
        """Append text to the buffer, ensuring the buffer does not exceed max_length."""
        text_bytes = text.encode('utf-8')
        new_length = len(self.buffer) + len(text_bytes)

        # If appending the new text would exceed the buffer's maximum length, truncate
        if new_length > self.max_length:
            # Calculate how much of the new text can be appended
            max_additional_length = self.max_length - len(self.buffer)
            text_bytes = text_bytes[:max_additional_length]

        self.buffer.extend(text_bytes)

    def remove_keywords(self, keywords):
        """Remove all occurrences of keywords from the buffer."""
        keyword_set = {keyword.encode('utf-8') for keyword in keywords}
        new_buffer = bytearray()
        i = 0

        while i < len(self.buffer):
            match = None
            for keyword in keyword_set:
                # Check if the keyword matches the text starting at the current position
                if self.buffer[i:i+len(keyword)] == keyword:
                    match = keyword
                    break
            if match:
                # Skip the keyword length
                i += len(match)
            else:
                # Copy the current byte to the new buffer
                new_buffer.append(self.buffer[i])
                i += 1

        self.buffer = new_buffer

    def pop(self, n=0xFFFF):
        """Pop n characters from the buffer and return them as a string. Default: 0xFFFF."""
        if n > len(self.buffer):
            n = len(self.buffer)

        # Get the first n bytes
        popped_bytes = self.buffer[:n]

        # Remove those bytes from the buffer
        self.buffer = self.buffer[n:]

        # Decode the bytes to a string and return it
        return popped_bytes.decode('utf-8', errors='replace')


class StreamProcessor:
    """Process a stream of text to split based on tags using efficient find()."""

    def __init__(self, start_tag, end_tag, window_size=32, buffer_size=0xFFFF):
        """Initialize the processor. window_size is kept for compatibility."""
        self.start_tag = start_tag
        self.end_tag = end_tag
        self.in_filtered_area = False
        self.stream_A = StringStreamBuffer(buffer_size)  # Outside tags
        self.stream_B = StringStreamBuffer(buffer_size)  # Inside tags
        self.carry_over = ''

    def process_string(self, text):
        """Process a string using block search instead of character loops."""
        combined = self.carry_over + text
        self.carry_over = ''

        while combined:
            if not self.in_filtered_area:
                # Looking for start_tag
                idx = combined.find(self.start_tag)
                if idx != -1:
                    # Found start_tag: everything before belongs to A
                    self.stream_A.append_text(combined[:idx])
                    self.in_filtered_area = True
                    combined = combined[idx + len(self.start_tag):]
                else:
                    # Not found, but partial tag might be at the end
                    # We keep (len(start_tag) - 1) chars as carry_over
                    safe_len = max(0, len(combined) - len(self.start_tag) + 1)
                    self.stream_A.append_text(combined[:safe_len])
                    self.carry_over = combined[safe_len:]
                    break
            else:
                # Looking for end_tag
                idx = combined.find(self.end_tag)
                if idx != -1:
                    # Found end_tag: everything before belongs to B
                    self.stream_B.append_text(combined[:idx])
                    self.in_filtered_area = False
                    combined = combined[idx + len(self.end_tag):]
                else:
                    # Not found, keep (len(end_tag) - 1) chars
                    safe_len = max(0, len(combined) - len(self.end_tag) + 1)
                    self.stream_B.append_text(combined[:safe_len])
                    self.carry_over = combined[safe_len:]
                    break


class StreamFilterNode(Node):
    """String Stream topic filter node. Redirects text portions in start/end tags."""

    def __init__(self):
        super().__init__('stream_filter')

        lvl = (logging.DEBUG
               if self.get_logger().get_effective_level() == LoggingSeverity.DEBUG
               else logging.INFO)
        logging.basicConfig(
            level=lvl,
            format='[%(levelname)s] [%(asctime)s.] [%(name)s]: %(message)s',
            datefmt='%s')

        self.declare_parameter('start_tag',
                               os.getenv('STREAM_FILTER_START_TAG', '<think>'),
                               ParameterDescriptor(description=(
                                   "Start tag of area to sort out, default '<think>'. "
                                   'Can also be set via environment variable '
                                   'STREAM_FILTER_START_TAG.')))
        self.declare_parameter('end_tag',
                               os.getenv('STREAM_FILTER_END_TAG', '</think>'),
                               ParameterDescriptor(description=(
                                   "End tag of area to filter out, default '</think>'. "
                                   'Can also be set via env STREAM_FILTER_END_TAG.')))
        self.declare_parameter('window_size',
                               int(os.getenv('STREAM_FILTER_WINDOW_SIZE', '32')),
                               ParameterDescriptor(description=(
                                   'Scrolling window size to detect tags. Should be > then '
                                   'max tag length, default: 32. '
                                   'Can also be set via env STREAM_FILTER_WINDOW_SIZE.')))
        self.declare_parameter('buffer_size',
                               int(os.getenv('STREAM_FILTER_BUFFER_SIZE', str(0xFFFF))),
                               ParameterDescriptor(description=(
                                   'Input buffer size, default: 65535. '
                                   'Can also be set via env STREAM_FILTER_BUFFER_SIZE.')))

        self.token_handler = StreamProcessor(
            self.get_parameter('start_tag',).value,
            self.get_parameter('end_tag',).value,
            self.get_parameter('window_size',).value,
            self.get_parameter('buffer_size',).value)

        self.sub = self.create_subscription(
            String, 'stream_in', self.input_callback, 1000)
        self.pub = self.create_publisher(
            String, 'stream_out', 1000)
        self.pub_filtered = self.create_publisher(
            String, 'stream_filtered', 1000)

    def input_callback(self, msg: String):
        """Handle incoming message callback."""
        logging.debug(f'got: {msg.data}')
        self.token_handler.process_string(msg.data)

        text = self.token_handler.stream_A.pop()
        if text:
            self.pub.publish(String(data=text))

        text = self.token_handler.stream_B.pop()
        if text:
            self.pub_filtered.publish(String(data=text))


def main():
    rclpy.init(args=None)
    n = StreamFilterNode()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
