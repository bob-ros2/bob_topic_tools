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

class StringStreamBuffer:
    def __init__(self, max_length=0xFFFF):
        self.max_length = max_length
        self.buffer = bytearray()

    def append_text(self, text):
        """Appends text to the buffer, ensuring the buffer does not exceed max_length."""
        text_bytes = text.encode('utf-8')
        new_length = len(self.buffer) + len(text_bytes)
        
        # If appending the new text would exceed the buffer's maximum length, truncate
        if new_length > self.max_length:
            # Calculate how much of the new text can be appended
            max_additional_length = self.max_length - len(self.buffer)
            text_bytes = text_bytes[:max_additional_length]

        self.buffer.extend(text_bytes)

    def remove_keywords(self, keywords):
        """Removes all occurrences of keywords from the buffer."""
        keyword_set = set(keyword.encode('utf-8') for keyword in keywords)
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
        """Pops n characters from the buffer and returns them as a string. Default: 0xFFFF"""
        if n > len(self.buffer):
            n = len(self.buffer)

        # Get the first n bytes
        popped_bytes = self.buffer[:n]

        # Remove those bytes from the buffer
        self.buffer = self.buffer[n:]

        # Decode the bytes to a string and return it
        return popped_bytes.decode('utf-8', errors='replace')


class StreamProcessor:
    def __init__(self, start_tag, end_tag, window_size=32, buffer_size=0xFFFF):
        self.ring_buffer = [' '] * window_size
        self.window_size = window_size
        self.buffer_index = 0
        self.start_tag = start_tag
        self.end_tag = end_tag
        self.start_tag_found = False
        self.end_tag_found = False
        self.current_channel = "A"
        self.buffer_content = ""
        self.stream_A = StringStreamBuffer(buffer_size)
        self.stream_B = StringStreamBuffer(buffer_size)

    def process_char(self, char):
        # Add character to the ring buffer
        self.ring_buffer[self.buffer_index] = char
        self.buffer_index = (self.buffer_index + 1) % self.window_size

        # Append character to the buffer content
        self.buffer_content += char

        # Ensure buffer content does not exceed window size
        if len(self.buffer_content) > self.window_size:
            self.buffer_content = self.buffer_content[-self.window_size:]

        # Check if start_tag is found in the buffer content
        if not self.start_tag_found and self.start_tag in self.buffer_content:
            self.start_tag_found = True
            self.current_channel = "B"
            # Clear the buffer content after forwarding to channel B
            self.buffer_content = ""
            self.forward_to_channel_A(char)
        # Check if end_tag is found in the buffer content
        elif self.start_tag_found and self.end_tag in self.buffer_content:
            self.end_tag_found = True
            self.start_tag_found = False
            self.current_channel = "A"
            # Clear the buffer content after forwarding to channel A
            self.buffer_content = ""
            self.forward_to_channel_B(char)
        # Forward character to the appropriate channel
        elif self.current_channel == "A":
            self.forward_to_channel_A(char)
        else:
            self.forward_to_channel_B(char)

    def process_string(self, string):
        for char in string: 
            self.process_char(char)

    def forward_to_channel_A(self, string):
        logging.debug(f"Channel A: {string}")
        self.stream_A.append_text(string)

    def forward_to_channel_B(self, string):
        logging.debug(f"Channel B: {string}")    
        self.stream_B.append_text(string)


class StreamFilterNode(Node):
    """String Stream topic filter node. Can redirect text portion enclosured in start/end tags.
    """

    def __init__(self):
        super().__init__('stream_filter')

        logging.basicConfig(
            level = (logging.DEBUG
                if self.get_logger().get_effective_level() \
                    == LoggingSeverity.DEBUG \
                else logging.INFO),
            format="[%(levelname)s] [%(asctime)s.] [%(name)s]: %(message)s",
            datefmt="%s")
        
        self.declare_parameter('start_tag', 
            os.getenv('STREAM_FILTER_START_TAG','<think>'),
            ParameterDescriptor(description=
            "Start tag of the area to sort out, default '<think>'"))
        self.declare_parameter('end_tag', 
            os.getenv('STREAM_FILTER_END_TAG','</think>'),
            ParameterDescriptor(description=
            "End tag of the area to filter out, default '</think>'"))
        self.declare_parameter('window_size', 
            os.getenv('STREAM_FILTER_WINDOW_SIZE', 32),
            ParameterDescriptor(description=
            "Scrolling window size to detect the tags. This should be > then max tag length, default: 64"))
        self.declare_parameter('buffer_size', 
            os.getenv('STREAM_FILTER_BUFFER_SIZE', 0xFFFF),
            ParameterDescriptor(description=
            "Input buffer size, default: 65565"))

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
        """Will be called for every incoming message."""
        logging.debug("got: {msg.data}")
        self.token_handler.process_string(msg.data)

        self.token_handler.stream_A.remove_keywords(
            [self.token_handler.start_tag, self.token_handler.end_tag])
        self.token_handler.stream_B.remove_keywords(
            [self.token_handler.start_tag, self.token_handler.end_tag])

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
