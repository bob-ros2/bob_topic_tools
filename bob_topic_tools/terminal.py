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
import fcntl
import os
import select
import signal
import sys

import cv2
from PyQt5 import QtCore, QtGui
from PyQt5.Qsci import QsciLexerJSON
from PyQt5.Qsci import QsciLexerMarkdown
from PyQt5.Qsci import QsciLexerPython
from PyQt5.Qsci import QsciScintilla
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QScrollArea
from PyQt5.QtWidgets import QSlider
from PyQt5.QtWidgets import QSplitter
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QApplication

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class RosNode(Node):
    """String topic IO terminal ROS Node with syntax highlighting."""

    def __init__(self):
        super().__init__('terminal')

        self.declare_parameters(
            namespace='',
            parameters=[

                ('title', os.getenv('TERMINAL_TITLE', 'Topic IO Terminal'),
                 ParameterDescriptor(
                     description=(
                         'Title of window. '
                         'Can also be set via environment variable TERMINAL_TITLE.'))),

                ('lexer', os.getenv('TERMINAL_LEXER', 'QsciLexerMarkdown'),
                 ParameterDescriptor(
                     description=(
                         'The lexer to be used. Possible: '
                         'QsciLexerMarkdown, QsciLexerJSON, QsciLexerPython. '
                         'Can also be set via environment variable TERMINAL_LEXER.'))),

                ('image_view', os.getenv('TERMINAL_IMAGE_VIEW', 'false').lower() == 'true',
                 ParameterDescriptor(
                     description=(
                         'Start with image view option. '
                         'Can also be set via environment variable TERMINAL_IMAGE_VIEW.'))),

                ('opacity', float(os.getenv('TERMINAL_OPACITY', '1.0')),
                 ParameterDescriptor(
                     description=(
                         'Window opacity. '
                         'Can also be set via environment variable TERMINAL_OPACITY.'))),

                ('frameless', os.getenv('TERMINAL_FRAMELESS', 'false').lower() == 'true',
                 ParameterDescriptor(
                     description=(
                         'Switch off window caption. '
                         'Can also be set via environment variable TERMINAL_FRAMELESS.'))),

                ('fontname', os.getenv('TERMINAL_FONTNAME', 'courier'),
                 ParameterDescriptor(
                     description=(
                         'Window fontname. '
                         'Can also be set via environment variable TERMINAL_FONTNAME.'))),

                ('fontsize', int(os.getenv('TERMINAL_FONTSIZE', '15')),
                 ParameterDescriptor(
                     description=(
                         'Window fontsize. '
                         'Can also be set via environment variable TERMINAL_FONTSIZE.'))),

                ('geometry', [int(x) for x in
                              os.getenv('TERMINAL_GEOMETRY', '0,0,1000,480').split(',')],
                 ParameterDescriptor(
                     description=(
                         'Window geometry. [x, y, with, height]. '
                         'Can also be set via environment variable TERMINAL_GEOMETRY.'))),

                ('display', int(os.getenv('TERMINAL_DISPLAY', '0')),
                 ParameterDescriptor(
                     description=(
                         'Display where to show window. '
                         'Can also be set via environment variable TERMINAL_DISPLAY.'))),

                ('margin', [int(x) for x in os.getenv('TERMINAL_MARGIN', '10,0,0,0').split(',')],
                 ParameterDescriptor(
                     description=(
                         'Main window inner margin. [left, top, right, bottom]. '
                         'Can also be set via environment variable TERMINAL_MARGIN.'))),

                ('input', os.getenv('TERMINAL_INPUT', 'true').lower() == 'true',
                 ParameterDescriptor(
                     description=(
                         'Enables or disables the text input field. '
                         'Can also be set via environment variable TERMINAL_INPUT.'))),

                ('stylesheet',
                 os.getenv('TERMINAL_STYLESHEET', 'background-color: black; color: #e9e9e9;'),
                 ParameterDescriptor(
                     description=(
                         'Stylesheet qss of PlainText area. '
                         'Can also be set via environment variable TERMINAL_STYLESHEET.'))),

                ('stylesheet_window',
                 os.getenv('TERMINAL_STYLESHEET_WINDOW', 'background-color: gray; color white;'),
                 ParameterDescriptor(
                     description=(
                         'Stylesheet qss of Window area. '
                         'Can also be set via env TERMINAL_STYLESHEET_WINDOW.'))),

                ('line_count', int(os.getenv('TERMINAL_LINE_COUNT', '0')),
                 ParameterDescriptor(
                     description=(
                         'Maximum line count in text area. 0 = unlimited. '
                         'Can also be set via env TERMINAL_LINE_COUNT.'))),

                ('color_paper', os.getenv('TERMINAL_COLOR_PAPER', 'black'),
                 ParameterDescriptor(
                     description=(
                         'Background color (name or HEX). '
                         'Can also be set via environment variable TERMINAL_COLOR_PAPER.'))),

                ('color_text', os.getenv('TERMINAL_COLOR_TEXT', 'lightgray'),
                 ParameterDescriptor(
                     description=(
                         'Text color (name or HEX). '
                         'Can also be set via environment variable TERMINAL_COLOR_TEXT.')))
            ])

        self.pub = self.create_publisher(
            String, 'topic_out', 10)
        self.sub1 = None
        self.sub2 = None
        self.sub_image = None

    def publish(self, s):
        """Publish String message on topic."""
        self.pub.publish(String(data=s))

    def create_input_subscriptions(self, callback, callback_cr, callback_image):
        """Create topic subscriptions."""
        self.sub1 = self.create_subscription(
            String, 'topic_in', callback, 1000)
        self.sub2 = self.create_subscription(
            String, 'topic_in_cr', callback_cr, 1000)
        self.sub_image = self.create_subscription(
            Image, 'image', callback_image, 1000)


class StdinThread(QtCore.QThread):
    """QThread class to handle stdin reading."""

    result = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = True

    def run(self):
        """Read stdin loop function."""
        flags = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)
        while self.running:
            rlist, w, e = select.select([sys.stdin], [], [])
            if rlist:
                try:
                    buffer = sys.stdin.buffer.read(128).decode()
                except IOError:
                    buffer = []
                if len(buffer):
                    self.result.emit(buffer)
            else:
                self.sleep(0.1)
        sys.stdin.close()

    def stop(self):
        """Mark thread to stop the loop."""
        self.running = False


class Window(QMainWindow):
    """Main Window with terminal."""

    def __init__(self, node: RosNode):
        QWidget.__init__(self)
        self.node = node
        self.worker = StdinThread()
        self.worker.result.connect(self.update_text)
        self.oldPosition = QtCore.QPoint(0, 0)

        # setup main window
        self.setWindowTitle(
            self.node.get_parameter('title').value)

        self.setStyleSheet(
            self.node.get_parameter('stylesheet_window').value)

        self.setWindowOpacity(
            self.node.get_parameter('opacity').value)

        if self.node.get_parameter('frameless').value:
            self.node.setWindowFlag(
                QtCore.Qt.FramelessWindowHint)

        geometry = self.node.get_parameter('geometry').value
        if len(geometry) > 3:
            self.setGeometry(
                geometry[0], geometry[1], geometry[2], geometry[3])

        # create font
        font = QtGui.QFont(
            self.node.get_parameter('fontname').value)
        font.setPointSize(
            self.node.get_parameter('fontsize').value)

        # create text area
        self.tarea = QsciScintilla()
        self.tarea.setFont(font)

        # Set lexer
        lexer_name = self.node.get_parameter('lexer').value
        if lexer_name == 'QsciLexerMarkdown':
            self.lexer = QsciLexerMarkdown()
        elif lexer_name == 'QsciLexerJSON':
            self.lexer = QsciLexerJSON()
        elif lexer_name == 'QsciLexerPython':
            self.lexer = QsciLexerPython()
        else:
            self.node.get_logger().warn(
                f'Unknown lexer: {lexer_name} Using default: QsciLexerMarkdown')
            self.lexer = QsciLexerMarkdown()

        self.color_paper = self.node.get_parameter('color_paper').value
        self.color_text = self.node.get_parameter('color_text').value

        self.set_tarea_colors()
        self.set_lexer_styles()
        self.tarea.setLexer(self.lexer)

        # Optionally, set some other settings for QScintilla
        self.tarea.SendScintilla(QsciScintilla.SCI_SETHSCROLLBAR, 0)
        self.tarea.SendScintilla(QsciScintilla.SCI_SETVSCROLLBAR, 0)
        self.tarea.setMarginLineNumbers(0, False)
        self.tarea.setWrapMode(QsciScintilla.SC_WRAP_WORD)
        self.tarea.setIndentationsUseTabs(False)
        self.tarea.setIndentationWidth(4)
        self.tarea.setAutoIndent(True)

        # set max line count
        if self.node.get_parameter('line_count').value > 0:
            self.tarea.setMax.setMaximumBlockCount(
                self.node.get_parameter('line_count').value)

        # create input text field with history functionality from QComboBox
        self.text = ''
        self.tedit = QComboBox()
        self.tedit.setEditable(True)
        self.tedit.setStyleSheet(
            self.node.get_parameter('stylesheet').value)
        self.tedit.editTextChanged.connect(self.input_changed)
        self.tedit.lineEdit().setMaxLength(512000)
        self.tedit.lineEdit().setStyleSheet(
            self.node.get_parameter('stylesheet').value)
        self.tedit.lineEdit().returnPressed.connect(self.input_enter)

        # put together layout
        layout = QGridLayout()
        margin = self.node.get_parameter('margin').value
        if len(margin) > 3:
            layout.setContentsMargins(
                margin[0], margin[1], margin[2], margin[3])
        layout.setSpacing(0)

        layout.addWidget(self.tarea)
        if self.node.get_parameter('input').value:
            layout.addWidget(self.tedit)
            self.tedit.setFocus()
        else:
            self.tarea.setFocus()

        # Create a splitter to divide the main window into two parts
        self.splitter = QSplitter(QtCore.Qt.Horizontal)

        # Add the editor and image viewer to the splitter
        leftWidget = QWidget()
        leftWidget.setLayout(layout)

        self.splitter.addWidget(leftWidget)

        if self.node.get_parameter('image_view').value:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.cv_image = None
            self.original_cv_image = None
            self.splitter.addWidget(
                self.create_image_widget())
        else:
            self.bridge = None

        # Set the splitter as the central widget
        self.setCentralWidget(self.splitter)

        if self.node.get_parameter('display').value >= 0:
            screen = QtGui.QGuiApplication.screens()[
                self.node.get_parameter('display').value]
            self.move(screen.geometry().x()+geometry[0],
                      screen.geometry().y()+geometry[1])

        self.node.create_input_subscriptions(
            self.input_callback,
            self.input_callback_cr,
            self.input_callback_image)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(30)

    def create_image_widget(self):
        """Create image widget with slider."""
        # Create a QScrollArea and QLabel for the image viewer
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.image_label = QLabel()
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.scroll_area.setWidget(self.image_label)

        # Create a slider to resize the image
        self.slider = QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(1)
        self.slider.setMaximum(1000)
        self.slider.setValue(100)
        self.slider.valueChanged.connect(self.resize_image)

        # Create a layout for the image viewer with the slider
        self.image_widget = QWidget()
        self.image_layout = QVBoxLayout()
        self.image_layout.addWidget(self.scroll_area)
        self.image_layout.addWidget(self.slider)
        self.image_widget.setLayout(self.image_layout)

        return self.image_widget

    def spin_once(self):
        """Spin periodically once the ROS node."""
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def start_read_stdin(self):
        """Start the worker to read the incomming stdin stream."""
        self.worker.start()

    def set_lexer_styles(self):
        """Set all styles background."""
        self.lexer.setDefaultPaper(QtGui.QColor(self.color_paper))
        self.lexer.setDefaultColor(QtGui.QColor(self.color_text))
        for style in range(0, 22):
            self.lexer.setPaper(QtGui.QColor(self.color_paper), style)

    def update_settings(self,
                        prefix: str = 'lexer.markdown',
                        read: bool = False,
                        write: bool = True):
        """Update settings with given options."""
        self.settings = QtCore.QSettings('BobRos', 'Terminal')
        if read:
            self.lexer.readSettings(self.settings, prefix)
        elif write:
            self.lexer.writeSettings(self.settings, prefix)

    def set_tarea_colors(self):
        """Set ext area colors."""
        self.tarea.setCaretForegroundColor(QtGui.QColor(self.color_text))
        self.tarea.setMarginsBackgroundColor(QtGui.QColor(self.color_paper))
        self.tarea.setWrapMode(QsciScintilla.SC_WRAP_WORD)
        self.tarea.setIndentationGuides(True)
        self.tarea.setIndentationsUseTabs(False)
        self.tarea.setTabWidth(4)

    def update_text(self, s: str):
        """Append given text in the PlainText area."""
        # handle not closing code block issue from QsciLexerMarkdown
        if s.strip(' ').startswith('```'):
            s = s.strip(' ')
        self.tarea.append(s)
        last_line = self.tarea.lines() - 1
        last_line_length = len(self.tarea.text(last_line))
        self.tarea.setCursorPosition(last_line, last_line_length)

    def update_image(self):
        """Update image view from original cv image."""
        if self.cv_image is not None:
            self.original_cv_image = self.cv_image
        elif self.original_cv_image is None:
            return

        self.resize_image(self.slider.value())

    def resize_image(self, value):
        """Resize image based on slider value."""
        if self.original_cv_image is not None:

            height, width = self.original_cv_image.shape[:2]
            new_width = int(width * (value / 100))
            new_height = int(height * (value / 100))

            scaled_image = cv2.resize(
                self.original_cv_image,
                (new_width, new_height),
                interpolation=cv2.INTER_AREA)
            h, w = scaled_image.shape[:2]

            # Convert to RGB format for Qt display based on encoding
            encoding = getattr(self, 'image_encoding', 'bgr8')
            if encoding in ['mono8', 'mono16', '8UC1', '16UC1', '32FC1']:
                # Grayscale - convert to RGB
                if len(scaled_image.shape) == 2:
                    display_image = cv2.cvtColor(scaled_image, cv2.COLOR_GRAY2RGB)
                else:
                    display_image = scaled_image
            elif encoding in ['rgb8', 'rgb16', '8UC3'] and 'rgb' in encoding.lower():
                # Already RGB
                display_image = scaled_image
            elif encoding in ['rgba8', 'rgba16', '8UC4']:
                # RGBA - convert to RGB
                display_image = cv2.cvtColor(scaled_image, cv2.COLOR_RGBA2RGB)
            elif encoding in ['bgra8', 'bgra16']:
                # BGRA - convert to RGB
                display_image = cv2.cvtColor(scaled_image, cv2.COLOR_BGRA2RGB)
            else:
                # Default: assume BGR (most common in OpenCV/ROS) - convert to RGB
                display_image = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2RGB)

            bytes_per_line = 3 * w
            q_img = QtGui.QImage(
                display_image.data, w, h, bytes_per_line,
                QtGui.QImage.Format_RGB888)

            pixmap = QtGui.QPixmap.fromImage(q_img)
            self.image_label.setPixmap(pixmap)

    def input_callback(self, msg: String):
        """Receive input text via ROS topic and update text."""
        self.update_text(msg.data)

    def input_callback_cr(self, msg: String):
        """Receive input text via ROS topic and update text with CR."""
        self.update_text(msg.data+"\n")

    def input_callback_image(self, msg: Image):
        """Receive input image via ROS topic and update image view."""
        try:
            if self.bridge:
                self.image_encoding = msg.encoding
                self.cv_image = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='passthrough')
                self.update_image()
            else:
                self.node.get_logger().warning(
                    'input_callback_image: image view not enabled, '
                    'see ROS params.', once=True)
        except Exception as e:
            self.node.get_logger().info(
                f"input_callback_image: failed to convert image: {e}")

    def input_changed(self, text):
        """Input edit change text callback."""
        self.text = text

    def input_enter(self):
        """Input edit enter callback."""
        if self.text:
            self.node.publish(self.text)
        self.tedit.lineEdit().setText('')
        self.text = ''

    def closeEvent(self, event):
        """Window close callback. This stops also the stdin worker thread."""
        self.worker.stop()
        event.accept()

    def mousePressEvent(self, event):
        """Mouse press callback. Handles also window moving without caption."""
        self.oldPosition = event.globalPos()

    def mouseMoveEvent(self, event):
        """Mouse move callback. Handles also window moving without caption."""
        delta = QtCore.QPoint(event.globalPos() - self.oldPosition)
        self.move(self.x() + delta.x(), self.y() + delta.y())
        self.oldPosition = event.globalPos()


def sigint_handler(*args):
    """Handle SIGINT signal."""
    QApplication.quit()


def main():
    """Run the main ROS 2 node."""
    signal.signal(signal.SIGINT, sigint_handler)
    app = QApplication(sys.argv)
    rclpy.init(args=sys.argv)

    n = RosNode()
    screen = Window(n)

    if not os.isatty(sys.stdin.fileno()):
        screen.start_read_stdin()

    screen.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
