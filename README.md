# ROS Package [bob_topic_tools](https://github.com/bob-ros2/bob_topic_tools)
[![ROS 2 CI](https://github.com/bob-ros2/bob_topic_tools/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/bob-ros2/bob_topic_tools/actions/workflows/ros2_ci.yml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This ROS Package is part of Bob's NLP and LLM tools.

The contained ROS Nodes are a collection of std_msgs/String topic tools. It extends the already existing [`ROS package topic_tools`](https://github.com/ros-tooling/topic_tools) but more dedicated to NLP tasks.

## ROS Node Filter
String topic filter ROS node. It supports also dynamic parameter reconfigure during runtime.

### Parameters

| Name | Type | Description |
|---|---|---|
| `white_filter` | string array | String array with white list rules. Can also be set via environment variable `FILTER_WHITE_FILTER`. |
| `black_filter` | string array | String array with blacklist rules. Can also be set via environment variable `FILTER_BLACK_FILTER`. |
| `white_list` | string | White list file. This overrides parameter `white_filter`. Format: Yaml file with a list of strings containing regex rules. Can also be set via environment variable `FILTER_WHITE_LIST`. |
| `black_list` | string | Black list file. This overrides parameter `black_filter`. Format: Yaml file with a list of strings containing regex rules. Can also be set via environment variable `FILTER_BLACK_LIST`. |
| `substitute` | string array | Substitutions using regex. Expects pairs: `['pattern', 'replace', ...]`. Can also be set via environment variable `FILTER_SUBSTITUTE`. |
| `trim_data` | boolean | If `true`, trims whitespace (or `trim_chars`) from the message. Default `false`. Can also be set via environment variable `FILTER_TRIM_DATA`. |
| `trim_chars` | string | Characters to trim if `trim_data` is true. Defaults to standard whitespace if empty. Can also be set via environment variable `FILTER_TRIM_CHARS`. |
| `skip_empty` | boolean | If `true`, skips messages that are empty after processing. Default `false`. Can also be set via environment variable `FILTER_SKIP_EMPTY`. |

### Topics

| Name | Type | Description |
|---|---|---|
| `~topic_in` | `std_msgs/String` | Read input data from topic. |
| `~topic_out` | `std_msgs/String` | Publish filtered output. |
| `~topic_rejected` | `std_msgs/String` | Publish rejected output. |

## ROS Node Stream Filter
String Stream topic filter node. Can redirect text portion enclosured in start/end tags.

### Parameters

| Name | Type | Description |
|---|---|---|
| `buffer_size` | integer | Input buffer size, default: 65535. Can also be set via environment variable `STREAM_FILTER_BUFFER_SIZE`. |
| `end_tag` | string | End tag of the area to filter out, default `</think>`. Can also be set via environment variable `STREAM_FILTER_END_TAG`. |
| `start_tag` | string | Start tag of the area to sort out, default `<think>`. Can also be set via environment variable `STREAM_FILTER_START_TAG`. |
| `window_size` | integer | Rolling window size to detect the tags. This should be > then max tag length, default: 32. Can also be set via environment variable `STREAM_FILTER_WINDOW_SIZE`. |

### Topics

| Name | Type | Description |
|---|---|---|
| `~stream_in` | `std_msgs/String` | Read input string stream from topic. |
| `~stream_out` | `std_msgs/String` | Publish data outside of the enclosing tags. |
| `~stream_filtered` | `std_msgs/String` | Publish data within the enclosing tags. |

## ROS Node Aggregator
String Stream aggregator node.

### Parameters

| Name | Type | Description |
|---|---|---|
| `delimiters` | string array | List of delimiter strings where to stop aggregating. Supports multi-character delimiters like `. `. Default: `['.', ',', '!', '?', '\n']`. |
| `auto_flush` | integer | Auto-flush timer in milliseconds. Default 1500ms. |

### Topics

| Name | Type | Description |
|---|---|---|
| `~stream_in` | `std_msgs/String` | Read input string stream from topic. |
| `~stream_out` | `std_msgs/String` | Publish aggregated output. |

## ROS Node Terminal
Basic String topic IO terminal ROS Node with syntax highlighting.

### Dependencies
The required QT5 libraries should already exist if ROS is installed. If missing use below installation to get them.
```bash
sudo apt install python3-pyqt5
sudo apt install python3-pyqt5.qsci
```

### Usage
```bash
# run frameless terminal window using ROS parameter
ros2 run bob_topic_tools terminal --ros-args -p frameless:=true -p geometry:=[300,300,600,480]

# show window on another display
# display what will be received from socket
netcat -U /tmp/some.sock | ros2 run bob_topic_tools terminal --ros-args -p display:=1 -p geometry:=[300,300,600,480]
```

### Parameters

| Name | Type | Description |
|---|---|---|
| `display` | integer | Display where to show window. Can also be set via environment variable `TERMINAL_DISPLAY`. |
| `fontname` | string | Window fontname. Can also be set via environment variable `TERMINAL_FONTNAME`. |
| `fontsize` | integer | Window fontsize. Can also be set via environment variable `TERMINAL_FONTSIZE`. |
| `frameless` | boolean | Switch off window caption. Can also be set via environment variable `TERMINAL_FRAMELESS`. |
| `geometry` | integer array | Window geometry. `[x, y, width, height]`. Can also be set via environment variable `TERMINAL_GEOMETRY`. |
| `input` | boolean | Enables or disables the text input field. Can also be set via environment variable `TERMINAL_INPUT`. |
| `line_count` | integer | Maximum line count in the text area. 0 = unlimited. If exceeded, lines are removed from the top. Can also be set via environment variable `TERMINAL_LINE_COUNT`. |
| `margin` | integer array | Window inner margin. `[left, top, right, bottom]`. Can also be set via environment variable `TERMINAL_MARGIN`. |
| `opacity` | double | Window opacity. Can also be set via environment variable `TERMINAL_OPACITY`. |
| `stylesheet` | string | Stylesheet qss of PlainText area. Can also be set via environment variable `TERMINAL_STYLESHEET`. |
| `stylesheet_window` | string | Stylesheet qss of Window area. Can also be set via environment variable `TERMINAL_STYLESHEET_WINDOW`. |
| `title` | string | Title of window. Can also be set via environment variable `TERMINAL_TITLE`. |
| `lexer` | string | The lexer to be used: `QsciLexerMarkdown`, `QsciLexerJSON`, `QsciLexerPython`. Default: `QsciLexerMarkdown`. Can also be set via environment variable `TERMINAL_LEXER`. |
| `image_view` | boolean | Start with image view option. Default: `false`. Can also be set via environment variable `TERMINAL_IMAGE_VIEW`. |
| `color_paper` | string | Background color (named or HEX). Default: `black`. Can also be set via environment variable `TERMINAL_COLOR_PAPER`. |
| `color_text` | string | Text color (named or HEX). Default: `lightgray`. Can also be set via environment variable `TERMINAL_COLOR_TEXT`. |

### Topics

| Name | Type | Description |
|---|---|---|
| `~topic_in` | `std_msgs/String` | Read input data from topic. |
| `~topic_in_cr` | `std_msgs/String` | Read input data and append newline. |
| `~topic_out` | `std_msgs/String` | Publish text entered in terminal. |
| `~image` | `sensor_msgs/Image` | Optional image subscription. |

## ROS Node Drain
String topic forwarding node with drain behavior. Controls the flow of messages to the output topic at a specified frequency.

### Parameters

| Name | Type | Description |
|---|---|---|
| `frequency` | double | Frequency in seconds to forward incoming messages to output topic. Default `1.0`. |

### Topics

| Name | Type | Description |
|---|---|---|
| `~drain_in` | `std_msgs/String` | Input stream topic. |
| `~drain_out` | `std_msgs/String` | Buffered output stream topic. |