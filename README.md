# ROS Package Bob Topic Tools
This ROS package is part of Bob's NLP and LLM tools

The contained ROS Nodes are a collection of std_msgs/String topic tools. It extends the already existing [`ROS package topic_tools`](https://github.com/ros-tooling/topic_tools) but more dedicated to NLP tasks.

## ROS Node Filter
String topic filter ROS node. It supports also dynamic parameter reconfigure during runtime.

### Parameter

> **Parameter name**: white_filter\
> **Type**: string array\
> **Description**: String array with white list rules.

> **Parameter name**: black_filter\
> **Type**: string array\
> **Description**: String array with blacklist rules.

> **Parameter name**: white_list\
> **Type**: string\
> **Description**: White list file. This overides parameter white_filter.\
> **Format**: Yaml file with a list of string containing regex rules.

> **Parameter name**: black_list\
> **Type**: string\
> **Description**: Black list file. This overides parameter black_filter.\
> **Format**: Yaml file with a list of strings containing regex rules.

> **Parameter name**: substitute\
> **Type**: string array\
> **Description**: Substitute regex for the string message similar to python re.sub\
> **Expects an array with two entries**: ['pattern','replace']

### Subscribed Topics

> ~chat (std_msgs/String)\
Read input data from topic.

### Published Topics

> ~chat_filtered (std_msgs/String)\
Publish filtered output.

> ~chat_rejected (std_msgs/String)\
Publish rejected output.

## ROS Node Stream Filter
String Stream topic filter node. Can redirect text portion enclosured in start/end tags.

### Parameter
> **Parameter name**: buffer_size\
> **Type**: integer\
> **Description**: Input buffer size, default: 65565

> **Parameter name**: end_tag\
> **Type**: string\
> **Description**: End tag of the area to filter out, default '</think>'

> **Parameter name**: start_tag\
> **Type**: string\
> **Description**: Start tag of the area to sort out, default '<think>'

> **Parameter name**: window_size\
> **Type**: integer\
> **Description**: Rolling window size to detect the tags. This should be > then max tag length, default: 64

### Subscribed Topics

> ~stream_in (std_msgs/String)\
Read input string stream from topic

### Published Topics

> ~stream_out (std_msgs/String)\
Publish data outside of the enclosing tags.

> ~stream_filtered (std_msgs/String)\
Publish data within the enclosing tags.


## ROS Node Aggregator
String Stream aggregator node.

### Parameter
> **Parameter name**: delimeters\
> **Type**: string\
> **Description**: Delimeter character list where to stop aggregating the input and publish the aggregated data to the ouput topic. This parameter can also be set via environment variable AGGREGATOR_DELIMETERS, default '.:,;!?-*\n\t'

### Subscribed Topics

> ~stream_in (std_msgs/String)\
Read input string stream from topic.

### Published Topics

> ~stream_out (std_msgs/String)\
Publish aggregated output.

## ROS Node Terminal
Basic String topic IO terminal ROS Node.

### Usage
```bash
# run frameless terminal window using ROS parameter
ros2 run bob_topic_tools terminal --ros-args -p frameless:=true -p geometry:=[300,300,600,480]

# show window on another display
# the node can also read from stdin source
# display what will be received from socket
netcat -U /tmp/some.sock | ros2 run bob_topic_tools terminal --ros-args -p display:=1 -p geometry:=[300,300,600,480]
```

### Parameter

> **Parameter name**: display\
> **Type**: integer\
> **Description**: Display where to show window.

> **Parameter name**: fontname\
> **Type**: string\
> **Description**: Window fontname.

> **Parameter name**: fontsize\
> **Type**: integer\
> **Description**: Window fontsize.

> **Parameter name**: frameless\
> **Type**: boolean\
> **Description**: Switch off window caption.

> **Parameter name**: geometry\
> **Type**: integer array\
> **Description**: Window geometry. [x, y, with, height]

> **Parameter name**: input\
> **Type**: boolean\
> **Description**: Enables or disables the text input field.

> **Parameter name**: line_count\
> **Type**: integer\
> **Description**: Maximum line count in the text area. 0 = unlimited\
> If the number exceeds the lines are removed from the top.

> **Parameter name**: margin\
> **Type**: integer array\
> **Description**: Window inner margin. [left, top, right, bottom]

> **Parameter name**: opacity\
> **Type**: double\
> **Description**: Window opacity.

> **Parameter name**: stylesheet\
> **Type**: string\
> **Description**: Stylesheet qss of PlainText area.

> **Parameter name**: stylesheet_window\
> **Type**: string\
> **Description**: Stylesheet qss of Window area.

> **Parameter name**: title\
> **Type**: string\
> **Description**: Title of window.

### Subscribed Topics

> ~topic_in (std_msgs/String)\
Read input data from topic in addition to be able to read data from stdin.

### Published Topics

> ~topic_out (std_msgs/String)\
Publish to output.