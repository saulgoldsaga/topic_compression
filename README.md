# Topic Compression

![CI](https://github.com/RaymondKirk/topic_compression/workflows/Topic%20Compression/badge.svg?branch=main)

Compress common ROS topic messages to save space or bandwidth.


## Installation

```bash
# From source
cd catkin_ws/src
git clone https://github.com/RaymondKirk/topic_compression 
catkin build topic_compression
```

## Usage 

Pass the topic to compress/decompress (in) and a output topic to publish on. 

```bash
# For example to compress
rosrun topic_compression run in:="/camera/depth/image_raw" out:="/camera/depth/compressed"

# To decompress it's the same syntax
rosrun topic_compression run in:="/camera/depth/compressed" out:="/camera/depth/image_raw_1"
```