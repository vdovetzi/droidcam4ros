# droidcam4ros

**ROS2 wrapper for the [DroidCam](https://www.dev47apps.com/droidcam/)**

This package integrates the DroidCam Linux client with ROS2, allowing you to use your Android/iPhone as a camera source in ROS2 applications. It wraps the DroidCam Linux library and provides a ROS2 publisher that publishes camera frames as ROS2 image messages.

## Dependencies

Make sure the following libraries are installed on your system:
- `glib-2.0`
- `libswscale`, `libavutil`
- `libturbojpeg`
- `libusbmuxd`
- `speex`
- `alsa`
- `pthread`
- `m`

On Ubuntu/Debian you can install most with:

```bash
sudo apt update
sudo apt install \
    build-essential cmake pkg-config \
    libglib2.0-dev libswscale-dev libavutil-dev \
    libturbojpeg0-dev libusbmuxd-dev \
    libspeex-dev libasound2-dev
```

## Build

Clone into your ROS2 workspace and build:
``` bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/droidcam4ros.git
cd ~/ros2_ws
colcon build --packages-select droidcam4ros
source install/setup.bash
```

## Usage

```bash
ros2 launch droidcam4ros droidcam-publisher.launch.py -s

Arguments (pass arguments as '<name>:=<value>'):
    'ip':
        DroidCam device`s WiFi IP

    'port':
        DroidCam device`s port
        (default: '4747')

    'output_topic':
        Ouput topic name for publisher
        (default: 'image_raw')
```

## Nodes

### [`droidcam-cli`](src/droidcam-cli.c)
- Connects to the DroidCam app (via WiFi/USB).
- Command-line client equivalent to `droidcam-cli` from [here](https://github.com/dev47apps/droidcam-linux-client)
### [`droidcam-publisher`](src/droidcam-publisher.cpp)
- Wraps the V4L2 device and publishes images to ROS2.
- Parameters:
    - device (int): V4L2 device number
    - output_topic (string): topic name for publishing images
## Launch Files
### [`droidcam-publisher.launch.py`](launch/droidcam-publisher.launch.py)
- The provided `droidcam-publisher.launch.py` runs both nodes:
  - `droidcam-cli` → starts the DroidCam client 
  - `droidcam-publisher` → reads from the device and publishes ROS images
