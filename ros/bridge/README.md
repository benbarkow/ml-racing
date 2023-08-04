# ViconROS2dockerBridge

Because our Vicon system at the ASL@UzL is only ROS1, you can use this bridge to publish the Vicon data to ROS2 also.

## Installation
(1. Install docker and docker-compose on your system)

2. Clone this repository && cd into it

## Usage
```bash
docker compose up
```

To test it you may use: ```docker run -it --net=host -v /dev/shm:/dev/shm --pid=host ros:foxy-ros-core``` and run ```ros2 topic list```. Using the host network, pid-range and mounting the shared memory are all required for ROS2 Node discovery - so do not omit them!


Note: This brdige compose can be run both on a embedded device directly (e.g. your autonomous car), or on any other host in the TP_Link Wifi (e.g. your laptop). A possible downside of running it on the embedded device is the additional CPU usage, while a possible downside of running it on another machine in the network is the potential wifi connection lag or loss - so choose wisely where to run this. 

## Troubleshooting
If there is versioning issues, you can try to change the ROS(2) version in the docker-compose.yaml file. Also, you can try to change the ROS1 version in the Dockerfile.

## Details
Internally, it is just the regular ROS1 Vicon vrpn_client_ros node, which publishes the data to a ROS1 topic, in one docker container. Then, there is a second docker container, which runs the default ros1_bridge, which subscribes to the ROS1 topic and publishes the data to a ROS2 topic.