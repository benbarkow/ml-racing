FROM ros:kinetic-ros-core

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-vrpn-client-ros netbase
RUN rm -rf /var/lib/apt/lists/*

RUN ./ros_entrypoint.sh

# launch ros package
CMD ["roslaunch", "vrpn_client_ros", "sample.launch", "server:=192.168.0.100"]

