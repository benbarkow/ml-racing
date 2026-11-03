FROM ros:foxy-ros-core

# install ros package
RUN apt-get update && \
      #apt remove -y ros-foxy-gmock-vendor ros-foxy-gtest-vendor && \
      apt-get install -y ros-foxy-vrpn netbase git python3-colcon-common-extensions && \
      rm -rf /var/lib/apt/lists/*


RUN mkdir ros2vrpnws && cd ros2vrpnws

RUN git clone https://github.com/toshi67026/vrpn_client_ros.git && \
      cd vrpn_client_ros && \
      git checkout foxy_test && \
      sed -i 's/172.19.9.139/192.168.0.100/g' config/sample.params.yaml
RUN colcon build && \
      source install/setup.bash
      
RUN apt-get update
RUN apt-get install build-essentials -y
RUN apt-get install ros-foxy-tf2-tools -y
RUN apt-get install ros-foxy-tf2-kdl -y
RUN apt-get install ros-foxy-tf2-geometry-msgs -y

#RUN ./ros_entrypoint.sh

# launch ros package
CMD ["ros2", "launch", "vrpn_client_ros", "sample.launch.py"]

