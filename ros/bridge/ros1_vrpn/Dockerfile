FROM ros:foxy-ros-core
SHELL ["/bin/bash", "-c"]

# install ros package
RUN apt-get update && \
      #apt remove -y ros-foxy-gmock-vendor ros-foxy-gtest-vendor && \
      apt-get install -y ros-foxy-vrpn netbase git python3-colcon-common-extensions && \
      rm -rf /var/lib/apt/lists/*

RUN apt-get update
RUN apt-get install build-essential -y
RUN apt-get install ros-foxy-tf2-tools -y
RUN apt-get install ros-foxy-tf2-kdl -y
RUN apt-get install ros-foxy-tf2-geometry-msgs -y

RUN mkdir ros2vrpnws && cd ros2vrpnws

RUN git clone https://github.com/toshi67026/vrpn_client_ros.git
WORKDIR vrpn_client_ros
RUN git checkout foxy_test
RUN sed -i 's/172.19.9.139/192.168.0.100/g' config/sample.params.yaml

RUN . /opt/ros/foxy/setup.sh && \
	colcon build
#RUN colcon build
#RUN source install/setup.bash
      
#RUN echo "source /vrpn_client_ros/install/setup.bash" >> /root/.bashrc
WORKDIR /
RUN ./ros_entrypoint.sh
WORKDIR vrpn_client_ros
RUN source install/setup.bash
# launch ros package
#ENTRYPOINT ["source", "install/setup.bash"]
RUN echo "source install/setup.bash && ros2 launch vrpn_client_ros sample.launch.py" > run.sh
RUN chmod +x run.sh
CMD ["./run.sh"]
#CMD ["ros2", "launch", "vrpn_client_ros", "sample.launch.py"]


