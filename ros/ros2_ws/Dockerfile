FROM ros:humble-ros-core
RUN apt-get update 
RUN apt-get install build-essential -y
RUN apt-get install -y python3-colcon-common-extensions
#RUN apt-get install -y python3-opencv
RUN apt-get install -y python3-rosdep
RUN apt-get install -y python3-pip
RUN apt-get install -y guvcview
RUN pip install setuptools==58.2.0
RUN rosdep init
RUN rosdep update

COPY ./install /ros2_ws/install
COPY ./src /ros2_ws/src
COPY ./log /ros2_ws/log
COPY ./build /ros2_ws/build
WORKDIR /ros2_ws

RUN apt-get update
RUN rosdep install -y --from-paths src
RUN colcon build --symlink-install

RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

