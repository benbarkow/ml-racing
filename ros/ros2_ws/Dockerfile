FROM ros:humble-ros-core
RUN apt-get update 
#RUN apt-get install build-essential -y
RUN apt-get install -y python3-colcon-common-extensions
#RUN apt-get install -y python3-opencv
RUN apt-get install -y python3-rosdep
RUN apt-get install -y python3-pip
RUN apt-get install -y guvcview
RUN pip install onnx onnxruntime
RUN pip install setuptools==58.2.0
RUN pip install stable-baselines3
RUN rosdep init
RUN rosdep update

WORKDIR /ros2_ws
COPY ./install install
COPY ./src src
COPY ./log log

RUN apt-get update
RUN rosdep install -y --from-paths src

RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

