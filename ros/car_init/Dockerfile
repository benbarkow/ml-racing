FROM dustynv/ros:melodic-ros-base-l4t-r32.7.1

RUN apt-get update && apt-get install ros-melodic-driver-base ros-melodic-tf libyaml-cpp-dev sed -y && rm -rf /var/lib/apt/lists/*

RUN git clone -b melodic https://github.com/f1tenth/f1tenth_system f1tenth_ws/src && cd f1tenth_ws && find . -name “*.py” -exec chmod +x {} \;

RUN sed -i 's/servo_min: 0.15/servo_min: 0.05/g' /f1tenth_ws/src/racecar/racecar/config/racecar-v2/vesc.yaml && sed -i 's/servo_max: 0.85/servo_max: 0.75/g' /f1tenth_ws/src/racecar/racecar/config/racecar-v2/vesc.yaml

RUN sed -i '8,15d' /f1tenth_ws/src/racecar/racecar/launch/includes/racecar-v2/vesc.launch.xml

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd f1tenth_ws; catkin_make; source devel/setup.bash'

RUN echo "source /f1tenth_ws/devel/setup.bash" >> /root/.bashrc
RUN export ROS_MASTER_URI=http://192.168.0.104:11311/

WORKDIR /f1tenth_ws

