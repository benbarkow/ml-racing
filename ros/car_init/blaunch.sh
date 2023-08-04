#!/bin/bash

sudo docker build . -t ros1-melodic-f1tenth
sudo docker run -it -v /dev:/dev --net=host --pid=host --device-cgroup-rule='c 166:* rmw' --device-cgroup-rule='c 13:* rmw' ros1-melodic-f1tenth:latest
