FROM ghcr.io/rezenders/knowrob_docker:main

COPY . /knowrob_ws/src/metacontrol_knowrob
WORKDIR /knowrob_ws
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && catkin_make"]
