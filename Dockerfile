FROM ghcr.io/rezenders/knowrob_docker:main

COPY package.xml /knowrob_ws/src/metacontrol_knowrob/package.xml

WORKDIR /knowrob_ws
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash \
    && apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/"]

COPY CMakeLists.txt /knowrob_ws/src/metacontrol_knowrob/CMakeLists.txt
COPY setup.py /knowrob_ws/src/metacontrol_knowrob/setup.py
COPY launch/ /knowrob_ws/src/metacontrol_knowrob/launch/
COPY scripts/ /knowrob_ws/src/metacontrol_knowrob/scripts/
COPY src/ /knowrob_ws/src/metacontrol_knowrob/src/
COPY test/ /knowrob_ws/src/metacontrol_knowrob/test/
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && catkin build"]
