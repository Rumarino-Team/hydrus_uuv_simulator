FROM osrf/ros:melodic-desktop-full

# Install dependencies
RUN apt update && apt install -y\
 ros-melodic-smach\
 ros-melodic-smach-ros\
 ros-melodic-executive-smach\
 ros-melodic-smach-viewer\
 vim\
 nano\
 python-rosdep\
 python-rosinstall\
 python-rosinstall-generator\
 python-wstool\
 build-essential\
 ros-melodic-catkin\
 python-catkin-tools\
 python-scipy

# Update dependencies
RUN rosdep update
RUN apt-get update && apt-get dist-upgrade -y

RUN useradd -ms /bin/bash uuv

USER uuv

# Create a Catkin Workspace
RUN mkdir -p /home/uuv/catkin_ws/src
WORKDIR /home/uuv/catkin_ws
RUN . /opt/ros/melodic/setup.sh && \
 catkin_make

# Download ROS packages
WORKDIR /home/uuv/catkin_ws/src
RUN git clone https://github.com/uuvsimulator/uuv_simulator.git
RUN git clone https://github.com/uuvsimulator/uuv_simulation_evaluation.git
RUN git clone https://github.com/uuvsimulator/rexrov2.git

# Fix issue with uuv_simulator_evaluation
RUN sed -i '/<buildtool_depend>catkin<\/buildtool_depend>/a <build_depend>uuv_simulation_evaluation<\/build_depend>' uuv_simulation_evaluation/uuv_smac_utils/package.xml


# Install packages and their dependencies
WORKDIR /home/uuv/catkin_ws
RUN . /opt/ros/melodic/setup.sh && \
 . /home/uuv/catkin_ws/devel/setup.sh && \
 catkin_make install

# Add setup.sh for catkin workspace to bashrc
RUN echo "source /home/uuv/catkin_ws/devel/setup.sh" >> ~/.bashrc
