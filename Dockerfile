FROM ros:humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    ros-humble-moveit \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-controller-interface \
    ros-humble-joint-trajectory-controller \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \
    ros-humble-control-msgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-broadcaster \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers \
    ros-humble-effort-controllers \
    ros-humble-forward-command-controller \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace for the user's own ROS 2 packages
WORKDIR /ros2_ws/src

# Copy any packages in the repository into the workspace (optional)
COPY . /ros2_ws/src/

# Install dependencies using rosdep
WORKDIR /ros2_ws
RUN rosdep update && \
    rosdep install --ignore-src --from-paths src -y || true

# Source the workspace in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Open an interactive shell by default
CMD ["/bin/bash"] 