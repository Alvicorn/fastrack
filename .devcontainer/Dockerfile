FROM althack/ros2:humble-dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ` # system packages ` \
    build-essential \
    ca-certificates \
    libtinfo5 \
    locales \
    nano \
    tmux \
    wget \
    git \
    \
    ` # ROS2 core libraries` \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    \
    `# install ROS demo nodes` \
    ros-dev-tools \
    ros-humble-desktop \
    ros-humble-turtlesim \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Additional pip packages
COPY requirements.txt /tmp/requirements.txt
RUN pip install --upgrade pip \
    && pip install --no-cache-dir --timeout 3000 -r /tmp/requirements.txt

# Set up auto-source of workspace for ros user
ARG WORKSPACE=/workspaces/fastrack
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# Set env variables for Nav2
RUN echo "export TURTLEBOT3_MODEL=burger" >> /home/ros/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /home/ros/.bashrc
