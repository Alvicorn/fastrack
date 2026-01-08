# ROS2 Workspace
Workspace for `CMPT720` running **ROS2 Humble**
- Originally fork of https://github.com/athackst/vscode_ros2_workspace/tree/humble-nvidia

In this course, we use Docker together with VS Code Dev Containers to provide a consistent and reliable development environment for ROS 2. 
Robotics software is particularly sensitive to differences in system configuration, and this approach helps us avoid many common issues.

# Setup
Before proceeding, ensure the following are installed on your system:
* [Docker](https://docs.docker.com/engine/install/)
* [Visual Studio Code](https://code.visualstudio.com/)
* [VS Code Remote Containers extention](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

## Docker Post-Installation (Linux)

To run Docker without sudo and allow VS Code to connect to containers, follow the official Docker post-installation steps:

https://docs.docker.com/engine/install/linux-postinstall/

In summary, run the following commpands:
```
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```




# Usage