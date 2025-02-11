# ROS2 Humble bot

Schoolproject made by Stef & Gert-Jan

Idea: bot with lidar scanner will drive in a room to map the room as a 3D map

Instructions followed for setting up VM in VirtualBox:
1. Install VirtualBox: https://www.virtualbox.org/wiki/Downloads
2. Install VirtualBox Extension Pack https://www.virtualbox.org/wiki/Downloads
3. Download Ubuntu ISO 22.04.5 LTS https://ubuntu.com/download/desktop
4. Create new VirtualBox VM
5. Set RAM to 4096GB
6. Set CPU cores to max available
7. Turn on Windows Hypervisor Platform under Windows Features
8. Boot Ubuntu VM
9. Turn on Shared Clipboard: VM -> Devices -> Shared Clipboard -> Bidirectional
10. Install VirtualBox Guest Additions: VM -> Devices -> Insert Guest Additions CD image...

Instructions followed for WSL:
1. Install WSL (```wsl --install```)
2. Install Ubuntu-22.04 (```wsl --install --d Ubuntu-22.04```)
3. Setup user account: input UNIX username, input and repeat password

Install ROS2 Humble in Ubuntu. Instructions followed: <br>
https://docs.ros.org/en/humble/ <br>

## Setting up a ROS2 Humble workspace, and creating a package
Documentation used: https://docs.ros.org/en/eloquent/Tutorials/Creating-Your-First-ROS2-Package.html
1. Create a ROS2 Humble workspace: ```mkdir ros_project/src```
2. Go into ROS2 Humble workspace: ```cd ros_project/src```
3. Create a ROS2 package: ```ros2 pkg create --build-type ament_cmake ros_project```
