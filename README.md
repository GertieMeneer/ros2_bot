# ROS2 Humble bot

Schoolproject made by Stef & Gert-Jan

Idea: bot with lidar scanner will drive in a room while measuring the amount of light using a LDR. After the mapping, it will find the brightest spot in the room, and use LED's to show how close it is to that spot.

## Setting up local ROS2 Humble development environment
#### Instructions followed for setting up VM in VirtualBox:
1. Download and install [VirtualBox](https://www.virtualbox.org/wiki/Downloads)
2. Download and install [VirtualBox Extension Pack](https://www.virtualbox.org/wiki/Downloads)
3. Download [Ubuntu ISO 22.04.5 LTS](https://ubuntu.com/download/desktop)
4. Create new VirtualBox VM
5. Set RAM to 8192 MB
6. Set CPU cores to max available
7. Turn on Windows Hypervisor Platform under Windows Features
8. Boot Ubuntu VM
9. Turn on Shared Clipboard: VM -> Devices -> Shared Clipboard -> Bidirectional
10. Install VirtualBox Guest Additions: VM -> Devices -> Insert Guest Additions CD image...

#### Instructions followed for WSL:
1. Install WSL: ```wsl --install```
2. Install Ubuntu-22.04: ```wsl --install --d Ubuntu-22.04```
3. Setup user account: input UNIX username, input and repeat password

<br>

Install ROS2 Humble in Ubuntu by following [these](https://docs.ros.org/en/humble/) instructions.

## Setting up a ROS2 Humble workspace, and creating a package
Followed [this](https://docs.ros.org/en/eloquent/Tutorials/Creating-Your-First-ROS2-Package.html) documentation.
1. Create a ROS2 Humble workspace: ```mkdir ros_project/src```
2. Go into ROS2 Humble workspace: ```cd ros_project/src```
3. Create a ROS2 package: ```ros2 pkg create --build-type ament_cmake ros_project```

## Preparing TurtleBot3
1. Get all hardware required to physically build TurtleBot3
2. Download and install [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
3. Choose Pi Device (Pi 4 in our case)
4. Choose and setup OS
    - Other general-purpose OS --> Ubuntu --> Ubuntu Server 22.04.5 LTS (64-bit)
5. Choose storage: SD card from Pi
6. Hit "next"
7. Customize OS
    - Edit Settings --> Set Hostname, Set Username and Password, Configure wireless LAN --> Service: Enable SSH --> Save --> Hit "yes" --> Hit "yes" again
8. After flash: plug SD card into Raspberry Pi, make sure wireless network is available and startup the Pi
9. SSH into Ubuntu Server using ```ssh <username>@<ip-address>```
10. Install ROS2 Humble in Ubuntu Server by following [these](https://docs.ros.org/en/humble/) instructions.

## Testing the bot
#### Documentation used:
1. [TurtleBot 3 opencr-setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup)
    - To flash the OpenCR board
2. [TurtleBot 3 basic-operation](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#teleoperation)
    - To control/test the TurtleBot 3

#### Steps:
1. Login to the bot: ```ssh turtle@<ip-address>```
2. Run ```export TURTLEBOT3_MODEL=burger```
3. Run ```export LDS_MODEL=LDS-01```
4. Run ```ros2 launch turtlebot3_bringup robot.launch.py```
5. Open another terminal and repeat step 1, 2, 3
6. Run ```ros2 run turtlebot3_teleop teleop_keyboard```
7. Control the bot using keys WASDX
