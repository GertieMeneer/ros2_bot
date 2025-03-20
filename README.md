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

## How to see lidar data in rviz on computer from bot
Install libraries for lds 02 because lds 01 isn't the sensor we have yikes

1. Setup X11 in SSH
    - ```sudo apt install x11-apps```
    - This will install necessary packages to allow X11 apps
2. SSH into the bot using ```ssh -X ...```
    - This will SSH into the bot using X11 forwarding
    - X11 forwarding allows you to run GUI applications on a remote server while displaying them on your local machine
3. Run ```export TURTLEBOT3_MODEL=burger```
4. Run ```ros2 launch turtlebot3_cartographer cartographer.launch.py```
    - This will launch rviz via X11 on local machine
5. Change rviz settings
    - Set ```Fixed Frame``` to ```base_scan```
    - ```Laser Scan``` --> ```reliability```: set to ```best effort```
  
## How to make bot drive to 2D goal 
1. Run ```ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}```
2. Run ```ros2 launch turtlebot3_bringup robot.launch.py```
3. Open a new terminal
4. Run ```ssh -X ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}```
5. Run ```ros2 launch turtlebot3_cartographer cartographer.launch.py```
6. Open a new terminal
7. Run ```ssh -X ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}```
8. Run ```ros2 run turtlebot3_teleop teleop_keyboard```
9. Drive around with the bot using the teleop keys
10. CTRL C out of the teleop terminal
11. Run ```ros2 run nav2_map_server map_saver_cli -f ~/map```
12. Run ```ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml```
13. Rviz will start running and when setting a goal by using the button at the top of Rviz the bot drives to the set goal

## Problems we faced during the project
WSL not working with ssh into bot.
Fix was to install ubuntu on VM.

When install turtlebot3 there is a commando ```rm -r turtlebot3_cartographer turtlebot3_navigation2``` this uninstalls the cartographer and the navigation. Making the bot not able to use SLAM.
Fix do not run this commando so it's not removed so SLAM works.
