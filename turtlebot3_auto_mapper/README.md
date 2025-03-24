```ros2 pkg create --build-type ament_python turtlebot3_auto_mapper```
Create python file
```chmod +x file.py```
Edit setup.py file by adding 'autonomous_mapper = turtlebot3_auto_mapper.autonomous_mapper:main', to the entry_points
```colcon build --packages-select turtlebot3_auto_mapper```
```source ~/ros2_ws/install/setup.bash```
```ros2 run turtlebot3_auto_mapper autonomous_mapper```
