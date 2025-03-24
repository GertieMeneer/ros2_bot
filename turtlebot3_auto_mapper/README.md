1. ```ros2 pkg create --build-type ament_python turtlebot3_auto_mapper```
2. Create python file
3. ```chmod +x file.py```
4. Edit setup.py file by adding 'autonomous_mapper = turtlebot3_auto_mapper.autonomous_mapper:main', to the entry_points
5. ```colcon build --packages-select turtlebot3_auto_mapper```
6. ```source ~/ros2_ws/install/setup.bash```
7. ```ros2 run turtlebot3_auto_mapper autonomous_mapper```
