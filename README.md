<img width="1032" height="536" alt="image" src="https://github.com/user-attachments/assets/d8435df4-fc57-4690-8e7c-9f6c908e5f22" />

## ros2 launch turtlebot3_bringup robot.launch.py
## ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
## ros2 run safety safe_area
## ros2 run nav2_collision_monitor collision_monitor --ros-args \
## -p autostart:=True \
## --params-file /home/dcu/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/humble/collision_monitor.yaml

## ros2 lifecycle set /collision_monitor configure
## ros2 lifecycle set /collision_monitor activate
