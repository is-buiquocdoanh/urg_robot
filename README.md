# LUMIR: Robot hỗ trợ người khiếm thị

## Robot (Raspberry PI)
### Terminal 1
```
source install/setup.bash
ros2 run robot_control cmd_vel_to_arduino
```

### Terminal 2
```
source install/setup.bash
ros2 launch urg_lidar urg_lidar.launch.py 
```

## Laptop

### Terminal 1: Khởi chạy description
```
source install/setup.bash
ros2 launch robot_description display.launch.py
```

### Terminal 2: khởi chạy joystick
```
source install/setup.bash
ros2 launch robot_joy joystick.launch.py 
```

### Terminal 3: Khởi chạy odometry (odom từ lidar)
(Note: Chỉ sử dụng khi bật nav2, cartographer thì không bật node này)
```
source install/setup.bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py 
```

### Terminal 4: Khởi chạy cartographer (Mapping)
```
source install/setup.bash
ros2 launch robot_mapping cartographer.launch.py
```
Lưu map:
```
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Terminal 5: khởi chạy navigation (khi đã có map)
```
source install/setup.bash
ros2 launch robot_navigation navigation.launch.py 
```