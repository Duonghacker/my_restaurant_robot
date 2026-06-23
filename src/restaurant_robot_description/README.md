# restaurant_robot_description

Package ROS 2 mô tả robot tự hành phục vụ nhà hàng.

## Cấu trúc Package

```
restaurant_robot_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── robot.urdf.xacro      ← File chính (entry point)
│   ├── robot_core.xacro      ← Cấu trúc cơ học
│   └── gazebo.xacro          ← Plugin Gazebo
├── launch/
│   └── rsp.launch.py         ← Robot State Publisher launch
└── rviz/
    └── robot_view.rviz       ← Config RViz2
```

## Build Package

```bash
# 1. Tạo workspace (nếu chưa có)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Copy package vào src/
cp -r restaurant_robot_description .

# 3. Cài đặt dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --packages-select restaurant_robot_description

# 5. Source workspace
source install/setup.bash
```

## Chạy Robot State Publisher (xem URDF trong RViz2)

```bash
# Terminal 1: Launch RSP
source ~/ros2_ws/install/setup.bash
ros2 launch restaurant_robot_description rsp.launch.py

# Terminal 2: Mở RViz2
source ~/ros2_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix restaurant_robot_description)/share/restaurant_robot_description/rviz/robot_view.rviz
```

## Chạy với Gazebo (Simulation)

```bash
# Terminal 1: Khởi động Gazebo với world mặc định
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Spawn robot vào Gazebo
source ~/ros2_ws/install/setup.bash
ros2 launch restaurant_robot_description rsp.launch.py use_sim_time:=true

# Terminal 3: Spawn model
source ~/ros2_ws/install/setup.bash
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity restaurant_robot

# Terminal 4: Điều khiển robot bằng bàn phím
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Kiểm tra Topics

```bash
# Xem danh sách topics
ros2 topic list

# Xem Lidar data
ros2 topic echo /scan

# Xem Odometry
ros2 topic echo /odom

# Xem TF tree
ros2 run tf2_tools view_frames
```

## Thông số Robot

| Thành phần | Thông số |
|---|---|
| Chassis | Hình hộp 200 x 190 x 70 mm |
| Khối lượng | ~0.8 kg (tổng theo URDF) |
| Bánh xe | Ø65mm, dày 26mm, GA25 motor |
| Wheel separation | 220mm |
| Caster | Ø33mm, phía trước |
| Lidar | RPLidar A1, 360°, 0.15-12m |
