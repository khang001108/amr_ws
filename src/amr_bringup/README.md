# AMR ROS 2 Jazzy — Hướng dẫn đầy đủ (SLAM + Nav2)

## Cấu trúc thư mục hoàn chỉnh

```
amr_bringup/
├── amr_bringup/
│   ├── __init__.py
│   └── amr_serial_bridge.py
├── launch/
│   ├── amr_launch.py            ← Bringup cơ bản
│   ├── slam_launch.py           ← Tạo map (SLAM Toolbox)
│   └── nav2_launch.py           ← Điều hướng tự động
├── config/
│   ├── amr_params.yaml
│   ├── slam_toolbox_params.yaml
│   ├── nav2_params.yaml
│   └── 99-amr-devices.rules     ← udev: tên cố định /dev/esp32 và /dev/lidar
├── urdf/
│   └── amr.urdf
├── package.xml / setup.py / setup.cfg
└── README.md
```

---

## Bước 0 — Cài đặt phụ thuộc

```bash
sudo apt update && sudo apt upgrade -y

# ROS 2 cơ bản
sudo apt install -y \
  ros-jazzy-rclpy ros-jazzy-geometry-msgs ros-jazzy-nav-msgs \
  ros-jazzy-sensor-msgs ros-jazzy-tf2-ros \
  ros-jazzy-robot-state-publisher ros-jazzy-rviz2

# SLAM
sudo apt install -y ros-jazzy-slam-toolbox

# Nav2
sudo apt install -y \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-map-server ros-jazzy-nav2-planner \
  ros-jazzy-nav2-controller ros-jazzy-nav2-behaviors \
  ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-dwb-core

pip3 install pyserial
```

---

## Bước 1 — Cài driver LiDAR

### RPLIDAR A1M8 (Slamtec):
```bash
sudo apt install -y ros-jazzy-rplidar-ros
```

### LDROBOT (LD06/LD14/LD19):
```bash
cd ~/amr_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
cd ~/amr_ws && colcon build --packages-select ldlidar_stl_ros2
```

---

## Bước 2 — Build package AMR

```bash
mkdir -p ~/amr_ws/src
cp -r amr_bringup ~/amr_ws/src/
cd ~/amr_ws
colcon build --packages-select amr_bringup
echo "source ~/amr_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Bước 3 — Cấu hình USB cố định

```bash
# Tìm serial của từng thiết bị
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|{serial}"
udevadm info -a -n /dev/ttyUSB1 | grep -E "idVendor|idProduct|{serial}"

# Chỉnh file 99-amr-devices.rules → điền serial thực
sudo cp config/99-amr-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Sau đó: ESP32 → /dev/esp32 | LiDAR → /dev/lidar
sudo usermod -aG dialout $USER   # logout + login lại
```

---

## Bước 4 — Tạo Map (SLAM)

```bash
# RPLIDAR A1M8
ros2 launch amr_bringup slam_launch.py \
  lidar:=rplidar esp_port:=/dev/esp32 lidar_port:=/dev/lidar use_rviz:=true

# LDROBOT LD19
ros2 launch amr_bringup slam_launch.py \
  lidar:=ldlidar lidar_model:=LDLiDAR_LD19 \
  esp_port:=/dev/esp32 lidar_port:=/dev/lidar use_rviz:=true
```

Điều khiển robot bằng teleop:
```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# i=tiến  ,=lùi  j=trái  l=phải  k=dừng
```

Lưu map khi đã vẽ xong:
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

---

## Bước 5 — Navigation (Nav2)

```bash
ros2 launch amr_bringup nav2_launch.py \
  map:=$HOME/maps/my_map.yaml \
  lidar:=rplidar \
  esp_port:=/dev/esp32 \
  lidar_port:=/dev/lidar \
  use_rviz:=true
```

Trong RViz2:
1. **"2D Pose Estimate"** → đặt vị trí ban đầu của robot
2. Chờ AMCL localise (particle tụ lại)
3. **"Nav2 Goal"** → click điểm đến → robot tự chạy!

---

## Troubleshooting

| Triệu chứng | Nguyên nhân | Fix |
|-------------|-------------|-----|
| RViz hiển thị quãng đường sai | Encoder CPR chưa hiệu chỉnh | Mặc định đã hiệu chỉnh 4400 counts/rev; đo lại quãng đường chuẩn và tinh chỉnh nếu cần |
| Robot không đi thẳng | Encoder/PID chưa cân bằng | Kiểm tra encoder hai bánh và tinh chỉnh PID |
| SLAM bị drift | Odom không chính xác | Kiểm tra /odom Hz, cân chỉnh encoder |
| Nav2 không tìm đường | Inflation radius lớn | Giảm inflation_radius trong nav2_params |
| Pi bị lag | CPU overload | Tăng throttle_scans, giảm controller_frequency |
| /scan không có data | Port sai hoặc baud sai | Kiểm tra ls /dev/tty*, thử 115200 vs 230400 |

---

## Kiểm tra nhanh

```bash
ros2 topic hz /scan        # ~10Hz
ros2 topic hz /odom        # ~20Hz
ros2 topic hz /imu/data    # ~20Hz
ros2 run tf2_tools view_frames   # Xem TF tree
```
