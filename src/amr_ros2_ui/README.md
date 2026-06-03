# AMR ROS2 Control Panel — Qt5 UI

## Cấu trúc files

```
amr_ros2_ui/
├── amr_main_window.ui        ← Qt Designer 5 layout file (mở bằng Qt Designer)
├── amr_control_panel.py      ← Ứng dụng chính, load .ui và kết nối ROS2
├── amr_serial_qt_bridge.py   ← Serial bridge Qt wrapper + PID/Odom dialogs
└── README.md
```

---

## Cài đặt

### 1. Cài PyQt5 và pyserial

```bash
pip3 install PyQt5 pyserial
```

### 2. Cài ROS2 Jazzy (Ubuntu 24.04)

```bash
# Theo hướng dẫn: https://docs.ros.org/en/jazzy/Installation.html
sudo apt install ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
```

### 3. Build workspace

```bash
cd ~/ros2_ws
colcon build --packages-select amr_bringup
source install/setup.bash
```

---

## Chạy ứng dụng

### Có ROS2
```bash
# Terminal 1 — khởi động ROS2 + robot
source /opt/ros/jazzy/setup.bash
ros2 launch amr_bringup amr_launch.py

# Terminal 2 — mở UI
python3 amr_control_panel.py
```

### Không có ROS2 (Demo Mode)
```bash
python3 amr_control_panel.py
# → Chạy với robot ảo, hiển thị demo animation
```

---

## Mở file .ui trong Qt Designer

```bash
designer amr_main_window.ui
# hoặc
qtchooser -run-tool=designer -qt=5
```

---

## Các tính năng UI

| Panel | Chức năng |
|-------|-----------|
| **Serial Connection** | Chọn port, baudrate, kết nối/ngắt |
| **Robot State** | Hiển thị x/y/yaw, velocity, encoder realtime |
| **Manual Control** | D-pad teleop, slider vận tốc, max linear/angular |
| **Emergency Stop** | Dừng khẩn cấp (màu đỏ nổi bật) |
| **Map View** | Vẽ bản đồ, LiDAR, path, robot pose, goal |
| **Navigation (Nav2)** | Start/Stop Nav2, nhập goal x/y/yaw, gửi goal |
| **SLAM Toolbox** | Start/Stop SLAM, Save/Load map, chọn mode |
| **LiDAR (RPLidar)** | Start/Stop, chọn model A1/A2/A3/S1/S2 |
| **Console Log** | Log màu theo level, nhập lệnh ROS2 |
| **ROS2 Topics** | Xem danh sách topics |

---

## Giao thức Serial (firmware)

Giao tiếp qua UART với `amr_serial_bridge.py`:

**PC → MCU:**
```
CMD,<linear*100>,<angular*100>\n   # cmd_vel
RESET_ODOM\n                        # reset odometry
PID,<kp>,<ki>,<kd>\n               # PID params
```

**MCU → PC:**
```
ODOM,<x>,<y>,<yaw>,<vx>,<vz>\n    # odometry
ENC,<left>,<right>\n                # encoder counts  
BAT,<voltage_mv>\n                  # battery
ERR,<code>\n                        # error
```

---

## Tuỳ chỉnh stylesheet

Toàn bộ giao diện màu tối (GitHub Dark) được định nghĩa trong
`amr_main_window.ui` → `styleSheet` của `QMainWindow`.

Màu chính:
- Background: `#0d1117`
- Panel: `#161b22`  
- Accent Blue: `#58a6ff`
- Success Green: `#238636`
- Warning Yellow: `#fbbf24`
- Error Red: `#f87171`
