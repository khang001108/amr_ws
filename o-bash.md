copy đoạn này vào nano ~/.bashrc => dùng như 1 lệnh tắt


#=================== cau hinh ros2_ws ==============

# ROS2
source /opt/ros/jazzy/setup.bash
# source ~/ros2_ws/install/setup.bash
source ~/amr_ws/install/setup.bash
ls -l /dev | grep esp32
ls -l /dev | grep lidar
# source /home/ros2/.bashrc
#sudo udevadm control --reload-rules


alias battery='vcgencmd get_throttled'
alias all-usb='ls /dev/ttyUSB*'
alias o-bash='code ~/.bashrc'
alias rules-reset='sudo udevadm control --reload-rules'
alias rules-trigger='sudo udevadm trigger'

#=============================
#           DDS
#=============================
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export ROS_IP=192.168.1.28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

alias ip='hostname -I'

#=============================
#           SSH
#=============================
alias pi5='nautilus "sftp://khazg@192.168.1.28"'
alias ssh-pi5="ssh khazg@192.168.1.28"

alias laptop='nautilus "sftp://ros2@192.168.1.22"'
alias ssh-lap="ssh ros2@192.168.1.22"
alias ssh-enable='sudo systemctl enable ssh && sudo systemctl start ssh'
alias ssh-disable='sudo systemctl disable ssh && sudo systemctl stop ssh'
alias ssh-restart='sudo systemctl restart ssh'
alias ssh-status='sudo systemctl status ssh'

# ============================
#       FOR ROS2 JAZZY
# ============================
# =========== ESP32 =================
alias esp-usb='ls -l /dev | grep esp32'
alias esp-usb-='ls -l /dev/esp32'
alias esp-rules='sudo nano /etc/udev/rules.d/esp32.rules'
# =========== LIDAR =================
alias lidar-usb='ls -l /dev | grep lidar'
alias lidar-usb-='ls -l /dev/lidar'
alias lidar-rules='sudo nano /etc/udev/rules.d/lidar.rules'
alias lidar-run='ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/lidar'
# =========== TOPIC =================
alias topic='ros2 topic list'
alias topic-odom='ros2 topic echo /odom'
alias topic-imu='ros2 topic echo /imu'
alias topic-scan='ros2 topic echo /imu'
# =========== ONCE =================
alias ros-odom-once='ros2 topic echo /odom --once'
alias ros-scan-once='ros2 topic echo /scan --once'
alias ros-lidar-frame='ros2 topic echo /scan | grep frame_id'
# =========== BUILD =================
alias build='colcon build --symlink-install'
alias setup='source install/setup.bash'
alias teleo='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
alias tf-view='ros2 run tf2_tools view_frames'



#=============================
#          SLAM - NAV
#=============================
alias map-save='ros2 run nav2_map_server map_saver_cli -f ~/map'
alias localize='ros2 launch nav2_bringup localization_launch.py map:=/home/ros2/map.yaml'
alias nav='ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false'
alias slam-run='ros2 launch slam_toolbox online_async_launch.py'
alias slam-config='ros2 lifecycle set /slam_toolbox configure'
alias slam-active='ros2 lifecycle set /slam_toolbox activate'
alias slam-get='ros2 lifecycle get /slam_toolbox'

#=============================
#           ROS2_WS
#=============================
alias ros='cd ~/ros2_ws'
alias ros-imu-run='ros2 run imu_serial imu_node'
alias ros-my_robot='ros2 run robot_state_publisher robot_state_publisher ~/ros2_ws/src/my_robot/urdf/robot.urdf'
alias ros-odom-run='ros2 run odom_bridge odom_publisher'
alias ros-esp-run='ros2 run esp32_bridge esp_bridge'
alias ros-robot='ros2 launch my_robot_bringup bringup.launch.py'

#=============================
#           AMR_WS
#=============================
alias amr='cd ~/amr_ws'
alias amr-rules='sudo nano /etc/udev/rules.d/99-amr-devices.rules'
alias amr-run='ros2 launch amr_bringup amr_launch.py'
alias amr-slam-run='ros2 launch amr_bringup slam_launch.py'
alias amr-nav-run='ros2 launch amr_bringup nav2_launch.py'



#alias ros-esp-run='ros2 run esp32_bridge esp_bridge --ros-args -p port:=/dev/esp32'
#alias ros-esp-run='ros2 run esp32_bridge esp_bridge'
#lias ros-lidar-run='ros2 launch rplidar_ros rplidar_a1_launch.py'
#alias ros-my-robot='ros2 run robot_state_publisher robot_state_publisher \ ~/ros2_ws/src/my_robot/urdf/robot.urdf'
#alias ros-lidar-run='ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/rplidar serial_baudrate:=115200'
#alias ros-lidar-run='ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/rplidar serial_baudrate:=115200'

# export NVM_DIR="$HOME/.nvm"
# [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
# [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion
