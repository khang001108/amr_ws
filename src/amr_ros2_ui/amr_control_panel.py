#!/usr/bin/env python3
"""
AMR ROS2 Control Panel
Kết nối Qt5 UI (amr_main_window.ui) với ROS2

Yêu cầu:
    pip install PyQt5
    sudo apt install python3-rclpy ros-jazzy-*

Chạy:
    python3 amr_control_panel.py
"""

import sys
import os
import math
import threading
import time
from datetime import datetime

from PyQt5 import QtWidgets, QtCore, QtGui, uic
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF
from PyQt5.QtWidgets import QFileDialog, QMessageBox

# ─── ROS2 import (bắt buộc có ROS2 Jazzy) ────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from nav2_msgs.action import NavigateToPose
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[WARNING] ROS2 not found — running in DEMO mode")

# ─── Đường dẫn tới file .ui ───────────────────────────────────────────────────
UI_FILE = os.path.join(os.path.dirname(__file__), "amr_main_window.ui")


# ══════════════════════════════════════════════════════════════════════════════
#  MapWidget — vẽ bản đồ, lidar, robot lên QFrame
# ══════════════════════════════════════════════════════════════════════════════
class MapWidget(QtWidgets.QWidget):
    """Widget vẽ bản đồ 2D, tia LiDAR, pose robot, path, goal"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 300)

        # Dữ liệu
        self.robot_x = 0.0          # m
        self.robot_y = 0.0          # m
        self.robot_yaw = 0.0        # rad
        self.scan_ranges = []        # list[float]
        self.scan_angle_min = -math.pi
        self.scan_angle_increment = 0.01
        self.path_points = []        # list[(x,y)]
        self.goal_x = None
        self.goal_y = None
        self.map_data = None         # OccupancyGrid hoặc None

        # View
        self.scale = 60.0           # px/m
        self.offset_x = 0.0        # px offset (pan)
        self.offset_y = 0.0
        self._drag_start = None

        # Options
        self.show_lidar = True
        self.show_path = True
        self.show_costmap = False

        # Grid
        self.setMouseTracking(True)
        self.mouse_map_x = 0.0
        self.mouse_map_y = 0.0

    # ── Cập nhật dữ liệu ──────────────────────────────────────────────────────
    def update_robot_pose(self, x, y, yaw):
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        self.update()

    def update_scan(self, ranges, angle_min, angle_increment):
        self.scan_ranges = ranges
        self.scan_angle_min = angle_min
        self.scan_angle_increment = angle_increment
        self.update()

    def update_path(self, points):
        self.path_points = points
        self.update()

    def set_goal(self, x, y):
        self.goal_x = x
        self.goal_y = y
        self.update()

    def fit_view(self):
        self.scale = 60.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.update()

    # ── Toạ độ: map (m) <-> widget (px) ──────────────────────────────────────
    def map_to_widget(self, mx, my):
        cx = self.width() / 2 + self.offset_x
        cy = self.height() / 2 + self.offset_y
        wx = cx + mx * self.scale
        wy = cy - my * self.scale          # Y lật
        return wx, wy

    def widget_to_map(self, wx, wy):
        cx = self.width() / 2 + self.offset_x
        cy = self.height() / 2 + self.offset_y
        mx = (wx - cx) / self.scale
        my = -(wy - cy) / self.scale
        return mx, my

    # ── Mouse events ──────────────────────────────────────────────────────────
    def mousePressEvent(self, e):
        if e.button() == Qt.MiddleButton or e.button() == Qt.LeftButton:
            self._drag_start = (e.x(), e.y(), self.offset_x, self.offset_y)

    def mouseMoveEvent(self, e):
        mx, my = self.widget_to_map(e.x(), e.y())
        self.mouse_map_x = mx
        self.mouse_map_y = my
        if self._drag_start and (e.buttons() & (Qt.LeftButton | Qt.MiddleButton)):
            sx, sy, ox, oy = self._drag_start
            self.offset_x = ox + (e.x() - sx)
            self.offset_y = oy + (e.y() - sy)
            self.update()

    def mouseReleaseEvent(self, e):
        self._drag_start = None

    def wheelEvent(self, e):
        factor = 1.15 if e.angleDelta().y() > 0 else 1 / 1.15
        self.scale = max(10.0, min(500.0, self.scale * factor))
        self.update()

    # ── Paint ─────────────────────────────────────────────────────────────────
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # Background
        p.fillRect(self.rect(), QColor("#050a0f"))

        # Grid
        self._draw_grid(p)

        # LiDAR scan
        if self.show_lidar and self.scan_ranges:
            self._draw_scan(p)

        # Path
        if self.show_path and self.path_points:
            self._draw_path(p)

        # Goal
        if self.goal_x is not None:
            self._draw_goal(p)

        # Robot
        self._draw_robot(p)

        # Origin cross
        ox, oy = self.map_to_widget(0, 0)
        p.setPen(QPen(QColor("#30363d"), 1))
        p.drawLine(int(ox) - 8, int(oy), int(ox) + 8, int(oy))
        p.drawLine(int(ox), int(oy) - 8, int(ox), int(oy) + 8)

        # Coords display
        p.setPen(QColor("#8b949e"))
        p.setFont(QFont("Courier New", 9))
        p.drawText(8, self.height() - 8,
                   f"Map: ({self.mouse_map_x:.2f}, {self.mouse_map_y:.2f}) m  |  "
                   f"Scale: {self.scale:.0f} px/m")

    def _draw_grid(self, p):
        grid_spacing = 1.0   # 1 m
        pen = QPen(QColor("#1a2030"), 1, Qt.DotLine)
        p.setPen(pen)
        # Vertical lines
        x0 = self.width() / 2 + self.offset_x
        step_px = grid_spacing * self.scale
        if step_px < 8:
            return
        start_i = int(-x0 / step_px) - 1
        end_i = int((self.width() - x0) / step_px) + 1
        for i in range(start_i, end_i + 1):
            x = int(x0 + i * step_px)
            p.drawLine(x, 0, x, self.height())
        # Horizontal lines
        y0 = self.height() / 2 + self.offset_y
        start_j = int(-y0 / step_px) - 1
        end_j = int((self.height() - y0) / step_px) + 1
        for j in range(start_j, end_j + 1):
            y = int(y0 + j * step_px)
            p.drawLine(0, y, self.width(), y)

    def _draw_scan(self, p):
        p.setPen(QPen(QColor(20, 180, 255, 80), 1))
        rx, ry = self.robot_x, self.robot_y
        for i, r in enumerate(self.scan_ranges):
            if r == float('inf') or r > 12.0 or r < 0.05:
                continue
            angle = self.scan_angle_min + i * self.scan_angle_increment + self.robot_yaw
            px = rx + r * math.cos(angle)
            py = ry + r * math.sin(angle)
            wx, wy = self.map_to_widget(px, py)
            p.drawPoint(int(wx), int(wy))

    def _draw_path(self, p):
        if len(self.path_points) < 2:
            return
        pen = QPen(QColor("#3b82f6"), 2)
        pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        pts = [self.map_to_widget(x, y) for x, y in self.path_points]
        for i in range(len(pts) - 1):
            p.drawLine(int(pts[i][0]), int(pts[i][1]),
                       int(pts[i+1][0]), int(pts[i+1][1]))

    def _draw_goal(self, p):
        gx, gy = self.map_to_widget(self.goal_x, self.goal_y)
        # Circle
        p.setPen(QPen(QColor("#f59e0b"), 2))
        p.setBrush(QBrush(QColor(245, 158, 11, 50)))
        r = 12
        p.drawEllipse(int(gx) - r, int(gy) - r, 2*r, 2*r)
        # Cross
        p.setPen(QPen(QColor("#f59e0b"), 2))
        p.drawLine(int(gx) - r, int(gy), int(gx) + r, int(gy))
        p.drawLine(int(gx), int(gy) - r, int(gx), int(gy) + r)
        p.setFont(QFont("Arial", 8))
        p.drawText(int(gx) + 14, int(gy) + 4, "GOAL")

    def _draw_robot(self, p):
        rx, ry = self.map_to_widget(self.robot_x, self.robot_y)
        size = max(14, int(self.scale * 0.35))

        p.save()
        p.translate(int(rx), int(ry))
        p.rotate(-math.degrees(self.robot_yaw))

        # Body
        p.setPen(QPen(QColor("#58a6ff"), 2))
        p.setBrush(QBrush(QColor(31, 111, 235, 180)))
        p.drawEllipse(-size, -size, size*2, size*2)

        # Arrow (heading)
        p.setPen(QPen(QColor("#e6edf3"), 2))
        p.drawLine(0, 0, size + 4, 0)
        # Arrowhead
        poly = QPolygonF()
        poly.append(QtCore.QPointF(size + 4, 0))
        poly.append(QtCore.QPointF(size - 2, -4))
        poly.append(QtCore.QPointF(size - 2,  4))
        p.setBrush(QBrush(QColor("#e6edf3")))
        p.drawPolygon(poly)

        p.restore()

        # Label
        p.setPen(QColor("#e6edf3"))
        p.setFont(QFont("Arial", 8, QFont.Bold))
        p.drawText(int(rx) + size + 6, int(ry) + 4, "AMR")


# ══════════════════════════════════════════════════════════════════════════════
#  ROS2Worker — chạy spin() trong thread riêng
# ══════════════════════════════════════════════════════════════════════════════
if ROS2_AVAILABLE:
    class AMRNode(Node):
        def __init__(self, signals):
            super().__init__("amr_control_panel")
            self.signals = signals

            # Publishers
            self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

            # Subscribers
            self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
            self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
            self.create_subscription(OccupancyGrid, "/map", self._map_cb, 1)
            self.create_subscription(Path, "/plan", self._path_cb, 5)

            # Nav2 action client
            self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self.get_logger().info("Nav2 action client ready (Jazzy)")

            self.get_logger().info("AMR Control Panel Node started")

        def _odom_cb(self, msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            # quaternion to yaw
            siny = 2*(q.w*q.z + q.x*q.y)
            cosy = 1 - 2*(q.y*q.y + q.z*q.z)
            yaw = math.atan2(siny, cosy)
            vx = msg.twist.twist.linear.x
            vz = msg.twist.twist.angular.z
            self.signals.odom_received.emit(x, y, yaw, vx, vz)

        def _scan_cb(self, msg):
            self.signals.scan_received.emit(
                list(msg.ranges), msg.angle_min, msg.angle_increment)

        def _map_cb(self, msg):
            self.signals.map_received.emit(msg)

        def _path_cb(self, msg):
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.signals.path_received.emit(pts)

        def publish_cmd_vel(self, linear, angular):
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.cmd_vel_pub.publish(msg)

        def send_goal(self, x, y, yaw):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(x)
            goal_msg.pose.pose.position.y = float(y)
            cy = math.cos(math.radians(yaw) / 2)
            sy = math.sin(math.radians(yaw) / 2)
            goal_msg.pose.pose.orientation.w = cy
            goal_msg.pose.pose.orientation.z = sy
            # Jazzy: behavior_tree field có thể để rỗng
            goal_msg.behavior_tree = ''
            self.nav_client.send_goal_async(goal_msg)

    class ROS2Signals(QObject):
        odom_received  = pyqtSignal(float, float, float, float, float)
        scan_received  = pyqtSignal(list, float, float)
        map_received   = pyqtSignal(object)
        path_received  = pyqtSignal(list)
        log_message    = pyqtSignal(str, str)   # level, text


# ══════════════════════════════════════════════════════════════════════════════
#  MainWindow
# ══════════════════════════════════════════════════════════════════════════════
class AMRMainWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()
        uic.loadUi(UI_FILE, self)

        self._setup_map_widget()
        self._setup_timers()
        self._connect_signals()
        self._setup_ros2()
        self._demo_mode_if_needed()

        self.log("[AMR] Control Panel started", "INFO")
        self.statusBar().showMessage("Ready — " +
            ("ROS2 Active" if ROS2_AVAILABLE else "DEMO Mode"))

    # ── Thay thế map_frame bằng MapWidget ────────────────────────────────────
    def _setup_map_widget(self):
        self.map_widget = MapWidget(self)
        # SizePolicy: Expanding cả 2 chiều để chiếm hết không gian
        sp = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding)
        sp.setVerticalStretch(1)
        self.map_widget.setSizePolicy(sp)
        self.map_widget.setMinimumHeight(100)

        layout = self.map_frame.parent().layout()
        # Tìm và thay thế map_frame
        for i in range(layout.count()):
            item = layout.itemAt(i)
            if item and item.widget() == self.map_frame:
                layout.removeWidget(self.map_frame)
                self.map_frame.hide()
                layout.insertWidget(i, self.map_widget, stretch=1)
                break

    # ── Timers ────────────────────────────────────────────────────────────────
    def _setup_timers(self):
        # Đồng hồ
        self._clock_timer = QTimer(self)
        self._clock_timer.timeout.connect(self._update_clock)
        self._clock_timer.start(1000)

        # Demo animation
        self._demo_timer = QTimer(self)
        self._demo_timer.timeout.connect(self._demo_tick)
        self._demo_angle = 0.0

    # ── Kết nối tất cả signals / slots ───────────────────────────────────────
    def _connect_signals(self):
        # Connection
        self.btn_connect.clicked.connect(self._on_connect)
        self.btn_disconnect.clicked.connect(self._on_disconnect)
        self.btn_refresh_ports.clicked.connect(self._refresh_ports)

        # Teleop D-pad (pressed/released để giữ phím)
        self.btn_forward.pressed.connect(lambda: self._drive(1, 0))
        self.btn_backward.pressed.connect(lambda: self._drive(-1, 0))
        self.btn_left.pressed.connect(lambda: self._drive(0, 1))
        self.btn_right.pressed.connect(lambda: self._drive(0, -1))
        self.btn_forward.released.connect(self._stop_drive)
        self.btn_backward.released.connect(self._stop_drive)
        self.btn_left.released.connect(self._stop_drive)
        self.btn_right.released.connect(self._stop_drive)
        self.btn_stop.clicked.connect(self._stop_drive)

        # E-STOP
        self.btn_emergency_stop.clicked.connect(self._emergency_stop)

        # Velocity slider
        self.slider_max_vel.valueChanged.connect(
            lambda v: self.lbl_vel_pct.setText(f"{v}%"))

        # Navigation
        self.btn_nav_start.clicked.connect(self._start_nav2)
        self.btn_nav_stop.clicked.connect(self._stop_nav2)
        self.btn_send_goal.clicked.connect(self._send_goal)
        self.btn_cancel_goal.clicked.connect(self._cancel_goal)
        self.btn_browse_map.clicked.connect(self._browse_map)

        # SLAM
        self.btn_slam_start.clicked.connect(self._start_slam)
        self.btn_slam_stop.clicked.connect(self._stop_slam)
        self.btn_save_map.clicked.connect(self._save_map)
        self.btn_load_map.clicked.connect(self._load_map)

        # LiDAR
        self.btn_lidar_start.clicked.connect(self._start_lidar)
        self.btn_lidar_stop.clicked.connect(self._stop_lidar)

        # Map toolbar
        self.btn_map_fit.clicked.connect(self.map_widget.fit_view)
        self.btn_map_zoom_in.clicked.connect(
            lambda: setattr(self.map_widget, 'scale',
                            min(500.0, self.map_widget.scale * 1.3)) or
                    self.map_widget.update())
        self.btn_map_zoom_out.clicked.connect(
            lambda: setattr(self.map_widget, 'scale',
                            max(10.0, self.map_widget.scale / 1.3)) or
                    self.map_widget.update())
        self.chk_show_lidar.toggled.connect(
            lambda v: setattr(self.map_widget, 'show_lidar', v) or
                      self.map_widget.update())
        self.chk_show_path.toggled.connect(
            lambda v: setattr(self.map_widget, 'show_path', v) or
                      self.map_widget.update())
        self.chk_show_costmap.toggled.connect(
            lambda v: setattr(self.map_widget, 'show_costmap', v) or
                      self.map_widget.update())

        # Console
        self.btn_cmd_send.clicked.connect(self._send_ros_command)
        self.btn_console_clear.clicked.connect(self.text_console.clear)
        self.edit_cmd_input.returnPressed.connect(self._send_ros_command)

        # Topics
        self.btn_refresh_topics.clicked.connect(self._refresh_topics)

        # Menu
        self.action_exit.triggered.connect(self.close)
        self.action_show_rviz.triggered.connect(
            lambda: os.system("rviz2 &"))
        self.action_show_rqt.triggered.connect(
            lambda: os.system("rqt &"))
        self.action_launch_bringup.triggered.connect(self._launch_bringup)
        self.action_kill_all_nodes.triggered.connect(self._kill_nodes)
        self.action_about.triggered.connect(self._show_about)
        self.action_save_config.triggered.connect(self._save_config)
        self.action_open_config.triggered.connect(self._open_config)

    # ── ROS2 init ─────────────────────────────────────────────────────────────
    def _setup_ros2(self):
        self.ros2_node = None
        self.ros2_thread = None

        if not ROS2_AVAILABLE:
            return

        # Jazzy: init với context riêng để tránh xung đột QSocketNotifier
        try:
            rclpy.init(args=None)
        except RuntimeError:
            pass  # Đã init rồi
        self._ros2_signals = ROS2Signals()
        self._ros2_signals.odom_received.connect(self._on_odom)
        self._ros2_signals.scan_received.connect(self._on_scan)
        self._ros2_signals.path_received.connect(self._on_path)

        self.ros2_node = AMRNode(self._ros2_signals)

        # Jazzy: dùng SingleThreadedExecutor trong thread riêng
        # → tránh lỗi 'QSocketNotifier: Can only be used with threads started with QThread'
        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.ros2_node)
        self.ros2_thread = threading.Thread(
            target=self._executor.spin, daemon=True)
        self.ros2_thread.start()

        self._update_ros_status(True)
        self.log("[ROS2] Node started successfully", "INFO")

    # ── Demo mode ─────────────────────────────────────────────────────────────
    def _demo_mode_if_needed(self):
        if not ROS2_AVAILABLE:
            self._demo_timer.start(100)
            self.log("[DEMO] Running without ROS2", "WARN")
            self.log("[DEMO] Install ROS2 Jazzy to connect real robot", "WARN")
            self._update_ros_status(False)
            # Fake scan demo
            self._fake_scan = [2.5 + math.sin(i * 0.1) * 0.5
                                for i in range(360)]

    def _demo_tick(self):
        self._demo_angle += 0.03
        # Simulate robot moving
        x = math.cos(self._demo_angle * 0.3) * 1.5
        y = math.sin(self._demo_angle * 0.3) * 1.5
        yaw = self._demo_angle * 0.5
        self._on_odom(x, y, yaw, 0.1, 0.05)
        # Update fake scan
        scan = [2.0 + math.sin(i * 0.05 + self._demo_angle) * 0.8
                for i in range(360)]
        self._on_scan(scan, -math.pi, math.pi / 180)

    # ═════════════════════════════════════════════════════════════════════════
    #  SLOT handlers
    # ═════════════════════════════════════════════════════════════════════════

    def _on_odom(self, x, y, yaw, vx, vz):
        self.map_widget.update_robot_pose(x, y, yaw)
        self.lbl_pos_x.setText(f"{x:.3f} m")
        self.lbl_pos_y.setText(f"{y:.3f} m")
        self.lbl_heading.setText(f"{math.degrees(yaw):.1f}°")
        self.lbl_linear_vel.setText(f"{vx:.3f} m/s")
        self.lbl_angular_vel.setText(f"{vz:.3f} rad/s")
        self.lbl_map_coords.setText(f"Robot: ({x:.2f}, {y:.2f})")

    def _on_scan(self, ranges, angle_min, angle_inc):
        self.map_widget.update_scan(ranges, angle_min, angle_inc)
        self.lbl_scan_points.setText(str(len(ranges)))
        self.lbl_scan_rate.setText("10 Hz")

    def _on_path(self, points):
        self.map_widget.update_path(points)

    # ── Connection ────────────────────────────────────────────────────────────
    def _on_connect(self):
        port = self.combo_serial_port.currentText()
        baud = self.combo_baudrate.currentText()
        self.log(f"[SERIAL] Connecting to {port} @ {baud} baud...", "INFO")
        # Thực tế: gọi amr_serial_bridge connect
        self.lbl_conn_dot.setStyleSheet(
            "background-color:#238636;border-radius:5px;"
            "min-width:10px;max-width:10px;min-height:10px;max-height:10px;")
        self.lbl_conn_status.setText("Connected")
        self.lbl_conn_status.setStyleSheet("color:#7ee787;font-size:10px;")
        self.log(f"[SERIAL] Connected: {port}", "INFO")
        self.statusBar().showMessage(f"Connected: {port} @ {baud}")

    def _on_disconnect(self):
        self.log("[SERIAL] Disconnected", "WARN")
        self.lbl_conn_dot.setStyleSheet(
            "background-color:#b91c1c;border-radius:5px;"
            "min-width:10px;max-width:10px;min-height:10px;max-height:10px;")
        self.lbl_conn_status.setText("Disconnected")
        self.lbl_conn_status.setStyleSheet("color:#f87171;font-size:10px;")
        self.statusBar().showMessage("Disconnected")

    def _refresh_ports(self):
        import glob
        self.combo_serial_port.clear()
        ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        if not ports:
            ports = ["/dev/ttyUSB0"]
        for p in ports:
            self.combo_serial_port.addItem(p)
        self.log(f"[SERIAL] Found ports: {ports}", "INFO")

    # ── Teleop ────────────────────────────────────────────────────────────────
    def _drive(self, fwd, rot):
        vel_pct = self.slider_max_vel.value() / 100.0
        linear  = fwd * self.spin_max_linear.value() * vel_pct
        angular = rot * self.spin_max_angular.value() * vel_pct
        if self.ros2_node:
            self.ros2_node.publish_cmd_vel(linear, angular)
        self.lbl_linear_vel.setText(f"{linear:.3f} m/s")
        self.lbl_angular_vel.setText(f"{angular:.3f} rad/s")

    def _stop_drive(self):
        if self.ros2_node:
            self.ros2_node.publish_cmd_vel(0.0, 0.0)

    def _emergency_stop(self):
        if self.ros2_node:
            self.ros2_node.publish_cmd_vel(0.0, 0.0)
        self.log("[ESTOP] ⚠ EMERGENCY STOP ACTIVATED ⚠", "ERROR")
        self.statusBar().showMessage("⚠ EMERGENCY STOP")

    # ── Navigation ────────────────────────────────────────────────────────────
    def _start_nav2(self):
        os.system("ros2 launch amr_bringup nav2_launch.py &")
        self.lbl_nav_status.setText("STARTING")
        self.lbl_nav_status.setStyleSheet("color:#fbbf24;font-weight:bold;")
        self.log("[NAV2] Starting navigation stack...", "INFO")

    def _stop_nav2(self):
        self.lbl_nav_status.setText("IDLE")
        self.lbl_nav_status.setStyleSheet("color:#8b949e;font-weight:bold;")
        self.log("[NAV2] Navigation stopped", "WARN")

    def _send_goal(self):
        x   = self.spin_goal_x.value()
        y   = self.spin_goal_y.value()
        yaw = self.spin_goal_yaw.value()
        self.map_widget.set_goal(x, y)
        if self.ros2_node:
            self.ros2_node.send_goal(x, y, yaw)
        self.lbl_nav_goal.setText(f"({x:.1f}, {y:.1f})")
        self.lbl_nav_status.setText("NAVIGATING")
        self.lbl_nav_status.setStyleSheet("color:#7ee787;font-weight:bold;")
        self.log(f"[NAV2] Goal sent: x={x:.2f} y={y:.2f} yaw={yaw:.1f}°", "INFO")

    def _cancel_goal(self):
        self.map_widget.goal_x = None
        self.map_widget.goal_y = None
        self.map_widget.update()
        self.lbl_nav_goal.setText("None")
        self.lbl_nav_status.setText("IDLE")
        self.lbl_nav_status.setStyleSheet("color:#8b949e;font-weight:bold;")
        self.log("[NAV2] Goal cancelled", "WARN")

    def _browse_map(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select Map File", "", "YAML Files (*.yaml);;All (*)")
        if path:
            self.edit_map_file.setText(path)

    # ── SLAM ──────────────────────────────────────────────────────────────────
    def _start_slam(self):
        mode = self.combo_slam_mode.currentText().lower()
        os.system(f"ros2 launch amr_bringup slam_launch.py slam_mode:={mode} &")
        self.lbl_slam_status.setText("Running")
        self.lbl_slam_status.setStyleSheet("color:#7ee787;")
        self.log(f"[SLAM] Started in {mode} mode", "INFO")

    def _stop_slam(self):
        self.lbl_slam_status.setText("Stopped")
        self.lbl_slam_status.setStyleSheet("color:#f87171;")
        self.log("[SLAM] Stopped", "WARN")

    def _save_map(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Map", "my_map", "YAML (*.yaml)")
        if path:
            name = path.replace(".yaml", "")
            os.system(f"ros2 run nav2_map_server map_saver_cli -f {name} &")
            self.log(f"[SLAM] Map saved to: {name}", "INFO")

    def _load_map(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Map", "", "YAML (*.yaml)")
        if path:
            os.system(f"ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={path} &")
            self.log(f"[SLAM] Map loaded: {path}", "INFO")

    # ── LiDAR ─────────────────────────────────────────────────────────────────
    def _start_lidar(self):
        model = self.combo_lidar_model.currentText().lower().replace(" ", "_")
        # Jazzy: rplidar_ros dùng rplidar_launch.py với param serial_port
        model_lower = model.replace(" ","_").lower()
        os.system(f"ros2 launch rplidar_ros rplidar_launch.py serial_port:=/dev/ttyUSB0 &")
        self.lbl_lidar_status.setText("Running")
        self.lbl_lidar_status.setStyleSheet("color:#7ee787;")
        self.log(f"[LIDAR] Started: {model}", "INFO")

    def _stop_lidar(self):
        self.lbl_lidar_status.setText("Stopped")
        self.lbl_lidar_status.setStyleSheet("color:#f87171;")
        self.log("[LIDAR] Stopped", "WARN")

    # ── Console / Topics ──────────────────────────────────────────────────────
    def _send_ros_command(self):
        cmd = self.edit_cmd_input.text().strip()
        if not cmd:
            return
        self.log(f"$ {cmd}", "CMD")
        self.edit_cmd_input.clear()
        # Execute in background
        threading.Thread(
            target=lambda: os.system(cmd), daemon=True).start()

    def _refresh_topics(self):
        def _do():
            result = os.popen("ros2 topic list --show-types 2>/dev/null").read()
            if not result:
                result = "/cmd_vel [geometry_msgs/msg/Twist]\n/odom [nav_msgs/msg/Odometry]\n/scan [sensor_msgs/msg/LaserScan]\n/map [nav_msgs/msg/OccupancyGrid]"
            lines = []
            for line in result.strip().split("\n"):
                parts = line.split(" ")
                if len(parts) == 2:
                    lines.append(f"<span style='color:#58a6ff'>{parts[0]}</span>"
                                 f" <span style='color:#8b949e'>{parts[1]}</span>")
                else:
                    lines.append(f"<span style='color:#58a6ff'>{line}</span>")
            self.text_topics.setHtml("<br>".join(lines))
        threading.Thread(target=_do, daemon=True).start()

    # ── Menu handlers ─────────────────────────────────────────────────────────
    def _launch_bringup(self):
        os.system("ros2 launch amr_bringup amr_launch.py &")
        self.log("[LAUNCH] amr_launch.py started", "INFO")

    def _kill_nodes(self):
        reply = QMessageBox.question(self, "Kill Nodes",
            "Kill all ROS2 nodes?",
            QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            os.system("pkill -f ros2")
            self.log("[SYSTEM] All ROS2 nodes killed", "WARN")

    def _save_config(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Config", "amr_config.ini", "INI (*.ini)")
        if path:
            with open(path, "w") as f:
                f.write(f"[serial]\nport={self.combo_serial_port.currentText()}\n"
                        f"baudrate={self.combo_baudrate.currentText()}\n"
                        f"[nav]\nmax_linear={self.spin_max_linear.value()}\n"
                        f"max_angular={self.spin_max_angular.value()}\n")
            self.log(f"[CONFIG] Saved: {path}", "INFO")

    def _open_config(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Config", "", "INI (*.ini)")
        if path:
            self.log(f"[CONFIG] Loaded: {path}", "INFO")

    def _show_about(self):
        QMessageBox.about(self, "About AMR Control Panel",
            "<b>AMR ROS2 Control Panel</b><br>"
            "Version 1.0<br><br>"
            "Qt5 UI for AMR with ROS2 Jazzy<br>"
            "Nav2 · SLAM Toolbox · RPLidar<br><br>"
            "Serial Bridge: amr_serial_bridge.py")

    # ── Utilities ─────────────────────────────────────────────────────────────
    def _update_ros_status(self, connected: bool):
        if connected:
            self.lbl_ros_dot.setStyleSheet(
                "background-color:#238636;border-radius:5px;"
                "min-width:10px;max-width:10px;min-height:10px;max-height:10px;")
            self.lbl_ros_status.setText("ROS2: Connected")
            self.lbl_ros_status.setStyleSheet("color:#7ee787;font-size:10px;font-weight:bold;")
        else:
            self.lbl_ros_dot.setStyleSheet(
                "background-color:#f59e0b;border-radius:5px;"
                "min-width:10px;max-width:10px;min-height:10px;max-height:10px;")
            self.lbl_ros_status.setText("ROS2: Demo Mode")
            self.lbl_ros_status.setStyleSheet("color:#fbbf24;font-size:10px;font-weight:bold;")

    def _update_clock(self):
        self.lbl_time.setText(datetime.now().strftime("%H:%M:%S"))

    def log(self, msg: str, level: str = "INFO"):
        colors = {
            "INFO":  "#7ee787",
            "WARN":  "#fbbf24",
            "ERROR": "#f87171",
            "CMD":   "#a78bfa",
        }
        color = colors.get(level, "#e6edf3")
        ts = datetime.now().strftime("%H:%M:%S")
        html = (f"<span style='color:#8b949e'>[{ts}]</span> "
                f"<span style='color:{color}'>{msg}</span>")
        self.text_console.append(html)
        # Auto-scroll
        sb = self.text_console.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def closeEvent(self, event):
        self._stop_drive()
        if ROS2_AVAILABLE and self.ros2_node:
            # Jazzy: shutdown executor trước
            if hasattr(self, '_executor'):
                self._executor.shutdown(timeout_sec=1.0)
            self.ros2_node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass
        event.accept()


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════
def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("AMR Control Panel")
    app.setStyle("Fusion")

    # Dark palette fallback khi không có stylesheet
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor("#0d1117"))
    palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#e6edf3"))
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor("#161b22"))
    palette.setColor(QtGui.QPalette.Text, QtGui.QColor("#e6edf3"))
    app.setPalette(palette)

    window = AMRMainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
