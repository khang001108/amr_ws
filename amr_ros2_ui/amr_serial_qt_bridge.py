#!/usr/bin/env python3
"""
amr_serial_qt_bridge.py
Lớp bọc serial bridge dùng trong Qt UI, kết nối với firmware AMR qua UART.

Giao thức giả định từ amr_serial_bridge.py trong dự án:
  OUT: "CMD,<linear*100>,<angular*100>\n"   → gửi cmd_vel
  IN:  "ODOM,<x>,<y>,<yaw>,<vx>,<vz>\n"    → odometry
  IN:  "ENC,<left>,<right>\n"               → encoder counts
  IN:  "BAT,<voltage_mv>\n"                 → battery voltage
"""

import serial
import threading
import time
from typing import Callable, Optional

from PyQt5.QtCore import QObject, pyqtSignal


class AMRSerialBridge(QObject):
    """
    Qt-compatible serial bridge cho AMR.

    Signals:
        odom_updated(x, y, yaw, vx, vz)
        encoder_updated(left, right)
        battery_updated(voltage_mv)
        connected()
        disconnected()
        error(msg)
    """

    odom_updated    = pyqtSignal(float, float, float, float, float)
    encoder_updated = pyqtSignal(int, int)
    battery_updated = pyqtSignal(float)
    connected       = pyqtSignal()
    disconnected    = pyqtSignal()
    error           = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._serial: Optional[serial.Serial] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False

        # Trạng thái robot
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.vx  = 0.0
        self.vz  = 0.0
        self.enc_left  = 0
        self.enc_right = 0
        self.battery_mv = 0.0

    # ── Kết nối / ngắt kết nối ────────────────────────────────────────────────
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self._running = True
            self._rx_thread = threading.Thread(
                target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            self.connected.emit()
            return True
        except serial.SerialException as e:
            self.error.emit(f"Serial open failed: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self._serial and self._serial.is_open:
            self._serial.close()
        self.disconnected.emit()

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # ── Gửi lệnh ──────────────────────────────────────────────────────────────
    def send_cmd_vel(self, linear: float, angular: float):
        """Gửi lệnh vận tốc tới firmware."""
        if not self.is_connected:
            return
        lin_int = int(linear * 100)
        ang_int = int(angular * 100)
        msg = f"CMD,{lin_int},{ang_int}\n"
        try:
            self._serial.write(msg.encode())
        except serial.SerialException as e:
            self.error.emit(f"Write error: {e}")
            self.disconnect()

    def send_reset_odom(self):
        """Reset odometry về 0."""
        if not self.is_connected:
            return
        try:
            self._serial.write(b"RESET_ODOM\n")
        except serial.SerialException:
            pass

    def send_pid_params(self, kp: float, ki: float, kd: float):
        """Cấu hình PID."""
        if not self.is_connected:
            return
        msg = f"PID,{kp:.4f},{ki:.4f},{kd:.4f}\n"
        try:
            self._serial.write(msg.encode())
        except serial.SerialException:
            pass

    # ── RX loop ───────────────────────────────────────────────────────────────
    def _rx_loop(self):
        buffer = ""
        while self._running and self._serial and self._serial.is_open:
            try:
                data = self._serial.readline().decode("utf-8", errors="ignore").strip()
                if data:
                    self._parse_line(data)
            except serial.SerialException as e:
                self.error.emit(f"Read error: {e}")
                self.disconnect()
                break
            except Exception:
                pass

    def _parse_line(self, line: str):
        """Phân tích gói tin từ firmware."""
        parts = line.split(",")
        if not parts:
            return

        cmd = parts[0].upper()

        if cmd == "ODOM" and len(parts) >= 6:
            try:
                x   = float(parts[1])
                y   = float(parts[2])
                yaw = float(parts[3])
                vx  = float(parts[4])
                vz  = float(parts[5])
                self.x   = x
                self.y   = y
                self.yaw = yaw
                self.vx  = vx
                self.vz  = vz
                self.odom_updated.emit(x, y, yaw, vx, vz)
            except ValueError:
                pass

        elif cmd == "ENC" and len(parts) >= 3:
            try:
                left  = int(parts[1])
                right = int(parts[2])
                self.enc_left  = left
                self.enc_right = right
                self.encoder_updated.emit(left, right)
            except ValueError:
                pass

        elif cmd == "BAT" and len(parts) >= 2:
            try:
                mv = float(parts[1])
                self.battery_mv = mv
                self.battery_updated.emit(mv)
            except ValueError:
                pass

        elif cmd == "ERR" and len(parts) >= 2:
            self.error.emit(f"Robot error: {','.join(parts[1:])}")


# ══════════════════════════════════════════════════════════════════════════════
#  PID Tuning Dialog
# ══════════════════════════════════════════════════════════════════════════════
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QFormLayout,
                              QDoubleSpinBox, QPushButton, QLabel,
                              QDialogButtonBox, QGroupBox, QHBoxLayout)


class PIDTuningDialog(QDialog):
    """Dialog chỉnh thông số PID cho motor left/right."""

    def __init__(self, bridge: AMRSerialBridge, parent=None):
        super().__init__(parent)
        self.bridge = bridge
        self.setWindowTitle("PID Tuning")
        self.setMinimumWidth(340)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        for side in ("Left Motor", "Right Motor"):
            grp = QGroupBox(side)
            form = QFormLayout(grp)
            for param in ("Kp", "Ki", "Kd"):
                spin = QDoubleSpinBox()
                spin.setRange(0.0, 100.0)
                spin.setSingleStep(0.1)
                spin.setDecimals(4)
                spin.setValue(1.0 if param == "Kp" else 0.0)
                setattr(self, f"spin_{side[0].lower()}_{param.lower()}", spin)
                form.addRow(f"{param}:", spin)
            layout.addWidget(grp)

        btn_apply = QPushButton("Apply PID")
        btn_apply.clicked.connect(self._apply)
        layout.addWidget(btn_apply)

        btns = QDialogButtonBox(QDialogButtonBox.Close)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

    def _apply(self):
        kp = self.spin_l_kp.value()
        ki = self.spin_l_ki.value()
        kd = self.spin_l_kd.value()
        self.bridge.send_pid_params(kp, ki, kd)


# ══════════════════════════════════════════════════════════════════════════════
#  Odometry Calibration Dialog
# ══════════════════════════════════════════════════════════════════════════════
class OdomCalibDialog(QDialog):
    """Dialog hiệu chỉnh thông số odometry."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Odometry Calibration")
        self.setMinimumWidth(360)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        form = QFormLayout()

        self.spin_wheel_radius = QDoubleSpinBox()
        self.spin_wheel_radius.setRange(0.01, 0.30)
        self.spin_wheel_radius.setSingleStep(0.001)
        self.spin_wheel_radius.setDecimals(4)
        self.spin_wheel_radius.setValue(0.0325)
        form.addRow("Wheel Radius (m):", self.spin_wheel_radius)

        self.spin_wheel_base = QDoubleSpinBox()
        self.spin_wheel_base.setRange(0.1, 1.0)
        self.spin_wheel_base.setSingleStep(0.001)
        self.spin_wheel_base.setDecimals(4)
        self.spin_wheel_base.setValue(0.287)
        form.addRow("Wheel Base (m):", self.spin_wheel_base)

        self.spin_enc_resolution = QDoubleSpinBox()
        self.spin_enc_resolution.setRange(100, 10000)
        self.spin_enc_resolution.setSingleStep(1)
        self.spin_enc_resolution.setDecimals(0)
        self.spin_enc_resolution.setValue(1000)
        form.addRow("Encoder PPR:", self.spin_enc_resolution)

        layout.addLayout(form)

        # Calibration test
        grp_test = QGroupBox("Straight Line Test")
        test_layout = QHBoxLayout(grp_test)
        self.btn_drive_1m = QPushButton("Drive 1m")
        self.btn_drive_1m.clicked.connect(lambda: None)
        test_layout.addWidget(QLabel("Drive exactly 1 m forward:"))
        test_layout.addWidget(self.btn_drive_1m)
        layout.addWidget(grp_test)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

    def get_params(self):
        return {
            "wheel_radius":     self.spin_wheel_radius.value(),
            "wheel_base":       self.spin_wheel_base.value(),
            "encoder_ppr":      int(self.spin_enc_resolution.value()),
        }
