#!/usr/bin/env python3
"""
serial_joint_bridge.py — v0.1.3
====================================
ROS 2 node that bridges an Arduino‑streamed joint status feed to standard ROS topics.

Changes in **0.1.3**
-------------------
* **Command prefix**: outgoing joint commands now start with `MOVE ` to match your Arduino firmware (`MOVE J1:… J2:…`).
* Added optional parameter `command_prefix` (default `"MOVE"`) so you can change it without touching code.

Traits
------
* Publishes `/joint_states` (`sensor_msgs/JointState`).
* Subscribes `/joint_commands` (same type), converts to `"MOVE J1:10.00 …"` strings, and writes to serial.
* Services `/home` & `/stop` (`std_srvs/Trigger`).
"""

from __future__ import annotations

import re
import threading
import time
from typing import List
from math import pi

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

_SERIAL_TIMEOUT_S = 0.05  # non‑blocking, 50 ms poll period
RAD2DEG = 180.0 / pi
DEG2RAD = pi / 180.0


class SerialJointBridge(Node):
    """Bridge an Arduino serial stream to ROS 2."""

    def __init__(self) -> None:  # noqa: D401
        super().__init__("serial_joint_bridge")

        # ---------------- Parameters ----------------------------------------------
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("command_prefix", "MOVE")
        self.declare_parameter("read_rate_hz", 50.0)

        self._port: str = self.get_parameter("port").value
        self._baudrate: int = int(self.get_parameter("baudrate").value)
        self._command_prefix: str = self.get_parameter("command_prefix").value
        self._joint_names: List[str] = [
            "revolute_1",
            "revolute_2",
            "revolute_3",
            "revolute_4",
            "revolute_5",
            "revolute_6",
            "prismatic_1",
            "prismatic_2",
        ]
        self._read_period_s: float = 1.0 / float(self.get_parameter("read_rate_hz").value)

        # ---------------- Serial setup --------------------------------------------
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=_SERIAL_TIMEOUT_S,
            )
            self.get_logger().info(
                f"Opened serial port {self._port} @ {self._baudrate} baud (timeout {_SERIAL_TIMEOUT_S}s)"
            )
            self._serial.reset_input_buffer()
        except serial.SerialException as exc:
            self.get_logger().fatal(f"Failed to open serial port {self._port}: {exc}")
            raise SystemExit from exc

        # ---------------- ROS entities --------------------------------------------
        self._js_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(JointState, "joint_commands", self._command_callback, 10)
        self.create_service(Trigger, "home", self._home_cb)
        self.create_service(Trigger, "stop", self._stop_cb)

        # ---------------- Internals -----------------------------------------------
        self._regex = re.compile(r"Js(\d+):([+-]?[0-9]*\.?[0-9]+)")
        self._running = True
        self._reader = threading.Thread(target=self._serial_reader, daemon=True)
        self._reader.start()

        self.get_logger().info("SerialJointBridge initialised ✔︎ (v0.1.3)")

    # ---------------------------------------------------------------------------
    # Background serial reader
    # ---------------------------------------------------------------------------

    def _serial_reader(self) -> None:
        while rclpy.ok() and self._running:
            try:
                raw_line = self._serial.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f"Serial read error: {exc}")
                time.sleep(self._read_period_s)
                continue

            if not raw_line:
                time.sleep(self._read_period_s)
                continue

            line = raw_line.decode(errors="ignore").strip()
            self.get_logger().debug(f"Raw serial: {line}")

            matches = self._regex.findall(line)
            if not matches:
                time.sleep(self._read_period_s)
                continue

            matches.sort(key=lambda tpl: int(tpl[0]))
            positions = [float(val)*DEG2RAD for _, val in matches]
            names = self._joint_names

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions + [0.0,0.0]
            self._js_pub.publish(msg)
            self.get_logger().debug(f"Published JointState: {positions}")

            time.sleep(self._read_period_s)

    # ---------------------------------------------------------------------------
    # Topic: /joint_commands
    # ---------------------------------------------------------------------------

    def _command_callback(self, msg: JointState) -> None:
        if not msg.position:
            self.get_logger().warn("Received empty JointState on /joint_commands — ignored")
            return

        cmd_body = " ".join(
            f"J{idx + 1}:{(pos*RAD2DEG):.2f}" for idx, pos in enumerate(msg.position)
        )
        cmd = f"{self._command_prefix} {cmd_body}\n"
        self._write_serial(cmd)

    # ---------------------------------------------------------------------------
    # Services
    # ---------------------------------------------------------------------------

    def _home_cb(self, _req: Trigger.Request, res: Trigger.Response):
        res.success = self._write_serial("HOME\n")
        res.message = "HOME command sent" if res.success else "Failed to send HOME"
        return res

    def _stop_cb(self, _req: Trigger.Request, res: Trigger.Response):
        res.success = self._write_serial("STOP\n")
        res.message = "STOP command sent" if res.success else "Failed to send STOP"
        return res

    # ---------------------------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------------------------

    def _write_serial(self, cmd: str) -> bool:
        try:
            self._serial.write(cmd.encode())
            self.get_logger().debug(f"Sent serial: {cmd.strip()}")
            return True
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write error: {exc}")
            return False

    # ---------------------------------------------------------------------------
    # Shutdown
    # ---------------------------------------------------------------------------

    def destroy_node(self) -> None:  # type: ignore[override]
        self._running = False
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if self._serial.is_open:
            self._serial.close()
        super().destroy_node()


# =============================================================================
# main()
# =============================================================================

def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SerialJointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
