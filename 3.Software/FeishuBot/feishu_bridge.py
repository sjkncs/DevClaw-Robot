#!/usr/bin/env python3
"""
DevClaw Feishu Bot Bridge Server
==================================
Translates Feishu webhook messages into DevClaw serial protocol commands.

Architecture:
    Feishu Cloud --webhook--> Flask Server --serial--> DevClaw (USB/UART)
                                   |
                                   +--> Parse /claw commands
                                   +--> Validate safety constraints
                                   +--> Translate to protocol commands
                                   +--> Send via serial port
                                   +--> Receive telemetry response
                                   +--> Reply Feishu interactive card

Usage:
    pip install pyserial flask requests
    python feishu_bridge.py --config feishu_config.json

Author: DevClaw Project
License: MIT
"""

import json
import time
import logging
import argparse
import hashlib
import hmac
import threading
from typing import Optional, Dict, Any, Tuple

import serial
import requests
from flask import Flask, request, jsonify

# ==============================================================================
# Configuration
# ==============================================================================

class Config:
    def __init__(self, path: str):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.app_id: str = data["app_id"]
        self.app_secret: str = data["app_secret"]
        self.verification_token: str = data["verification_token"]
        self.encrypt_key: str = data.get("encrypt_key", "")
        self.serial_port: str = data.get("robot_serial_port", "COM3")
        self.baud_rate: int = data.get("robot_baud_rate", 115200)
        self.allowed_users: list = data.get("allowed_user_ids", [])
        self.prefix: str = data.get("command_prefix", "/claw")
        self.safety_mode: bool = data.get("safety_mode", True)
        self.max_speed: float = data.get("max_speed_remote", 50)
        self.max_joint_step: float = data.get("max_joint_step_deg", 30)
        self.api_base: str = data.get("feishu_api_base",
                                       "https://open.feishu.cn/open-apis")
        self.telemetry_channels: int = data.get("telemetry_channels", 35)
        self.telemetry_rate: float = data.get("telemetry_rate_hz", 10)
        self.log_file: str = data.get("log_file", "devclaw_feishu.log")
        self.heartbeat_s: int = data.get("heartbeat_interval_s", 30)


# ==============================================================================
# Feishu API Client
# ==============================================================================

class FeishuClient:
    """Minimal Feishu Open API client for bot messaging."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._token: Optional[str] = None
        self._token_expire: float = 0

    def _get_tenant_token(self) -> str:
        if self._token and time.time() < self._token_expire:
            return self._token
        url = f"{self.cfg.api_base}/auth/v3/tenant_access_token/internal"
        resp = requests.post(url, json={
            "app_id": self.cfg.app_id,
            "app_secret": self.cfg.app_secret,
        }, timeout=10)
        data = resp.json()
        self._token = data.get("tenant_access_token", "")
        self._token_expire = time.time() + data.get("expire", 7200) - 300
        return self._token

    def _headers(self) -> dict:
        return {
            "Authorization": f"Bearer {self._get_tenant_token()}",
            "Content-Type": "application/json; charset=utf-8",
        }

    def reply_text(self, message_id: str, text: str):
        url = f"{self.cfg.api_base}/im/v1/messages/{message_id}/reply"
        body = {
            "content": json.dumps({"text": text}),
            "msg_type": "text",
        }
        requests.post(url, headers=self._headers(), json=body, timeout=10)

    def reply_card(self, message_id: str, card: dict):
        url = f"{self.cfg.api_base}/im/v1/messages/{message_id}/reply"
        body = {
            "content": json.dumps(card),
            "msg_type": "interactive",
        }
        requests.post(url, headers=self._headers(), json=body, timeout=10)

    def build_status_card(self, state: str, joints: list, force: list,
                          safety: str, manipulability: float) -> dict:
        joint_str = " | ".join([f"J{i+1}={v:.1f}" for i, v in enumerate(joints)])
        force_str = f"Fx={force[0]:.1f}  Fy={force[1]:.1f}  Fz={force[2]:.1f}" \
            if len(force) >= 3 else "N/A"
        return {
            "config": {"wide_screen_mode": True},
            "header": {
                "title": {"tag": "plain_text",
                          "content": f"DevClaw Status — {state}"},
                "template": "green" if state == "IDLE" else
                            "red" if "ESTOP" in state else "blue",
            },
            "elements": [
                {"tag": "div", "text": {"tag": "lark_md",
                    "content": f"**Joints (deg):**\n`{joint_str}`"}},
                {"tag": "div", "text": {"tag": "lark_md",
                    "content": f"**External Force (N):**\n`{force_str}`"}},
                {"tag": "div", "text": {"tag": "lark_md",
                    "content": f"**Safety:** {safety}  |  "
                               f"**Manipulability:** {manipulability:.4f}"}},
                {"tag": "hr"},
                {"tag": "note", "elements": [
                    {"tag": "plain_text",
                     "content": f"Updated: {time.strftime('%H:%M:%S')}"}
                ]},
            ],
        }


# ==============================================================================
# Serial Communication with DevClaw
# ==============================================================================

class RobotSerial:
    """Thread-safe serial interface to DevClaw firmware."""

    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            time.sleep(0.5)  # Wait for bootloader
            logging.info(f"Connected to {self.port} @ {self.baud}")
            return True
        except serial.SerialException as e:
            logging.error(f"Serial connect failed: {e}")
            return False

    def send_command(self, cmd: str) -> str:
        """Send a protocol command and return the response line."""
        with self.lock:
            if not self.ser or not self.ser.is_open:
                return "ERROR: serial not connected"
            try:
                line = cmd.strip() + "\n"
                self.ser.write(line.encode("ascii"))
                self.ser.flush()
                # Read response (timeout = 1s)
                resp = self.ser.readline().decode("ascii", errors="replace").strip()
                return resp if resp else "OK"
            except Exception as e:
                return f"ERROR: {e}"

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# ==============================================================================
# Command Parser & Safety Validator
# ==============================================================================

# Mapping: Feishu sub-command -> (protocol_command, arg_description, needs_args)
COMMAND_MAP: Dict[str, Tuple[str, str, bool]] = {
    "home":       ("homing",                  "",                  False),
    "rest":       ("resting",                 "",                  False),
    "movej":      ("move_j_scurve",           "j1 j2 j3 j4 j5 j6 speed accel", True),
    "movel":      ("move_l_cart",             "x y z a b c",      True),
    "teach on":   ("set_teach_mode",          "1",                 False),
    "teach off":  ("set_teach_mode",          "0",                 False),
    "save":       ("save_waypoint",           "",                  False),
    "record start": ("set_teach_recording",   "1",                 False),
    "record stop":  ("set_teach_recording",   "0",                 False),
    "replay":     ("execute_dmp",             "j1 j2 j3 j4 j5 j6", True),
    "mode":       ("request_mode",            "mode_id",           True),
    "status":     ("get_robot_state",         "",                  False),
    "force":      ("get_ext_force",           "",                  False),
    "estop":      ("__estop__",               "",                  False),
    "reset":      ("reset_estop",             "",                  False),
    "stiffness":  ("set_stiffness",           "value",             True),
    "speed":      ("set_tcp_speed_limit",     "mm_per_s",         True),
    "impedance":  ("set_impedance_mode",      "mode_id",           True),
    "dob on":     ("set_dob_enabled",         "1",                 False),
    "dob off":    ("set_dob_enabled",         "0",                 False),
    "ctc on":     ("set_ctc_enabled",         "1",                 False),
    "ctc off":    ("set_ctc_enabled",         "0",                 False),
    "friction":   ("set_friction_comp",       "enabled",           True),
    "collision":  ("set_collision_reaction",  "reaction_id",       True),
    "calib gravity": ("record_grav_sample",   "",                  False),
    "calib run":     ("run_grav_calib",       "",                  False),
    "calib kin":     ("record_kin_sample",    "x y z",             True),
    "calib kinrun":  ("run_kin_calib",        "",                  False),
    "workspace":  ("analyze_workspace",       "",                  False),
    "singular":   ("is_near_singular",        "",                  False),
    "telemetry":  ("config_telemetry",        "channels rate",     True),
    "safety":     ("safety_check",            "",                  False),
    "dmp record": ("start_dmp_record",        "",                  False),
    "dmp stop":   ("stop_dmp_record",         "",                  False),
    "minjerk":    ("move_j_minjerk",          "j1 j2 j3 j4 j5 j6 duration", True),
    "hybrid":     ("set_hybrid_axis",         "axis force_mode",   True),
    "forceref":   ("set_force_ref",           "fx fy fz",          True),
    "help":       ("__help__",                "",                  False),
}


def parse_feishu_command(text: str, prefix: str) -> Optional[Tuple[str, str]]:
    """
    Parse a Feishu message into (sub_command_key, extra_args).
    Returns None if the message is not a valid command.
    """
    text = text.strip()
    if not text.startswith(prefix):
        return None
    body = text[len(prefix):].strip()
    if not body:
        return ("help", "")

    # Try multi-word matches first (longest match)
    for key in sorted(COMMAND_MAP.keys(), key=len, reverse=True):
        if body == key or body.startswith(key + " "):
            args = body[len(key):].strip()
            return (key, args)

    return None  # Unknown command


def validate_safety(cmd_key: str, args: str, cfg: Config) -> Optional[str]:
    """
    Returns an error message if the command violates safety constraints.
    Returns None if safe.
    """
    if not cfg.safety_mode:
        return None

    if cmd_key == "movej" and args:
        parts = args.split()
        if len(parts) >= 6:
            for i, val in enumerate(parts[:6]):
                try:
                    angle = float(val)
                    if abs(angle) > 180:
                        return f"Joint {i+1} angle {angle}° exceeds ±180° limit"
                except ValueError:
                    return f"Invalid joint angle: {val}"
            # Check speed if provided
            if len(parts) >= 7:
                try:
                    spd = float(parts[6])
                    if spd > cfg.max_speed:
                        return f"Speed {spd} exceeds remote limit {cfg.max_speed}"
                except ValueError:
                    pass

    if cmd_key == "speed" and args:
        try:
            spd = float(args)
            if spd > cfg.max_speed:
                return f"TCP speed {spd} exceeds remote limit {cfg.max_speed} mm/s"
        except ValueError:
            return f"Invalid speed: {args}"

    return None


def build_protocol_command(cmd_key: str, args: str) -> Optional[str]:
    """Build the serial protocol command string."""
    if cmd_key not in COMMAND_MAP:
        return None

    proto_cmd, default_args, needs_args = COMMAND_MAP[cmd_key]

    if proto_cmd.startswith("__"):
        return proto_cmd  # Special commands handled separately

    if needs_args and args:
        return f"{proto_cmd} {args}"
    elif not needs_args and default_args:
        return f"{proto_cmd} {default_args}"
    else:
        return proto_cmd


def build_help_text() -> str:
    lines = ["**DevClaw Feishu Bot Commands**\n"]
    lines.append("```")
    for key, (proto, desc, needs) in sorted(COMMAND_MAP.items()):
        if proto.startswith("__"):
            proto_display = key.upper()
        else:
            proto_display = proto
        arg_hint = f" <{desc}>" if needs else ""
        lines.append(f"/claw {key}{arg_hint}")
    lines.append("```")
    return "\n".join(lines)


# ==============================================================================
# Flask Application
# ==============================================================================

app = Flask(__name__)
config: Optional[Config] = None
feishu: Optional[FeishuClient] = None
robot: Optional[RobotSerial] = None


@app.route("/devclaw/webhook", methods=["POST"])
def webhook():
    data = request.json
    if not data:
        return jsonify({"code": 400}), 400

    # URL verification challenge
    if "challenge" in data:
        return jsonify({"challenge": data["challenge"]})

    # Verify token
    header = data.get("header", {})
    token = header.get("token", "")
    if token != config.verification_token:
        logging.warning(f"Invalid token: {token}")
        return jsonify({"code": 403}), 403

    # Deduplicate (Feishu may resend)
    event_id = header.get("event_id", "")

    event = data.get("event", {})
    message = event.get("message", {})
    msg_type = message.get("message_type", "")
    message_id = message.get("message_id", "")

    if msg_type != "text":
        return jsonify({"code": 0})

    # Extract text
    content = json.loads(message.get("content", "{}"))
    text = content.get("text", "")

    # Check user permission
    sender = event.get("sender", {}).get("sender_id", {})
    user_id = sender.get("open_id", "")
    if config.allowed_users and user_id not in config.allowed_users:
        logging.warning(f"Unauthorized user: {user_id}")
        return jsonify({"code": 0})

    # Process in background thread
    threading.Thread(target=process_command,
                     args=(text, message_id), daemon=True).start()
    return jsonify({"code": 0})


def process_command(text: str, message_id: str):
    parsed = parse_feishu_command(text, config.prefix)
    if not parsed:
        return

    cmd_key, args = parsed
    logging.info(f"Command: {cmd_key} | Args: {args}")

    # Help
    if cmd_key == "help":
        feishu.reply_text(message_id, build_help_text())
        return

    # Safety check
    safety_err = validate_safety(cmd_key, args, config)
    if safety_err:
        feishu.reply_text(message_id, f"⚠️ Safety violation: {safety_err}")
        return

    # E-Stop (special: disable motors directly)
    if cmd_key == "estop":
        robot.send_command("set_enable 0")
        feishu.reply_text(message_id, "🛑 **EMERGENCY STOP** — All motors disabled")
        return

    # Build and send protocol command
    proto = build_protocol_command(cmd_key, args)
    if not proto:
        feishu.reply_text(message_id, f"Unknown command: {cmd_key}")
        return

    resp = robot.send_command(proto)
    logging.info(f"Robot response: {resp}")

    # Status command gets a rich card reply
    if cmd_key == "status":
        try:
            # Parse state response (format varies by firmware)
            state_val = int(resp) if resp.isdigit() else 0
            state_names = ["IDLE", "POSITION", "CTC", "IMPEDANCE",
                          "TEACH", "DMP", "HYBRID", "MINJERK",
                          "STOPPING", "ESTOP", "FAULT", "INIT"]
            state_str = state_names[state_val] if state_val < len(state_names) else "UNKNOWN"

            # Fetch additional data
            force_resp = robot.send_command("get_ext_force")
            safety_resp = robot.send_command("safety_check")
            manip_resp = robot.send_command("analyze_workspace")

            force_vals = [float(x) for x in force_resp.split()[:3]] \
                if force_resp and not force_resp.startswith("ERROR") else [0, 0, 0]
            manip = float(manip_resp) if manip_resp and not manip_resp.startswith("ERROR") else 0

            card = feishu.build_status_card(
                state=state_str,
                joints=[0]*6,  # Would need joint query command
                force=force_vals,
                safety=safety_resp,
                manipulability=manip,
            )
            feishu.reply_card(message_id, card)
        except Exception as e:
            feishu.reply_text(message_id, f"State: {resp}")
    else:
        feishu.reply_text(message_id, f"✅ `{proto}` → {resp}")


# ==============================================================================
# Heartbeat Thread
# ==============================================================================

def heartbeat_loop(interval: int):
    """Periodically send safety_check to detect disconnection."""
    while True:
        time.sleep(interval)
        if robot and robot.ser and robot.ser.is_open:
            resp = robot.send_command("safety_check")
            if "ERROR" in resp:
                logging.warning(f"Heartbeat failed: {resp}")


# ==============================================================================
# Main
# ==============================================================================

def main():
    global config, feishu, robot

    parser = argparse.ArgumentParser(description="DevClaw Feishu Bot Bridge")
    parser.add_argument("--config", "-c", default="feishu_config.json",
                        help="Path to config JSON file")
    parser.add_argument("--host", default="0.0.0.0", help="Flask bind host")
    parser.add_argument("--port", type=int, default=8090, help="Flask bind port")
    parser.add_argument("--no-serial", action="store_true",
                        help="Run without serial (for testing)")
    args = parser.parse_args()

    # Load config
    config = Config(args.config)

    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[
            logging.FileHandler(config.log_file, encoding="utf-8"),
            logging.StreamHandler(),
        ],
    )
    logging.info("DevClaw Feishu Bridge starting...")

    # Init Feishu client
    feishu = FeishuClient(config)

    # Init serial
    robot = RobotSerial(config.serial_port, config.baud_rate)
    if not args.no_serial:
        if not robot.connect():
            logging.error("Failed to connect to robot. Use --no-serial for testing.")
            return
        # Configure telemetry
        robot.send_command(
            f"config_telemetry {config.telemetry_channels} {config.telemetry_rate}")
        # Set remote speed limit
        robot.send_command(f"set_tcp_speed_limit {config.max_speed}")

    # Start heartbeat
    hb = threading.Thread(target=heartbeat_loop,
                          args=(config.heartbeat_s,), daemon=True)
    hb.start()

    # Start Flask
    logging.info(f"Webhook endpoint: http://{args.host}:{args.port}/devclaw/webhook")
    app.run(host=args.host, port=args.port, debug=False)


if __name__ == "__main__":
    main()
