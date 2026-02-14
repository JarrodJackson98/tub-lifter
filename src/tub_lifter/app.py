"""
Tub Lifter - Web interface for controlling a Thomson Electrak HD linear actuator
via CAN bus on a Raspberry Pi with Sequent Microsystems MEGA-IND hat.
"""

import os
from flask import Flask, render_template, jsonify, request
from tub_lifter.actuator import ElectrakHD

app = Flask(__name__)

I2C_BUS = int(os.environ.get("I2C_BUS", "1"))
I2C_ADDR = int(os.environ.get("I2C_ADDR", "0x50"), 0)
actuator = ElectrakHD(i2c_bus=I2C_BUS, i2c_addr=I2C_ADDR)

# Track whether CAN is connected
can_connected = False


def ensure_connected():
    global can_connected
    if not can_connected:
        try:
            actuator.connect()
            can_connected = True
        except Exception as e:
            return str(e)
    return None


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/extend", methods=["POST"])
def extend():
    err = ensure_connected()
    if err:
        return jsonify({"ok": False, "error": f"CAN connection failed: {err}"}), 500
    speed = request.json.get("speed", 50) if request.is_json else 50
    actuator.extend(speed_pct=speed)
    return jsonify({"ok": True, "action": "extend", "speed": speed})


@app.route("/api/retract", methods=["POST"])
def retract():
    err = ensure_connected()
    if err:
        return jsonify({"ok": False, "error": f"CAN connection failed: {err}"}), 500
    speed = request.json.get("speed", 50) if request.is_json else 50
    actuator.retract(speed_pct=speed)
    return jsonify({"ok": True, "action": "retract", "speed": speed})


@app.route("/api/stop", methods=["POST"])
def stop():
    err = ensure_connected()
    if err:
        return jsonify({"ok": False, "error": f"CAN connection failed: {err}"}), 500
    actuator.stop()
    return jsonify({"ok": True, "action": "stop"})


@app.route("/api/status")
def status():
    if not can_connected:
        return jsonify({"ok": False, "error": "Not connected"})
    return jsonify({"ok": True, **actuator.get_status()})


def main():
    app.run(host="0.0.0.0", port=5052)


if __name__ == "__main__":
    main()
