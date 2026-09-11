#!/usr/bin/env python3
# ============================================================================
# RoboMaster EP ES-Check script
# ============================================================================
# python ES-Check.py --mode static --conn ap --proto udp --live
# python ES-Check.py --mode motion --conn ap --proto udp
# การใช้งานแบบไทย:
#   1) โหมด static: ตรวจสถานะเซนเซอร์/แบตเตอรี่/IO/ADC/พอร์ต/การเชื่อมต่อ
#      python ES-Check.py --mode static --conn ap --proto udp --live
#
#   2) โหมด motion: ทดสอบการเคลื่อนที่ พร้อมอ่านตำแหน่งและ yaw
#      python ES-Check.py --mode motion --conn ap --proto udp --x-speed 0.2
#
# คำแนะนำเมื่อทำงานปกติ:
#   - static โหมด: ดูว่าทุก module มีงานอ้างอิงได้และ sensor/battery/io/adc ได้ค่า
#   - motion โหมด: ดูว่าช่วงทางเดิน/ตำแหน่ง/yaw/ล้อไม่ drift มาก
#
# หากพบปัญหา:
#   - sensor / distance: ดู distance_sensor, TOF, sensor_adaptor, usb/wifi
#   - battery: ดู sub_battery_info, percent, adc_value, temperature, current
#   - position / yaw: ดู chassis.sub_position(), chassis.sub_attitude()
#   - wheel alignment: ดู chassis.drive_speed(), drive_wheels(), delay, wheel signals
#   - connection: ดู conn_type/proto_type, robot IP, port map
#
# Log จะถูกเก็บแยกตามคำสั่งเป็นไฟล์:
#   sta_YYYYMMDD_HHMMSS.log สำหรับ static
#   mot_YYYYMMDD_HHMMSS.log สำหรับ motion
# ============================================================================

import sys
import json
import time
import traceback
import argparse
import os
import logging
from datetime import datetime

# -----------------------------------------------------------------------------
# IMPORTANT ENVIRONMENT NOTE:
# The RoboMaster SDK repo source tree is stored in the workspace folder
# d:\Cha sahdu\github\RoboMaster-241-251\src. The correct interpreter for that
# SDK is the repo venv's Python 3.8 runtime, not a newly created venv from the
# editor or Windows Store Python. The ES-Check script should therefore import
# robomaster from the repo source tree and run only with the repo's venv.
# -----------------------------------------------------------------------------
REPO_ROOT = r"d:\Cha sahdu\github\RoboMaster-241-251"
REPO_SRC = os.path.join(REPO_ROOT, "src")
if REPO_SRC not in sys.path:
    sys.path.insert(0, REPO_SRC)

try:
    from robomaster import robot, config
except Exception as exc:
    print("=== Environment import guidance ===")
    print("Expected Python interpreter: repo venv Python 3.8.x")
    print("Expected source import root:", REPO_SRC)
    print(
        "Expected repo venv path:",
        os.path.join(REPO_ROOT, ".venv", "Scripts", "python.exe"),
    )
    print("Import error:", repr(exc))
    raise

LOG_DIR = os.path.join(os.getcwd(), "logs")


def setup_logger(mode):
    """Create a separate log file for each command mode.

    static -> sta_YYYYMMDD_HHMMSS.log
    motion -> mot_YYYYMMDD_HHMMSS.log
    """
    os.makedirs(LOG_DIR, exist_ok=True)
    tag = "sta" if mode == "static" else "mot"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(LOG_DIR, f"{tag}_{stamp}.log")

    logger = logging.getLogger(f"ESCheck_{tag}")
    logger.handlers.clear()
    logger.setLevel(logging.INFO)
    logger.propagate = False

    fh = logging.FileHandler(log_path, encoding="utf-8")
    fh.setLevel(logging.INFO)
    fh.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(message)s"))
    logger.addHandler(fh)
    logger.info("logger started; mode=%s; log=%s", mode, log_path)
    return logger, log_path


MODULES = [
    "chassis",
    "gimbal",
    "blaster",
    "camera",
    "vision",
    "battery",
    "servo",
    "sensor",
    "sensor_adaptor",
    "robotic_arm",
    "gripper",
    "armor",
    "uart",
    "ai_module",
]


def build_parser():
    p = argparse.ArgumentParser(description="RoboMaster EP inspection script")
    p.add_argument(
        "--mode",
        default="static",
        choices=["static", "motion"],
        help="static: sensor/status check only; motion: test motion, wheel/yaw/position alignment",
    )
    p.add_argument(
        "--conn",
        default="ap",
        choices=["ap", "sta", "rndis"],
        help="Connection type: ap/sta/rndis",
    )
    p.add_argument(
        "--proto", default="udp", choices=["tcp", "udp"], help="Protocol: tcp/udp"
    )
    p.add_argument("--sn", default=None, help="Robot serial number, optional")
    p.add_argument("--ip", default=None, help="Robot IP address, optional")
    p.add_argument(
        "--live",
        action="store_true",
        help="Show a single-line realtime terminal dashboard",
    )
    p.add_argument(
        "--grid-size", type=float, default=60.0, help="Grid size in mm; default 60x60"
    )
    p.add_argument(
        "--grid-center",
        type=float,
        default=30.0,
        help="Center of the grid in mm; default 30",
    )
    p.add_argument(
        "--x-speed", type=float, default=0.2, help="For motion test, x speed in m/s"
    )
    p.add_argument(
        "--y-speed", type=float, default=0.0, help="For motion test, y speed in m/s"
    )
    p.add_argument(
        "--z-speed", type=float, default=0.0, help="For motion test, yaw speed in deg/s"
    )
    return p


def connect_robot(conn_type="ap", proto_type="udp", sn=None, ip=None):
    """Create Robot object and initialize using the SDK's own API.

    Important note: the SDK exposes modules only after initialize() finishes.
    """
    r = robot.Robot()
    try:
        # The SDK method signature is:
        # Robot.initialize(conn_type=config.DEFAULT_CONN_TYPE,
        #                  proto_type=config.DEFAULT_PROTO_TYPE,
        #                  sn=None)
        # So pass through user arguments.
        r.initialize(conn_type=conn_type, proto_type=proto_type, sn=sn)
    except Exception:
        # Keep user-level output readable.
        raise
    return r


def port_summary():
    """Return the network and stream ports defined in the SDK config object.

    Values are drawn from config.py, not guessed from outside the workspace.
    """
    return {
        "ROBOT_DEVICE_PORT": config.ROBOT_DEVICE_PORT,
        "ROBOT_PROXY_PORT": config.ROBOT_PROXY_PORT,
        "ROBOT_BROADCAST_PORT": config.ROBOT_BROADCAST_PORT,
        "ROBOT_SDK_PORT_MIN": config.ROBOT_SDK_PORT_MIN,
        "ROBOT_SDK_PORT_MAX": config.ROBOT_SDK_PORT_MAX,
        "ROBOT_DEFAULT_WIFI_ADDR": list(config.ROBOT_DEFAULT_WIFI_ADDR),
        "ROBOT_DEFAULT_RNDIS_ADDR": list(config.ROBOT_DEFAULT_RNDIS_ADDR),
        "ROBOT_DEFAULT_LOCAL_WIFI_ADDR": list(config.ROBOT_DEFAULT_LOCAL_WIFI_ADDR),
        "ROBOT_DEFAULT_LOCAL_RNDIS_ADDR": list(config.ROBOT_DEFAULT_LOCAL_RNDIS_ADDR),
        "EP video_stream_port": config.ep_conf.video_stream_port,
        "EP audio_stream_port": config.ep_conf.audio_stream_port,
        "EP video_stream_addr": config.ep_conf.video_stream_addr,
        "EP audio_stream_addr": config.ep_conf.audio_stream_addr,
        "EP video_stream_proto": config.ep_conf.video_stream_proto,
        "EP audio_stream_proto": config.ep_conf.audio_stream_proto,
    }


def module_state(r):
    """Return a summary of the module class names in the robot._modules map.

    Note: this is not a full hardware test; it is an SDK-level presence check.
    """
    live_modules = {}
    for name in MODULES:
        try:
            obj = getattr(r, name)
            live_modules[name] = {
                "object_class": obj.__class__.__name__,
                "has_get_version": hasattr(obj, "get_version"),
                "has_start": hasattr(obj, "start"),
                "has_stop": hasattr(obj, "stop"),
            }
        except Exception as exc:
            live_modules[name] = {
                "object_class": None,
                "error": str(exc),
            }
    return live_modules


def sensor_status(r):
    """Try to inspect the sensor-related modules via the SDK classes.

    The sensor classes in the SDK expose subscription APIs and some direct
    get_* methods. We print what is available and try the simplest getters.
    """
    out = {}

    # Distance sensor (TOF)
    try:
        sensor = r.sensor
        out["distance_sensor"] = {
            "class": sensor.__class__.__name__,
            "sub_distance": hasattr(sensor, "sub_distance"),
            "unsub_distance": hasattr(sensor, "unsub_distance"),
        }
    except Exception as exc:
        out["distance_sensor"] = {"error": str(exc)}

    # Sensor adaptor (pinboard)/adc/io/pulse
    try:
        adaptor = r.sensor_adaptor
        out["sensor_adaptor"] = {
            "class": adaptor.__class__.__name__,
            "get_adc": hasattr(adaptor, "get_adc"),
            "get_io": hasattr(adaptor, "get_io"),
            "get_pulse_period": hasattr(adaptor, "get_pulse_period"),
            "sub_adapter": hasattr(adaptor, "sub_adapter"),
        }
    except Exception as exc:
        out["sensor_adaptor"] = {"error": str(exc)}

    # Battery module
    try:
        battery = r.battery
        out["battery"] = {
            "class": battery.__class__.__name__,
            "sub_battery_info": hasattr(battery, "sub_battery_info"),
            "unsub_battery_info": hasattr(battery, "unsub_battery_info"),
        }
    except Exception as exc:
        out["battery"] = {"error": str(exc)}

    # Armors/Gripper/Uart etc. no active values available here without real data push
    for key in ["robotic_arm", "gripper", "armor", "uart"]:
        try:
            obj = getattr(r, key)
            out[key] = {"class": obj.__class__.__name__, "available": True}
        except Exception as exc:
            out[key] = {"error": str(exc)}

    return out


def robot_software_info(r):
    """Print SDK-accessible software and hardware IDs.

    The repo's Robot class includes get_version(), get_sn(), get_robot_mode(),
    and get_sdk_version? Not on RobotBase class. Need to be defensive.
    """
    info = {}

    # Product version
    try:
        info["product_version"] = r.get_version()
    except Exception as exc:
        info["product_version"] = f"ERR: {exc}"

    # Serial number
    try:
        info["serial_number"] = r.get_sn()
    except Exception as exc:
        info["serial_number"] = f"ERR: {exc}"

    # USB/WIFI AP/STA or protocol
    try:
        info["conn_type"] = r.conn_type
        info["proto_type"] = r.proto_type
    except Exception as exc:
        info["conn_type"] = f"ERR: {exc}"
        info["proto_type"] = f"ERR: {exc}"

    # Robot IP address gets from client remote address when connected.
    try:
        info["remote_robot_ip"] = r.ip
    except Exception as exc:
        info["remote_robot_ip"] = f"ERR: {exc}"

    # Robot object and product metadata
    try:
        info["product"] = r.product
    except Exception as exc:
        info["product"] = f"ERR: {exc}"

    return info


def try_subscribe_sample_data(r):
    """Subscribe a few example sensors and show that the push subject API exists.

    This does not prove actual sensor values are streaming; it proves the API
    methods exist in the installed SDK classes.
    """
    sample = {}

    try:
        # subscribe battery percent
        # callback receives the fractional/percent payload
        def on_battery(percent):
            sample["battery_percent"] = percent

        r.battery.sub_battery_info(freq=5, callback=on_battery)
        sample["battery_subscribe"] = "ok"
    except Exception as exc:
        sample["battery_subscribe"] = f"ERR: {exc}"

    try:

        def on_distance(distances):
            sample["distance_values"] = distances

        r.sensor.sub_distance(freq=5, callback=on_distance)
        sample["tof_subscribe"] = "ok"
    except Exception as exc:
        sample["tof_subscribe"] = f"ERR: {exc}"

    try:
        # Adaptor subject decodes IO values + ADC values.
        def on_adaptor(values):
            sample["adapter_values"] = values

        r.sensor_adaptor.sub_adapter(freq=5, callback=on_adaptor)
        sample["adapter_subscribe"] = "ok"
    except Exception as exc:
        sample["adapter_subscribe"] = f"ERR: {exc}"

    return sample


def live_dashboard(r):
    """Print a compact one-line terminal dashboard on the same line using \r.

    Realtime values that come in through DDS callbacks update the same line, so
    the user sees a readable number refresh instead of stacked output.
    """
    state = {
        "battery": "NA",
        "distance": "NA",
        "io": "NA",
        "adc": "NA",
        "pos": "NA",
        "yaw": "NA",
    }

    def on_battery(percent):
        state["battery"] = str(percent)

    def on_distance(values):
        # SDK returns a list of four TOF distances, in mm.
        try:
            state["distance"] = str(list(values))
        except Exception:
            state["distance"] = str(values)

    def on_adapter(values):
        # Values come from SensorAdaptor AdapterSubject.data_info().
        try:
            io_vals, adc_vals = values
            state["io"] = str(list(io_vals))
            state["adc"] = str(list(adc_vals))
        except Exception:
            state["io"] = str(values)

    def on_position(position):
        try:
            x, y, z = position
            state["pos"] = f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
        except Exception:
            state["pos"] = str(position)

    def on_attitude(attitude):
        try:
            yaw, pitch, roll = attitude
            state["yaw"] = f"yaw={yaw:.2f}, pitch={pitch:.2f}, roll={roll:.2f}"
        except Exception:
            state["yaw"] = str(attitude)

    try:
        r.battery.sub_battery_info(freq=5, callback=on_battery)
    except Exception as exc:
        state["battery"] = f"ERR:{exc}"

    try:
        r.sensor.sub_distance(freq=5, callback=on_distance)
    except Exception as exc:
        state["distance"] = f"ERR:{exc}"

    try:
        r.sensor_adaptor.sub_adapter(freq=5, callback=on_adapter)
    except Exception as exc:
        state["io"] = f"ERR:{exc}"
        state["adc"] = f"ERR:{exc}"

    try:
        r.chassis.sub_position(cs=0, freq=5, callback=on_position)
    except Exception as exc:
        state["pos"] = f"ERR:{exc}"

    try:
        r.chassis.sub_attitude(freq=5, callback=on_attitude)
    except Exception as exc:
        state["yaw"] = f"ERR:{exc}"

    try:
        print(
            "\nRoboMaster EP LIVE | IP={0} | mode={1} | conn={2}/{3} | battery={4} | tof={5} | io={6} | adc={7} | pos={8} | attitude={9}".format(
                r.ip,
                "static",
                r.conn_type,
                r.proto_type,
                state["battery"],
                state["distance"],
                state["io"],
                state["adc"],
                state["pos"],
                state["yaw"],
            ),
            end="",
            flush=True,
        )

        while True:
            info_line = "RoboMaster EP LIVE | IP={0} | mode={1} | conn={2}/{3} | battery={4} | tof={5} | io={6} | adc={7} | pos={8} | attitude={9}".format(
                r.ip,
                "static",
                r.conn_type,
                r.proto_type,
                state["battery"],
                state["distance"],
                state["io"],
                state["adc"],
                state["pos"],
                state["yaw"],
            )
            print("\r" + info_line, end="", flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n")
        print("Dashboard stopped by user.")


def motion_test(r, args):
    """Movement test mode.

    Separate command from static inspection and show raw movement telemetry in
    a single terminal line with 60x60 grid and 30mm center reference guidance.
    """
    print("=== MOTION MODE: grid 60x60, center reference 30mm ===")
    print(f"grid_size_mm={args.grid_size} grid_center_mm={args.grid_center}")
    print(f"x_speed={args.x_speed} y_speed={args.y_speed} z_speed={args.z_speed}")

    position_value = {"value": None}
    attitude_value = {"value": None}

    def on_position(pos):
        position_value["value"] = pos

    def on_attitude(att):
        attitude_value["value"] = att

    try:
        r.chassis.sub_position(cs=0, freq=5, callback=on_position)
        r.chassis.sub_attitude(freq=5, callback=on_attitude)
    except Exception as exc:
        print("DDS subscription for motion test failed:", exc)

    try:
        print("Sending forward movement test command...")
        r.chassis.drive_speed(
            x=args.x_speed, y=args.y_speed, z=args.z_speed, timeout=1.0
        )
        time.sleep(1.2)
        print("Motion test command completed.")
    except Exception as exc:
        print("Motion test command failed:", exc)

    print("=== Motion telemetry ===")
    print("position:", position_value["value"])
    print("attitude:", attitude_value["value"])
    print(
        "Check wheel alignment, yaw drift, and absolute chassis position around grid center 30mm."
    )


def main():
    parser = build_parser()
    args = parser.parse_args()
    logger, log_path = setup_logger(args.mode)

    try:
        print("=== RoboMaster EP SDK inspection ===")
        print("SDK repo path:", REPO_SRC)
        print("Requested connection:", args.conn, args.proto)
        print("SN:", args.sn)
        print("IP:", args.ip)
        print("Log file:", log_path)

        logger.info(
            "Start inspection; mode=%s; conn=%s/%s; sn=%s; ip=%s",
            args.mode,
            args.conn,
            args.proto,
            args.sn,
            args.ip,
        )

        # Connection and port constants from SDK.
        print("=== SDK port/config map ===")
        ports = port_summary()
        for k, v in ports.items():
            print(f"{k}: {v}")
        logger.info("SDK port/config map=%s", json.dumps(ports, ensure_ascii=False))

        # Create robot and connect.
        print("=== Connection ===")
        r = connect_robot(args.conn, args.proto, args.sn, args.ip)
        print("Connected robot object:", r.__class__.__name__)
        print("Robot conn_type:", r.conn_type)
        print("Robot proto_type:", r.proto_type)
        print("Robot IP:", r.ip)
        logger.info(
            "Connected robot object=%s; conn_type=%s; proto_type=%s; robot_ip=%s",
            r.__class__.__name__,
            r.conn_type,
            r.proto_type,
            r.ip,
        )

        if args.mode == "motion":
            motion_test(r, args)
            logger.info(
                "motion_test command complete; grid_size_mm=%s; center_mm=%s; speed=(x=%s,y=%s,z=%s)",
                args.grid_size,
                args.grid_center,
                args.x_speed,
                args.y_speed,
                args.z_speed,
            )
            r.close()
            print("=== Motion test complete ===")
            logger.info("Motion test completed; log_path=%s", log_path)
            return

        if args.live:
            # One-line realtime dashboard in terminal; clears by printing CR only.
            live_dashboard(r)
            r.close()
            return

        # Quick software/hardware info.
        print("=== Robot software/hardware info ===")
        info = robot_software_info(r)
        for k, v in info.items():
            print(f"{k}: {v}")
        logger.info("Robot software/hardware=%s", json.dumps(info, ensure_ascii=False))

        # Exposed SDK modules.
        print("=== Available SDK modules ===")
        modules = module_state(r)
        for k, v in modules.items():
            print(f"{k}: {v}")
        logger.info("Available SDK modules=%s", json.dumps(modules, ensure_ascii=False))

        # Sensor / battery / adaptor object layout.
        print("=== Sensor and regulator availability ===")
        sensors = sensor_status(r)
        for k, v in sensors.items():
            print(f"{k}: {v}")
        logger.info("Sensor status=%s", json.dumps(sensors, ensure_ascii=False))

        # Example subscription points that SDS push-derived data uses.
        print("=== Example push subscribe sources ===")
        sample = try_subscribe_sample_data(r)
        for k, v in sample.items():
            print(f"{k}: {v}")
        logger.info("Sample subscribe=%s", json.dumps(sample, ensure_ascii=False))

        # Wait briefly so the subscriptions can show some callback if robot pushes data.
        time.sleep(1.0)

        # Save JSON result
        results = {
            "software_info": info,
            "modules": modules,
            "sensor_status": sensors,
            "sample_subscribe": sample,
            "ports": ports,
        }
        out_path = "ES-Check-result.json"
        with open(out_path, "w", encoding="utf-8") as fp:
            json.dump(results, fp, indent=2)
        print("JSON report saved to:", out_path)
        logger.info("JSON results=%s", out_path)

        # Gracefully stop and disconnect.
        r.close()
        print("=== Inspection complete ===")
        logger.info("Static inspection complete; log_path=%s", log_path)

    except Exception as exc:
        print("=== Error ===")
        print(type(exc).__name__, exc)
        traceback.print_exc()
        logger.exception("Unhandled exception in ES-Check")
        sys.exit(1)


if __name__ == "__main__":
    main()
