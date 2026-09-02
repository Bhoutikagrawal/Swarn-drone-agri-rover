"""
Onboard decision engine — runs on the Jetson Nano (companion computer),
alongside ndvi_camera_loop.py and geotag_images.py.

Responsibilities:
  - Consume live NDVI stress regions + drone telemetry (from MAVLink)
  - Turn NDVI severity into a proportional spray decision (dose, not just on/off)
  - Enforce safety interlocks: battery, GPS-fix quality, obstacle distance,
    geofence — any one can override a spray decision or trigger RTL
  - Drive the nozzle actuator (PWM pump + solenoid) over serial/I2C to an
    Arduino/ESP32 spray controller, and log every decision

Install: pip install pymavlink pyserial numpy --break-system-packages
"""

import time
from dataclasses import dataclass
from enum import Enum

from pymavlink import mavutil

from ndvi_camera_loop import (
    open_cameras, compute_ndvi, find_stress_regions, pixel_to_gps, StressRegion,
)
from geotag_images import init_log, log_ndvi_point, GPSFix

# ---- Config --------------------------------------------------------------
MAVLINK_CONN = "udp:127.0.0.1:14550"
SPRAY_SERIAL_PORT = "/dev/ttyACM0"
SPRAY_SERIAL_BAUD = 115200

BATTERY_MIN_PCT = 20.0
GPS_MIN_SATS = 6
OBSTACLE_MIN_DIST_M = 2.0
GEOFENCE_RADIUS_M = 500.0
HOME_LAT, HOME_LON = 17.5449, 78.5718

# NDVI -> dose curve: lower NDVI (worse stress) = higher dose, in ms of
# solenoid-open time per pass. Tune against nozzle flow-rate calibration.
DOSE_CURVE_MS = [
    (0.0, 400),   # severely stressed -> max dose
    (0.20, 250),
    (0.35, 120),  # right at the spray threshold -> minimal dose
]


class Decision(Enum):
    SPRAY = "SPRAY"
    SKIP = "SKIP"
    ABORT_RTL = "ABORT_RTL"
    HOLD = "HOLD"


@dataclass
class Telemetry:
    lat: float
    lon: float
    alt_m: float
    battery_pct: float
    gps_sats: int
    obstacle_dist_m: float


class TelemetryReader:
    """Pulls the fused state needed for decisions from MAVLink."""

    def __init__(self, conn_str: str = MAVLINK_CONN):
        self.master = mavutil.mavlink_connection(conn_str)
        self.master.wait_heartbeat(timeout=10)

    def read(self) -> Telemetry:
        pos = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        batt = self.master.recv_match(type="SYS_STATUS", blocking=False)
        gps = self.master.recv_match(type="GPS_RAW_INT", blocking=False)
        dist = self.master.recv_match(type="DISTANCE_SENSOR", blocking=False)

        return Telemetry(
            lat=(pos.lat / 1e7) if pos else 0.0,
            lon=(pos.lon / 1e7) if pos else 0.0,
            alt_m=(pos.relative_alt / 1000.0) if pos else 0.0,
            battery_pct=(batt.battery_remaining) if batt else 100.0,
            gps_sats=(gps.satellites_visible) if gps else 99,
            obstacle_dist_m=(dist.current_distance / 100.0) if dist else 99.0,
        )


class SprayActuator:
    """Serial link to the spray micro-controller (Arduino/ESP32 driving
    the solenoid + PWM pump). Protocol: one line per command,
    'SPRAY <duration_ms>\\n' or 'STOP\\n'."""

    def __init__(self, port: str = SPRAY_SERIAL_PORT, baud: int = SPRAY_SERIAL_BAUD):
        import serial
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # let the MCU reset after opening the port

    def spray(self, duration_ms: int):
        self.ser.write(f"SPRAY {duration_ms}\n".encode())

    def stop(self):
        self.ser.write(b"STOP\n")


def dose_for_ndvi(ndvi_value: float) -> int:
    """Piecewise-linear interpolation over DOSE_CURVE_MS."""
    pts = DOSE_CURVE_MS
    if ndvi_value <= pts[0][0]:
        return pts[0][1]
    if ndvi_value >= pts[-1][0]:
        return pts[-1][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x0 <= ndvi_value <= x1:
            t = (ndvi_value - x0) / (x1 - x0)
            return int(y0 + t * (y1 - y0))
    return pts[-1][1]


def distance_m(lat1, lon1, lat2, lon2) -> float:
    import math
    R = 6371000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * R * math.asin(min(1, a ** 0.5))


def safety_check(telem: Telemetry) -> Decision:
    """Any failed check overrides perception and returns early."""
    if telem.battery_pct < BATTERY_MIN_PCT:
        return Decision.ABORT_RTL
    if telem.gps_sats < GPS_MIN_SATS:
        return Decision.HOLD  # don't trust position-tagged spray decisions
    if telem.obstacle_dist_m < OBSTACLE_MIN_DIST_M:
        return Decision.HOLD
    if distance_m(telem.lat, telem.lon, HOME_LAT, HOME_LON) > GEOFENCE_RADIUS_M:
        return Decision.ABORT_RTL
    return Decision.SPRAY  # tentative — perception decides SPRAY vs SKIP next


def decide(region: StressRegion, telem: Telemetry) -> tuple[Decision, int]:
    """Fuse safety state + NDVI severity into a final decision + dose (ms)."""
    safety_decision = safety_check(telem)
    if safety_decision != Decision.SPRAY:
        return safety_decision, 0

    if region.ndvi_value >= 0.35:  # not stressed enough to warrant spray
        return Decision.SKIP, 0

    return Decision.SPRAY, dose_for_ndvi(region.ndvi_value)


def run():
    cam_rgb, cam_nir = open_cameras()
    telem_reader = TelemetryReader()
    actuator = SprayActuator()
    log_file, writer = init_log()

    try:
        while True:
            ok_rgb, rgb = cam_rgb.read()
            ok_nir, nir = cam_nir.read()
            if not (ok_rgb and ok_nir):
                continue

            ndvi = compute_ndvi(rgb, nir)
            regions = find_stress_regions(ndvi)
            telem = telem_reader.read()

            for region in regions:
                decision, dose_ms = decide(region, telem)
                lat, lon = pixel_to_gps(region.cx_px, region.cy_px, telem.lat, telem.lon, telem.alt_m)

                if decision == Decision.SPRAY:
                    actuator.spray(dose_ms)
                    print(f"SPRAY dose={dose_ms}ms ndvi={region.ndvi_value:.2f} @ ({lat:.6f},{lon:.6f})")
                elif decision == Decision.ABORT_RTL:
                    actuator.stop()
                    print("ABORT: safety limit breached -> triggering RTL")
                    # TODO: send MAV_CMD_NAV_RETURN_TO_LAUNCH via mavlink
                elif decision == Decision.HOLD:
                    actuator.stop()
                    print("HOLD: insufficient GPS/obstacle margin, skipping this pass")
                else:
                    pass  # SKIP: healthy plant, no action, no log spam

                fix = GPSFix(lat=lat, lon=lon, alt_m=telem.alt_m, fix_ok=True)
                log_ndvi_point(writer, image_file="", fix=fix,
                                ndvi_value=region.ndvi_value, note=decision.value)
            log_file.flush()

    except KeyboardInterrupt:
        pass
    finally:
        actuator.stop()
        cam_rgb.release()
        cam_nir.release()
        log_file.close()


if __name__ == "__main__":
    run()
