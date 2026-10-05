"""Simulation-only GCS presence; never issues mode, arm or control commands."""

import signal
import time

from pymavlink import mavutil


def main() -> None:
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    connection = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=255)
    try:
        while running:
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            time.sleep(1)
    finally:
        connection.close()


if __name__ == "__main__":
    main()
