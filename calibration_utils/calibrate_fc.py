#!/usr/bin/env python3
"""
IMU Calibration Tool for PX4/ArduPilot/Custom flight controllers.

This tool can:
  - Record live magnetometer or accelerometer data via MAVLink.
  - Load previously recorded data from a log file.
  - Compute hard-iron and soft-iron calibration parameters for the magnetometer.
  - Compute accelerometer offsets and scales using the six-position method.
  - Upload calibration parameters to the flight controller.

Usage examples:
  # Offline calibration from a log file
  python calib_tool.py --file imu_log.csv

  # Online magnetometer calibration, save data, and upload results
  python calib_tool.py --port /dev/ttyACM0 --baud 115200 --mag --upload

  # Online accelerometer calibration
  python calib_tool.py --port COM3 --accel
"""

import argparse
import csv
import logging
import sys
import time
from typing import Optional, Tuple, List, Any

import numpy as np
from numpy.linalg import lstsq, inv, eig
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from pymavlink import mavutil

# -----------------------------------------------------------------------------
# Constants (MAVLink commands, sensor types, etc.)
# -----------------------------------------------------------------------------
MAV_CMD_PREFLIGHT_CALIBRATION = 241
MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242
MAV_CMD_PREFLIGHT_STORAGE = 245

SENSOR_TYPE_MAG = 0
SENSOR_TYPE_GYRO = 1
SENSOR_TYPE_ACC = 2
SENSOR_TYPE_ACC_SCALE = 5
SENSOR_TYPE_MAG_SECOND = 6

# -----------------------------------------------------------------------------
# Logging setup
# -----------------------------------------------------------------------------
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

# -----------------------------------------------------------------------------
# Ellipsoid fitting functions (magnetometer)
# -----------------------------------------------------------------------------
def fit_ellipsoid(xs: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Fit a general ellipsoid to Nx3 raw magnetometer samples.

    Args:
        xs: (N,3) array of raw samples.

    Returns:
        center: (3,) hard‑iron offset.
        M: (3,3) transformation matrix such that calibrated = M @ (x - center)
            lies approximately on a unit sphere.
        radii: (3,) principal radii of the fitted ellipsoid.
        eigvecs: (3,3) eigenvectors of the ellipsoid.
        eigvals: (3,) eigenvalues.
    """
    x, y, z = xs[:, 0], xs[:, 1], xs[:, 2]

    # Design matrix: A x^2 + B y^2 + C z^2 + D xy + E xz + F yz + G x + H y + I z = 1
    D = np.column_stack((x*x, y*y, z*z, x*y, x*z, y*z, x, y, z))
    rhs = np.ones(xs.shape[0])

    params, *_ = lstsq(D, rhs, rcond=None)
    A, B, C, Dd, Ee, Ff, G, H, I = params

    # Symmetric matrix Q
    Q = np.array([[A, Dd/2.0, Ee/2.0],
                  [Dd/2.0, B, Ff/2.0],
                  [Ee/2.0, Ff/2.0, C]])

    p = np.array([G, H, I]) / 2.0          # linear term

    # Ellipsoid centre
    center = -inv(Q) @ p

    # Constant term at centre
    val = center.T @ Q @ center - 1.0

    # Normalize Q so that (x‑c)ᵀ Q_norm (x‑c) = 1
    Q_norm = Q / (-val)

    eigvals, eigvecs = eig(Q_norm)
    eigvals = np.real(eigvals)
    eigvecs = np.real(eigvecs)

    if np.any(eigvals <= 0):
        raise ValueError("Fitted ellipsoid has non‑positive eigenvalues; bad fit or insufficient data.")

    # Transformation matrix: M = sqrt(Λ) · Vᵀ
    sqrt_lambda = np.sqrt(eigvals)
    M = (np.diag(sqrt_lambda) @ eigvecs.T)

    radii = 1.0 / sqrt_lambda

    return center, M, radii, eigvecs, eigvals


def apply_calibration(x: np.ndarray, center: np.ndarray, M: np.ndarray) -> np.ndarray:
    """
    Apply magnetometer calibration.

    Args:
        x: (3,) or (N,3) raw samples.
        center: (3,) hard‑iron offset.
        M: (3,3) soft‑iron transformation.

    Returns:
        Calibrated samples of the same shape.
    """
    x = np.asarray(x)
    if x.ndim == 1:
        return M @ (x - center)
    else:
        return (M @ (x - center).T).T


# -----------------------------------------------------------------------------
# Data acquisition (online)
# -----------------------------------------------------------------------------
class MavlinkConnection:
    """Handles MAVLink connection and message reception."""

    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.mav: Optional[mavutil.mavlink_connection] = None

    def connect(self) -> None:
        logger.info(f"Connecting to {self.port} at {self.baud} baud...")
        self.mav = mavutil.mavlink_connection(self.port, baud=self.baud, source_system=1)
        self.mav.wait_heartbeat(timeout=10)
        logger.info(f"Heartbeat received (system {self.mav.target_system} component {self.mav.target_component})")

    def close(self) -> None:
        if self.mav:
            self.mav.close()

    def send_command(self, command: int, param1: float = 0, param2: float = 0,
                     param3: float = 0, param4: float = 0,
                     param5: float = 0, param6: float = 0, param7: float = 0) -> bool:
        """Send a MAVLink command long and wait for acknowledgment."""
        if not self.mav:
            raise RuntimeError("Not connected")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            command, 0, param1, param2, param3, param4, param5, param6, param7
        )
        ack = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == command and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logger.debug(f"Command {command} accepted")
            return True
        else:
            logger.error(f"Command {command} rejected (result: {getattr(ack, 'result', 'None')})")
            return False


def record_data_live(conn: MavlinkConnection, sensor: str, duration: Optional[float] = None) -> np.ndarray:
    """
    Record live IMU data and display real‑time plots.

    Args:
        conn: Active MAVLink connection.
        sensor: 'mag' or 'accel'.
        duration: If given, stop after this many seconds; otherwise run until plot window closed.

    Returns:
        (N,3) array of recorded samples.
    """
    # Start magnetometer calibration (optional)
    if sensor == 'mag':
        if not conn.send_command(MAV_CMD_PREFLIGHT_CALIBRATION, 0, 1):
            sys.exit(1)

    # Prepare CSV file for logging
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    csv_filename = f"imu_raw_{sensor}_{timestamp}.csv"
    csv_file = open(csv_filename, 'w', newline='')
    writer = csv.writer(csv_file)
    writer.writerow(["time_usec", "xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro",
                     "xmag", "ymag", "zmag", "abs_pressure", "diff_pressure",
                     "pressure_alt", "temperature"])

    data_x, data_y, data_z = [], [], []
    start_time = time.time()

    # Set up plot
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    ax_xy, ax_xz, ax_yz, ax_3d = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

    def animate(_):
        nonlocal data_x, data_y, data_z
        msg = conn.mav.recv_match(type='HIGHRES_IMU', blocking=True, timeout=1)
        if msg is None:
            return

        # Extract fields
        if sensor == 'mag':
            x, y, z = msg.xmag, msg.ymag, msg.zmag
        else:
            x, y, z = msg.xacc, msg.yacc, msg.zacc

        data_x.append(x)
        data_y.append(y)
        data_z.append(z)

        # Write to CSV
        writer.writerow([
            msg.time_usec,
            msg.xacc, msg.yacc, msg.zacc,
            msg.xgyro, msg.ygyro, msg.zgyro,
            msg.xmag, msg.ymag, msg.zmag,
            msg.abs_pressure, msg.diff_pressure, msg.pressure_alt, msg.temperature
        ])

        # Update plots
        ax_xy.clear()
        ax_xz.clear()
        ax_yz.clear()
        ax_3d.clear()

        ax_xy.plot(data_x, data_y, 'r.')
        ax_xz.plot(data_x, data_z, 'b.')
        ax_yz.plot(data_y, data_z, 'g.')

        ax_xy.set(xlabel='X', ylabel='Y', title=f'{sensor} XY')
        ax_xz.set(xlabel='X', ylabel='Z', title=f'{sensor} XZ')
        ax_yz.set(xlabel='Y', ylabel='Z', title=f'{sensor} YZ')

        for a in (ax_xy, ax_xz, ax_yz):
            a.grid(True)
            a.axis('equal')

        # Optionally 3D scatter
        if len(data_x) > 10:
            ax_3d.remove()
            ax_3d = fig.add_subplot(2, 2, 4, projection='3d')
            ax_3d.scatter(data_x, data_y, data_z, c='k', s=1)
            ax_3d.set_title('3D view')

        if duration and (time.time() - start_time) > duration:
            plt.close(fig)

    ani = animation.FuncAnimation(fig, animate, interval=50)
    plt.show()
    csv_file.close()

    return np.column_stack((data_x, data_y, data_z))


# -----------------------------------------------------------------------------
# Calibration functions
# -----------------------------------------------------------------------------
def calibrate_magnetometer(raw_data: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Perform magnetometer calibration.

    Args:
        raw_data: (N,3) raw magnetometer samples.

    Returns:
        center, M, radii, eigvecs, eigvals (see fit_ellipsoid).
    """
    # Optional: rough centering to improve numerical stability
    rough_center = (raw_data.max(axis=0) + raw_data.min(axis=0)) / 2
    data_centered = raw_data - rough_center

    center, M, radii, eigvecs, eigvals = fit_ellipsoid(data_centered)
    # Add back the rough center to obtain absolute offsets
    total_offset = rough_center + center
    return total_offset, M, radii, eigvecs, eigvals


def calibrate_accelerometer_six_position(data_six: List[Tuple[float, float, float]]) -> Tuple[np.ndarray, np.ndarray]:
    """
    Six‑position accelerometer calibration.

    Args:
        data_six: List of 6 tuples, each (x, y, z) reading for:
                  up, down, left, right, forward, backward.

    Returns:
        offsets: (3,) accelerometer biases.
        scales: (3,) scale factors.
    """
    if len(data_six) != 6:
        raise ValueError("Exactly six readings required for six‑position calibration")

    xu, xd, yl, yr, zf, zb = data_six
    offsets = np.zeros(3)
    scales = np.zeros(3)

    # X axis: up / down
    offsets[0] = (xu[0] + xd[0]) / 2
    scales[0] = (xu[0] - xd[0]) / 2

    # Y axis: left / right
    offsets[1] = (yl[1] + yr[1]) / 2
    scales[1] = (yl[1] - yr[1]) / 2

    # Z axis: forward / backward
    offsets[2] = (zf[2] + zb[2]) / 2
    scales[2] = (zf[2] - zb[2]) / 2

    return offsets, scales


def upload_to_fc(conn: MavlinkConnection, mag_offsets: Optional[np.ndarray] = None,
                 mag_transform: Optional[np.ndarray] = None,
                 acc_offsets: Optional[np.ndarray] = None,
                 acc_scales: Optional[np.ndarray] = None) -> None:
    """
    Upload calibration parameters to the flight controller.

    Note: The parameter indices follow ArduPilot/PX4 conventions.
    """
    # Magnetometer offsets
    if mag_offsets is not None:
        if not conn.send_command(MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
                                 SENSOR_TYPE_MAG, *mag_offsets):
            logger.error("Failed to set magnetometer offsets")
            return

    # Magnetometer transformation (soft‑iron) – sent as two commands (matrix rows)
    if mag_transform is not None:
        # Row 1 (param2 = 5)
        if not conn.send_command(MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
                                 SENSOR_TYPE_MAG_SECOND, *mag_transform[0]):
            logger.error("Failed to set magnetometer transform row 0")
            return
        # Rows 2‑3 (param2 = 6)
        if not conn.send_command(MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
                                 SENSOR_TYPE_ACC_SCALE, *mag_transform[1], *mag_transform[2]):
            logger.error("Failed to set magnetometer transform rows 1‑2")
            return

    # Accelerometer offsets and scales
    if acc_offsets is not None and acc_scales is not None:
        if not conn.send_command(MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
                                 SENSOR_TYPE_ACC, *acc_offsets):
            logger.error("Failed to set accelerometer offsets")
            return
        if not conn.send_command(MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
                                 SENSOR_TYPE_ACC_SCALE, *acc_scales):
            logger.error("Failed to set accelerometer scales")
            return

    # Save to flash
    if not conn.send_command(MAV_CMD_PREFLIGHT_STORAGE, 1, 1):
        logger.error("Failed to save parameters to flash")


# -----------------------------------------------------------------------------
# Data loading from file (offline)
# -----------------------------------------------------------------------------
def load_imu_from_file(filename: str) -> np.ndarray:
    """
    Load HIGHRES_IMU messages from a space‑separated log file.

    Expected format: one line per message, columns separated by spaces,
    with the 5th field being "HIGHRES_IMU" and the following 14 fields
    being the message data.

    Returns:
        (N,3) array of magnetometer samples (columns 8‑10: xmag, ymag, zmag).
    """
    data = []
    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 5 and parts[4] == "HIGHRES_IMU" and len(parts[5:]) >= 14:
                # Extract mag fields (indices 7‑9 after the 5 initial tokens)
                mag_x = float(parts[7])
                mag_y = float(parts[8])
                mag_z = float(parts[9])
                data.append([mag_x, mag_y, mag_z])
    if not data:
        raise ValueError(f"No valid HIGHRES_IMU messages found in {filename}")
    return np.array(data)


# -----------------------------------------------------------------------------
# Main entry point
# -----------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description="IMU calibration tool")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--file", help="Path to log file for offline calibration")
    group.add_argument("--port", help="Serial port for online calibration")

    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--mag", action="store_true", help="Calibrate magnetometer")
    parser.add_argument("--accel", action="store_true", help="Calibrate accelerometer (six‑position)")
    parser.add_argument("--duration", type=float, help="Recording duration in seconds (online only)")
    parser.add_argument("--upload", action="store_true", help="Upload results to flight controller")
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")

    args = parser.parse_args()

    if args.verbose:
        logger.setLevel(logging.DEBUG)

    # If both --mag and --accel are given, we'll process both (accel first, then mag)
    if not (args.mag or args.accel):
        logger.error("Please specify at least one of --mag or --accel")
        sys.exit(1)

    if args.file:
        # Offline mode
        logger.info(f"Loading data from {args.file}")
        raw_data = load_imu_from_file(args.file)
        logger.info(f"Loaded {len(raw_data)} samples")

        if args.mag:
            logger.info("Calibrating magnetometer...")
            offsets, M, radii, _, _ = calibrate_magnetometer(raw_data)
            logger.info(f"Hard‑iron offsets: {offsets}")
            logger.info(f"Soft‑iron matrix:\n{M}")
            logger.info(f"Radii: {radii}")

            # Apply calibration and plot
            cal = apply_calibration(raw_data, offsets, M)
            norms = np.linalg.norm(cal, axis=1)
            logger.info(f"Calibrated norms: mean = {norms.mean():.3f}, std = {norms.std():.3f}")

            plt.figure()
            plt.plot(cal[:, 0], cal[:, 1], 'r.', label='X‑Y')
            plt.plot(cal[:, 0], cal[:, 2], 'b.', label='X‑Z')
            plt.plot(cal[:, 1], cal[:, 2], 'g.', label='Y‑Z')
            plt.axis('equal')
            plt.grid(True)
            plt.title("Calibrated magnetometer data")
            plt.legend()
            plt.show()

        if args.accel:
            # For offline accelerometer calibration we would need the six readings.
            logger.error("Offline accelerometer calibration not implemented (requires manual input).")
            sys.exit(1)

    else:  # online mode
        conn = MavlinkConnection(args.port, args.baud)
        try:
            conn.connect()
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            sys.exit(1)

        if args.mag:
            logger.info("Recording magnetometer data. Move the sensor in all orientations.")
            raw_mag = record_data_live(conn, 'mag', args.duration)
            logger.info(f"Recorded {len(raw_mag)} magnetometer samples.")
            offsets, M, radii, _, _ = calibrate_magnetometer(raw_mag)
            logger.info(f"Hard‑iron offsets: {offsets}")
            logger.info(f"Soft‑iron matrix:\n{M}")
            logger.info(f"Radii: {radii}")

            # Plot calibrated data
            cal = apply_calibration(raw_mag, offsets, M)
            norms = np.linalg.norm(cal, axis=1)
            logger.info(f"Calibrated norms: mean = {norms.mean():.3f}, std = {norms.std():.3f}")

            plt.figure()
            plt.plot(cal[:, 0], cal[:, 1], 'r.', label='X‑Y')
            plt.plot(cal[:, 0], cal[:, 2], 'b.', label='X‑Z')
            plt.plot(cal[:, 1], cal[:, 2], 'g.', label='Y‑Z')
            plt.axis('equal')
            plt.grid(True)
            plt.title("Calibrated magnetometer data")
            plt.legend()
            plt.show()

            if args.upload:
                logger.info("Uploading magnetometer calibration to flight controller...")
                upload_to_fc(conn, mag_offsets=offsets, mag_transform=M)

        if args.accel:
            # Six‑position calibration requires user interaction
            logger.info("Six‑position accelerometer calibration.")
            logger.info("Place the sensor in the following orientations and press Enter after each.")
            orientations = ["X up", "X down", "Y left", "Y right", "Z forward", "Z backward"]
            readings = []
            for orient in orientations:
                input(f"Position: {orient} → Press Enter when ready...")
                # Take a few samples and average
                samples = []
                for _ in range(50):
                    msg = conn.mav.recv_match(type='HIGHRES_IMU', blocking=True, timeout=1)
                    if msg is not None:
                        samples.append((msg.xacc, msg.yacc, msg.zacc))
                    time.sleep(0.01)
                avg = np.mean(samples, axis=0)
                readings.append(avg)
                logger.info(f"Average for {orient}: {avg}")

            offsets, scales = calibrate_accelerometer_six_position(readings)
            logger.info(f"Accelerometer offsets: {offsets}")
            logger.info(f"Accelerometer scales: {scales}")

            if args.upload:
                logger.info("Uploading accelerometer calibration...")
                upload_to_fc(conn, acc_offsets=offsets, acc_scales=scales)

        conn.close()


if __name__ == "__main__":
    main()
