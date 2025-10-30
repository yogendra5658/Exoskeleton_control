import serial
import csv
from datetime import datetime
from pynput import keyboard
import threading
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

# ------------------------------
# CONFIG
# ------------------------------
SERIAL_PORT = "/dev/ttyACM0"  # Or "COM3" on Windows
BAUD_RATE = 1000000
CSV_FLUSH_EVERY = 50      # flush every N rows
CALIB_DURATION = 5        # seconds for calibration

# ------------------------------
# GLOBAL VARIABLES
# ------------------------------
logging_active = False 
current_marker = ""
row_counter = 0

# ------------------------------
# KEYBOARD LISTENER
# ------------------------------
def on_press(key):
    """Capture key presses for task markers."""
    global current_marker
    try:
        k = key.char.upper()
        if k in ["1", "2", "3", "4", "5"]:
            timestamp = datetime.now().strftime("%d-%m-%Y %H:%M:%S.%f")[:-3]
            current_marker = f"T{k}_{timestamp}"
            print(f"\n>>> Marker {current_marker} added <<<\n")
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press)
listener.start()

# ------------------------------
# START COMMAND
# ------------------------------
cmd = input("Type 'start' to begin calibration and logging: ").strip().lower()
if cmd != "start":
    print("Logging aborted.")
    listener.stop()
    exit()

# ------------------------------
# FILE SETUP AND MAIN LOGIC
# ------------------------------
start_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"imu_semg_log_{start_time_str}.csv"

try:
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)

        # ############################################################### #
        # ###################### CALIBRATION PHASE ###################### #
        # ############################################################### #
        print(f"\nCalibrating... Keep still & relax muscle for {CALIB_DURATION} seconds.")
        acc_samples, gyro_samples, quat_samples = [], [], []
        semg_samples = []

        start_time_cal = time.time()
        while time.time() - start_time_cal < CALIB_DURATION:
            try:
                line = ser.readline().decode("utf-8").strip()
                parts = line.split(",")
                if len(parts) >= 17:  # expecting semg_raw + semg_filtered
                    quat_vals = [float(p) for p in parts[5:9]]
                    acc_vals = [float(p) for p in parts[9:12]]
                    gyro_vals = [float(p) for p in parts[12:15]]
                    semg_raw, semg_filtered = float(parts[15]), float(parts[16])

                    quat_samples.append(quat_vals)
                    acc_samples.append(acc_vals)
                    gyro_samples.append(gyro_vals)
                    semg_samples.append(semg_filtered)
            except (ValueError, IndexError):
                continue

        if not acc_samples or not gyro_samples or not quat_samples or not semg_samples:
            print("Calibration failed: Not enough data received. Exiting.")
            exit()

        # ---- IMU calibration ----
        acc_offset = np.mean(acc_samples, axis=0)
        gyro_offset = np.mean(gyro_samples, axis=0)
        quat_avg = np.mean(quat_samples, axis=0)
        quat_avg /= np.linalg.norm(quat_avg)
        q_ref_scipy = R.from_quat([quat_avg[1], quat_avg[2], quat_avg[3], quat_avg[0]])

        # ---- sEMG baseline ----
        semg_baseline = np.mean(np.square(semg_samples)) + 50  # margin
        print("\nCalibration done.")
        print(f"  Acc offset = {acc_offset}")
        print(f"  Gyro offset = {gyro_offset}")
        print(f"  Reference Quaternion (wxyz) = {quat_avg}")
        print(f"  sEMG baseline (envelope) = {semg_baseline}\n")
        ser.reset_input_buffer()

        # ############################################################### #
        # ###################### LOGGING PHASE ########################## #
        # ############################################################### #
        logging_active = True
        print("Logging started. Press Ctrl+C to stop.")

        # CSV header
        writer.writerow([
            "pc_timestamp", "imu_time_us", "semg_time_us",
            "roll_raw", "pitch_raw", "yaw_raw",
            "qw", "qx", "qy", "qz",
            "ax", "ay", "az",
            "gx", "gy", "gz",
            "roll_cal", "pitch_cal", "yaw_cal",
            "ax_cal", "ay_cal", "az_cal",
            "gx_cal", "gy_cal", "gz_cal",
            "semg_raw", "semg_filtered", "semg_envelope",
            "marker"
        ])

        while logging_active:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            values = line.split(",")
            if len(values) < 17:
                continue

            pc_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            try:
                # --- IMU calibration ---
                qw, qx, qy, qz = [float(v) for v in values[5:9]]
                q_current_scipy = R.from_quat([qx, qy, qz, qw])
                q_relative = q_current_scipy * q_ref_scipy.inv()
                roll_cal, pitch_cal, yaw_cal = q_relative.as_euler('xyz', degrees=True)

                ax, ay, az = [float(v) for v in values[9:12]]
                gx, gy, gz = [float(v) for v in values[12:15]]
                ax_cal, ay_cal, az_cal = [a - o for a, o in zip([ax, ay, az], acc_offset)]
                gx_cal, gy_cal, gz_cal = [g - o for g, o in zip([gx, gy, gz], gyro_offset)]

                # --- sEMG calibration ---
                semg_raw = float(values[15])
                semg_filtered = float(values[16])
                semg_envelope = semg_filtered ** 2
                if semg_envelope < semg_baseline:
                    semg_envelope = 0

                # marker
                marker = current_marker
                current_marker = ""

                # Write row
                output_row = (
                    [pc_time] + values[:15] +
                    [f"{v:.6f}" for v in [roll_cal, pitch_cal, yaw_cal]] +
                    [f"{v:.6f}" for v in [ax_cal, ay_cal, az_cal]] +
                    [f"{v:.6f}" for v in [gx_cal, gy_cal, gz_cal]] +
                    [semg_raw, semg_filtered, semg_envelope] +
                    [marker]
                )
                writer.writerow(output_row)
                row_counter += 1

                if row_counter % CSV_FLUSH_EVERY == 0:
                    file.flush()

            except (ValueError, IndexError):
                continue

except serial.SerialException:
    print(f"Could not open port {SERIAL_PORT}. Is the device connected and not in use?")
except KeyboardInterrupt:
    print("\nLogging stopped by user.")
finally:
    listener.stop()
    print(f"\nData saved to {filename}")

