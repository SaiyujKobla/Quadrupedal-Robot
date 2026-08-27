import math
import time

import serial
from vpython import box, color, label, rate, scene, vector

SERIAL_PORT = "/dev/cu.usbserial-0001"
BAUD_RATE = 115200

scene.title = "LSM9DS1 Accelerometer Angle Visualizer"
scene.width = 900
scene.height = 700
scene.background = color.black

board = box(
    pos=vector(0, 0, 0),
    length=4,
    height=0.25,
    width=2,
    color=color.cyan,
)

front_marker = box(
    pos=vector(2.2, 0, 0),
    length=0.4,
    height=0.35,
    width=0.7,
    color=color.red,
)

text = label(
    pos=vector(0, -2.2, 0),
    text="Waiting for IMU...",
    height=16,
    box=False,
    color=color.white,
)


def rotate_vector(v, roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)

    cp = math.cos(pitch)
    sp = math.sin(pitch)

    cy = math.cos(yaw)
    sy = math.sin(yaw)

    x = v.x
    y = v.y
    z = v.z

    new_y = y * cr - z * sr
    new_z = y * sr + z * cr

    y = new_y
    z = new_z

    new_x = x * cp + z * sp
    new_z = -x * sp + z * cp

    x = new_x
    z = new_z

    new_x = x * cy - y * sy
    new_y = x * sy + y * cy

    x = new_x
    y = new_y

    return vector(x, y, z)


print(f"Opening serial port: {SERIAL_PORT}")

try:
    ser = serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        timeout=1,
    )
except serial.SerialException as error:
    print("Could not open serial port.")
    print(error)
    raise SystemExit(1)

time.sleep(2)
ser.reset_input_buffer()

print("Listening for IMU data...")

try:
    while True:
        rate(60)

        try:
            line = (
                ser.readline()
                .decode(errors="ignore")
                .strip()
            )
        except Exception as error:
            print("Serial read error:", error)
            continue

        if not line:
            continue

        print("RAW:", line)

        lower_line = line.lower()

        if lower_line.startswith("roll"):
            continue

        if lower_line.startswith("error"):
            print("Arduino reported an IMU error.")
            continue

        parts = line.split(",")

        if len(parts) != 9:
            continue

        try:
            roll_deg = float(parts[0])
            pitch_deg = float(parts[1])
            yaw_deg = float(parts[2])

            ax = float(parts[3])
            ay = float(parts[4])
            az = float(parts[5])

            roll_rate_deg_s = float(parts[6])
            pitch_rate_deg_s = float(parts[7])
            yaw_rate_deg_s = float(parts[8])

        except ValueError:
            continue

        roll = math.radians(roll_deg)

        # Intentional orientation swap used in the older simulation
        pitch = math.radians(yaw_deg)
        yaw = math.radians(pitch_deg)

        axis = rotate_vector(
            vector(1, 0, 0),
            roll,
            pitch,
            yaw,
        )

        up = rotate_vector(
            vector(0, 1, 0),
            roll,
            pitch,
            yaw,
        )

        board.axis = axis
        board.up = up

        front_marker.pos = board.pos + axis * 2.2
        front_marker.axis = axis
        front_marker.up = up

        acceleration_magnitude = math.sqrt(
            ax * ax +
            ay * ay +
            az * az
        )

        text.text = (
            f"Roll: {roll_deg:7.2f}°\n"
            f"Pitch: {pitch_deg:7.2f}°\n"
            f"Yaw: {yaw_deg:7.2f}°\n\n"
            f"Ax: {ax:7.3f} m/s²\n"
            f"Ay: {ay:7.3f} m/s²\n"
            f"Az: {az:7.3f} m/s²\n"
            f"|A|: {acceleration_magnitude:7.3f} m/s²\n\n"
            f"Roll rate: {roll_rate_deg_s:7.2f}°/s\n"
            f"Pitch rate: {pitch_rate_deg_s:7.2f}°/s\n"
            f"Yaw rate: {yaw_rate_deg_s:7.2f}°/s"
        )

except KeyboardInterrupt:
    print("\nStopping visualizer.")

finally:
    ser.close()