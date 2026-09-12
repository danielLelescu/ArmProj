"""
armprocessing.py

Isolates arm-related landmarks from MediaPipe Pose and computes arm joint
angles (elbow angle, shoulder angle) for both arms.

Usage:
    python armprocessing.py                             # webcam, live view
    python armprocessing.py --source video.mp4          # process a video file
    python armprocessing.py --source 0                  # source 0 selects webcam
    python armprocessing.py --output arms.csv           # specify output csv, default is src/arm_data.csv
    python armprocessing.py --full-data                 # outputs full pose data on arm/hand positions and flexion
    python armprocessing.py --use-3d                    # performs pose calculations using z estimations (not recommended)

    Caveat: left and right arms are mirror images of each other, so the
    same physical gesture (e.g. both forearms pointing up) produces
    opposite-signed angles for left vs right.

Bluetooth streaming arguments for DFRobot Firebeetle ESP32:
    python armprocessing.py --bluetooth /dev/cu.FireBeetle_ArmData
        Streams "left_elbow,right_elbow,left_shoulder,right_shoulder\n" as a
        comma-separated ASCII line for every processed frame, over a paired
        Bluetooth Classic SPP serial connection, in addition to writing the
        CSV exactly as before. Values are the same signed/unsigned angles
        used in the CSV and on-screen overlay.

    python armprocessing.py --list-ports
        Lists available serial ports (including paired Bluetooth SPP
        devices) so you can find the right path for --bluetooth, then exits.

    python armprocessing.py --bluetooth /dev/cu.FireBeetle_ArmData --bt-rate 10
        Caps Bluetooth updates to 10/sec instead of the default 20/sec, in
        case the receiving sketch is doing slow work (e.g. driving servos)
        per message and falls behind at the default rate.

Pairing procedure (macOS):
    1. Pair the ESP32 via System Settings > Bluetooth. It is normal for connection to only hold for a moment.
    2. Ensure ESP32 appears in your device list in System Settings > Bluetooth
    3. Run python3 armprocessing_BL.py --list-ports
    4. Ensure that the ESP is recognized, something like "/dev/cu.FireBeetle_ArmData" in the serial list
    5. Run python3 armprocessing_BL.py --bluetooth <YOUR-PORT>
    6. When finished, forget the ESP32 device via System Settings > Bluetooth 
    and restart Settings before attempting to reconnect. The reconnect issue is a known bug for the FireBeetle ESP32

Crash procedure (macOS):
    If a camera crash occurs, ps aux | grep armprocessing - kill -9 <PID>

Requires:
    pip install mediapipe opencv python numpy pyserial

On my device, "armpy" replaces python due to outdated VScode filepaths
"""

import argparse
import csv
import time

import cv2
import mediapipe as mp
import numpy as np
import serial
import serial.tools.list_ports

mp_pose = mp.solutions.pose

# Arm-only landmark set (source: MediaPipe Pose documentation)
ARM_LANDMARKS = {
    "left_shoulder": 11,
    "right_shoulder": 12,
    "left_elbow": 13,
    "right_elbow": 14,
    "left_wrist": 15,
    "right_wrist": 16,
    # Wrist data is kept but commented out, could be useful later
}

# Hips are needed as a torso reference point for the shoulder angle,
# even though they aren't part of the "arm" output themselves
HIP_LANDMARKS = {"left_hip": 23, "right_hip": 24}

# Finger landmarks are only used to draw the hand lines on screen,
# kept out of ARM_LANDMARKS on purpose so they never get written to the CSV
FINGER_LANDMARKS = {
    "left_pinky": 17,
    "right_pinky": 18,
    "left_index": 19,
    "right_index": 20,
    "left_thumb": 21,
    "right_thumb": 22,
}

# Which landmark pairs to draw when rendering an arms-only skeleton
ARM_CONNECTIONS = [
    ("left_shoulder", "right_shoulder"),
    ("left_shoulder", "left_elbow"),
    ("left_elbow", "left_wrist"),
    ("right_shoulder", "right_elbow"),
    ("right_elbow", "right_wrist"),
    ("left_wrist", "left_index"),
    ("left_wrist", "left_pinky"),
    ("left_wrist", "left_thumb"),
    ("right_wrist", "right_index"),
    ("right_wrist", "right_pinky"),
    ("right_wrist", "right_thumb"),
]

ANGLE_FIELDS = [
    "left_elbow_angle",
    "right_elbow_angle",
    "left_shoulder_angle",
    "right_shoulder_angle",
]

# Order of values sent over Bluetooth, one comma-separated ASCII line per
# update: left_elbow,right_elbow,left_shoulder,right_shoulder\n
BT_FIELDS = ANGLE_FIELDS

# Default output: just the angles, nice and small.
MINIMAL_CSV_FIELDS = ["frame", "timestamp_s"] + ANGLE_FIELDS

# Opt-in output (--full-data): angles plus every raw arm landmark's x/y/z/visibility.
FULL_CSV_FIELDS = (
    ["frame", "timestamp_s"]
    + [f"{name}_{axis}" for name in ARM_LANDMARKS for axis in ("x", "y", "z", "visibility")]
    + ANGLE_FIELDS
)


def calculate_angle(a, b, c, use_z=False):
    """
    Angle at vertex b formed by points a-b-c, in degrees (0-180).
    Points are (x, y[, z]) in MediaPipe's normalized coordinate space,
    which is unit-consistent, so no pixel conversion is needed here.

    use_z=False computes a 2D (image-plane) angle instead, which is
    often more stable since MediaPipe's z estimate is noisier than x/y.
    """
    a = np.array(a[:3] if use_z else a[:2], dtype=float)
    b = np.array(b[:3] if use_z else b[:2], dtype=float)
    c = np.array(c[:3] if use_z else c[:2], dtype=float)

    ba = a - b
    bc = c - b
    denom = (np.linalg.norm(ba) * np.linalg.norm(bc)) + 1e-8
    cosine = np.clip(np.dot(ba, bc) / denom, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def calculate_signed_angle(a, b, c):
    """
    Signed angle at vertex b, from vector b->a to vector b->c, in the
    image plane (x,y only), in degrees, range (-180, 180].

    Unlike calculate_angle (which always returns the unsigned 0-180
    "included" angle), this preserves rotation direction. E.g. with the
    upper arm held horizontal, a forearm bent upward and one bent
    downward both have 90 degrees of flexion, but opposite sign here --
    that's the +90 vs -90 distinction.

    Always 2D: sign has no unambiguous meaning in 3D without picking an
    extra reference axis, so this ignores z regardless of use_z elsewhere.
    Magnitude (abs of the result) matches calculate_angle(..., use_z=False).
    """
    a2 = np.array(a[:2], dtype=float)
    b2 = np.array(b[:2], dtype=float)
    c2 = np.array(c[:2], dtype=float)

    ba = -(a2 - b2)
    # ba is negative, such that a straight arm is 0deg instead of oscillating between 180deg and -180deg
    bc = c2 - b2

    cross = ba[0] * bc[1] - ba[1] * bc[0]
    dot = float(np.dot(ba, bc))
    return float(np.degrees(np.arctan2(cross, dot)))


def extract_arm_points(landmarks):
    """Pull out just the arm (+ hip reference) landmarks as (x,y,z,visibility)."""
    pts = {}
    for name, idx in {**ARM_LANDMARKS, **HIP_LANDMARKS}.items():
        lm = landmarks[idx]
        pts[name] = (lm.x, lm.y, lm.z, lm.visibility)
    return pts


def compute_arm_angles(pts, use_z=False):
    def p(name):
        return pts[name][:3]

    return {
        # Signed so +90 (forearm up) and -90 (forearm down) are distinguishable even though both are 90 degrees of flexion
        "left_elbow_angle": calculate_signed_angle(p("left_shoulder"), p("left_elbow"), p("left_wrist")),
        "right_elbow_angle": calculate_signed_angle(p("right_shoulder"), p("right_elbow"), p("right_wrist")),
        # Shoulder angle: how far the upper arm is raised relative to the torso line (hip->shoulder)
        "left_shoulder_angle": calculate_angle(p("left_hip"), p("left_shoulder"), p("left_elbow"), use_z),
        "right_shoulder_angle": calculate_angle(p("right_hip"), p("right_shoulder"), p("right_elbow"), use_z),
        # Wrist angle: elbow-wrist-index, i.e. wrist flexion/extension
        #"left_wrist_angle": calculate_angle(p("left_elbow"), p("left_wrist"), p("left_index"), use_z),
        #"right_wrist_angle": calculate_angle(p("right_elbow"), p("right_wrist"), p("right_index"), use_z),
    }


def draw_arms_only(frame, landmarks, angles, min_visibility=0.5):
    h, w = frame.shape[:2]
    # Combined lookup for display purposes only — fingers are included
    # here so the hand lines still render, even though they're excluded
    # from ARM_LANDMARKS (and therefore the CSV).
    display_points = {**ARM_LANDMARKS, **HIP_LANDMARKS, **FINGER_LANDMARKS}

    def px(name):
        idx = display_points[name]
        lm = landmarks[idx]
        return int(lm.x * w), int(lm.y * h), lm.visibility

    for a, b in ARM_CONNECTIONS:
        ax, ay, av = px(a)
        bx, by, bv = px(b)
        if av < min_visibility or bv < min_visibility:
            continue
        cv2.line(frame, (ax, ay), (bx, by), (255, 255, 255), 3)

    for name in {**ARM_LANDMARKS, **FINGER_LANDMARKS}:
        x, y, v = px(name)
        if v < min_visibility:
            continue
        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

    labels = [
        f"L elbow: {angles['left_elbow_angle']:.0f}deg",
        f"R elbow: {angles['right_elbow_angle']:.0f}deg",
        f"L shoulder: {angles['left_shoulder_angle']:.0f}deg",
        f"R shoulder: {angles['right_shoulder_angle']:.0f}deg",
        #f"L wrist: {angles['left_wrist_angle']:.0f}deg",
        #f"R wrist: {angles['right_wrist_angle']:.0f}deg",
    ]
    for i, text in enumerate(labels):
        cv2.putText(frame, text, (10, 30 + i * 25), cv2.FONT_HERSHEY_COMPLEX,
                    0.6, (0, 255, 0), 2, cv2.LINE_AA)


def open_bluetooth_port(bt_port, bt_baud):
    """
    Open a paired Bluetooth Classic SPP serial port. Raises a RuntimeError
    with an actionable message on failure rather than a raw pyserial
    traceback, since the most common causes are user-fixable (not paired
    yet, or the port is already held open by another program).
    """
    try:
        conn = serial.Serial(bt_port, bt_baud, timeout=1)
    except serial.SerialException as e:
        raise RuntimeError(
            f"Could not open Bluetooth serial port '{bt_port}': {e}\n"
            "Things to check:\n"
            "  1. The ESP32 must already be paired via System Settings > Bluetooth.\n"
            "  2. Run 'python armprocessing.py --list-ports' to confirm the exact port name.\n"
            "  3. Close any other program (e.g. 'pio device monitor') that may already "
            "have the port open — only one process can hold a serial port at a time."
        ) from e

    # Give the Bluetooth SPP link a moment to settle after opening before
    # sending anything — writing immediately can silently drop the first
    # few lines on some macOS Bluetooth stacks.
    time.sleep(2)
    print(f"Bluetooth link opened on {bt_port}")
    return conn


def process(source=0, output_csv="src/arm_data.csv", use_z=False,
            min_detection_confidence=0.5, min_tracking_confidence=0.5,
            full_data=False, bt_port=None, bt_baud=115200, bt_rate=20.0):
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video source: {source}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    pose = mp_pose.Pose(
        min_detection_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence,
    )

    bt = open_bluetooth_port(bt_port, bt_baud) if bt_port else None
    bt_min_interval = (1.0 / bt_rate) if (bt is not None and bt_rate > 0) else 0.0
    last_bt_send = 0.0

    fieldnames = FULL_CSV_FIELDS if full_data else MINIMAL_CSV_FIELDS
    frame_idx = 0
    try:
        with open(output_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            while True:
                ok, frame = cap.read()
                if not ok:
                    break

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = pose.process(rgb)

                if results.pose_landmarks:
                    landmarks = results.pose_landmarks.landmark
                    pts = extract_arm_points(landmarks)
                    angles = compute_arm_angles(pts, use_z=use_z)

                    row = {"frame": frame_idx, "timestamp_s": round(frame_idx / fps, 3)}
                    row.update(angles)
                    if full_data:
                        for name, (x, y, z, vis) in pts.items():
                            if name in ARM_LANDMARKS:
                                row[f"{name}_x"] = x
                                row[f"{name}_y"] = y
                                row[f"{name}_z"] = z
                                row[f"{name}_visibility"] = vis
                    writer.writerow(row)

                    if bt is not None:
                        now = time.time()
                        if now - last_bt_send >= bt_min_interval:
                            line = ",".join(f"{angles[field]:.1f}" for field in BT_FIELDS) + "\n"
                            try:
                                bt.write(line.encode("utf-8"))
                            except serial.SerialException as e:
                                print(f"Bluetooth write failed, dropping the link and continuing with CSV only: {e}")
                                bt.close()
                                bt = None
                            last_bt_send = now

                    draw_arms_only(frame, landmarks, angles)

                cv2.imshow("Arm Tracking (press q to quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                frame_idx += 1
    finally:
        cap.release()
        cv2.destroyAllWindows()
        pose.close()
        if bt is not None:
            bt.close()

    print(f"Done. Wrote {frame_idx} frames of arm data to {output_csv}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Isolate arm landmarks/angles from MediaPipe Pose")
    parser.add_argument("--source", default="0", help="Webcam index (e.g. 0) or path to a video file")
    parser.add_argument("--output", default="src/arm_data.csv", help="CSV file to write per-frame arm data")
    parser.add_argument("--use-3d", action="store_true", help="Compute angles using MediaPipe's z depth estimate (default: 2D, x/y only)")
    parser.add_argument("--full-data", action="store_true", help="Include raw landmark x/y/z/visibility columns in the CSV (default: angles only)")
    parser.add_argument("--bluetooth", default=None, metavar="PORT",
                         help="Serial port of a paired Bluetooth Classic SPP device (e.g. an ESP32 FireBeetle) "
                              "to stream angle data to, in addition to the CSV. See --list-ports.")
    parser.add_argument("--bt-baud", type=int, default=115200,
                         help="Baud rate for the Bluetooth serial connection (default: 115200; mostly ignored "
                              "by Bluetooth SPP itself, but pyserial requires a value)")
    parser.add_argument("--bt-rate", type=float, default=20.0,
                         help="Max angle updates sent over Bluetooth per second, to avoid flooding the link "
                              "or a slow receiver (default: 20)")
    parser.add_argument("--list-ports", action="store_true",
                         help="List available serial ports (including paired Bluetooth devices) and exit")
    args = parser.parse_args()

    if args.list_ports:
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("No serial ports found.")
        else:
            print("Available serial ports:")
            for p in ports:
                print(f"  {p.device}  —  {p.description}")
        raise SystemExit(0)

    source = int(args.source) if args.source.isdigit() else args.source
    process(
        source=source,
        output_csv=args.output,
        use_z=args.use_3d,
        full_data=args.full_data,
        bt_port=args.bluetooth,
        bt_baud=args.bt_baud,
        bt_rate=args.bt_rate,
    )