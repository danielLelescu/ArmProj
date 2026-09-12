# Live Pose-Controlled Robotic Arm v1.0 :muscle:

A 2-DOF robotic arm (shoulder + elbow) that mirrors the movements of my real
arm in real time. A webcam feed is run through MediaPipe Pose to extract arm
joint angles, which are streamed over Bluetooth to an ESP32 that drives the
servos.

``` Data Pipeline
Webcam → MediaPipe Pose → Arm Joint Angles (in a CSV) → Bluetooth (SPP) → ESP32 → Servos
```

**Demo -** https://youtube.com/shorts/Py8oe3yVqUA?feature=share <br> (captions in English and French)

## How It Works

1. `armprocessing_BL.py` captures frames from a webcam (or a video file) and
   runs Google's MediaPipe Pose model on each one.
2. From the full body skeleton, only the arm-relevant landmarks are kept:
   shoulders, elbows, wrists (plus the hips, used as a torso reference for
   shoulder angle).
3. Four joint angles are computed per frame:
   - `left_elbow_angle`, `right_elbow_angle` - **signed** angle of flexion at
     the elbow (shoulder–elbow–wrist), so "forearm up" and "forearm down"
     are distinguishable even though both are 90° of flexion.
   - `left_shoulder_angle`, `right_shoulder_angle` - **unsigned** angle
     between the torso line (hip→shoulder) and the upper arm
     (shoulder→elbow), i.e. how far the arm is raised.
4. Every frame is written to a CSV for logging/analysis, and optionally the
   four angles are streamed live over a Bluetooth Classic SPP serial
   connection to a DFRobot FireBeetle ESP32, which uses them to drive the
   shoulder and elbow servos to match.
5. A live preview window shows the webcam feed with the tracked arm
   skeleton and the current angles overlaid.

**Note:** left and right arms are mirror images of each other, so the same
physical gesture (e.g. both forearms pointing up) produces opposite-signed
elbow angles for left vs right.

## Roadmap :construction:
This is one of my first personal robotics projects! I plan to continue this project, as I
feel there are many interesting directions I could take for new features. Here are a few of my ideas:

| Version | Description |
|---|---|
| v1.0 | Live pose mirroring and initial hardware implementation :white_check_mark: | 
| v1.5 | Mainly hardware upgrades: new motors, battery packs, improved mounting, motion smoothing |
| v2.0 | Implementation of computer-side audio keyword recognition as interface for a basic board-side FSM, switching states between Tracking and pre-coded Commands (e.g "Wave" or "Point x Degrees") |
| v3.0 | Implementation of computer-side learning algorithm, observing my movements and responding live to unheard Commands |
| ... | Hand/fingers implementation |

## Setup :computer:

```
pip install mediapipe opencv-python numpy pyserial
```

## Usage :bulb:

```bash
python armprocessing_BL.py                          # default: webcam, live view
python armprocessing_BL.py --source video.mp4       # process a video file
python armprocessing_BL.py --source 0               # source 0 selects webcam
python armprocessing_BL.py --output arms.csv        # custom CSV path (default: src/arm_data.csv)
python armprocessing_BL.py --full-data              # also log raw landmark x/y/z/visibility
python armprocessing_BL.py --use-3d                 # use MediaPipe's z estimate (not recommended — noisier)
```

### Bluetooth streaming to the ESP32 (macOS)

1. Pair the ESP32 via **System Settings > Bluetooth**. It's normal for the
   connection to only hold for a moment.
2. Confirm the ESP32 appears in your device list under **System Settings >
   Bluetooth**.
3. Run `python3 armprocessing_BL.py --list-ports`.
4. Confirm the ESP32 is recognized, e.g. as `/dev/cu.FireBeetle_ArmData`.
5. Run `python3 armprocessing_BL.py --bluetooth <YOUR-PORT>`.
6. When finished, **forget** the ESP32 device via **System Settings >
   Bluetooth** and restart Settings before attempting to reconnect. The
   reconnect issue is a known bug with the FireBeetle ESP32.

When running, one line per processed frame is streamed over the Bluetooth SPP serial
connection, in addition to writing the CSV exactly as without `--bluetooth`.

```bash
python armprocessing_BL.py --bluetooth /dev/cu.FireBeetle_ArmData --bt-rate 10
```
Caps Bluetooth updates to 10/sec instead of the default 20/sec, in case the
receiving sketch is doing slow work (e.g. driving servos) per message and
falls behind at the default rate.

## Hardware :gear:

- DFRobot FireBeetle ESP32, paired over Bluetooth Classic SPP
- 2 servos (shoulder, elbow) - (see docs/DFRobot_Servo_Datasheet.pdf)
- GPIO 25 drives the Shoulder, GPIO 26 drives the Elbow - (see docs/DFRobot_FireBeetle_Datasheet.pdf)
- Powering the servos: I use a 4x AA battery pack in series with a forward-biased diode
- Powering the ESP32: I use a 3x AA battery pack


## Troubleshooting :triangular_flag_on_post:

- **Camera crash (macOS):** `ps aux | grep armprocessing`, then
  `kill -9 <PID>`.
- **Bluetooth port won't open:** make sure the ESP32 is paired, confirm the
  exact port name with `--list-ports`, and make sure no other program (e.g.
  `pio device monitor`) already has the port open as only one process can
  hold a serial port at a time.

## Output

By default, `src/arm_data.csv` contains one row per frame:

| Column | Description |
|---|---|
| `frame` | frame index |
| `timestamp_s` | frame index/fps (seconds) |
| `left_elbow_angle`, `right_elbow_angle` | signed elbow flexion angle (deg) |
| `left_shoulder_angle`, `right_shoulder_angle` | unsigned shoulder elevation angle (deg) |

With `--full-data`, raw `x`, `y`, `z`, `visibility` columns for each arm
landmark (shoulders, elbows, wrists) are included as well.
