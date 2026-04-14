# 6-Axis Arduino Robot Arm Starter with Calibration and FK Visualizer

This version adds:
- persistent calibration file
- per-joint offset and reversal controls in the GUI
- a simple forward-kinematics module
- a live visualizer with side and top projections
- live estimated tool-center position (X, Y, Z in mm)

## Files

- `arduino/robot_arm_firmware.ino`
- `python/robot_arm_gui.py`
- `python/kinematics.py`
- `python/calibration.json`

## Install

```bash
pip install pyserial
```

## Run

```bash
python python/robot_arm_gui.py
```

## What the FK model assumes

This starter model treats the arm like:
- base yaw
- shoulder pitch
- elbow pitch
- wrist pitch

Wrist roll and gripper do not change the displayed point position yet.

## Geometry tuning

Edit `python/calibration.json` and use your real arm dimensions in millimeters.

## Calibration workflow

1. Connect to the Arduino
2. Adjust offsets and reversed flags in the GUI
3. Click Save Calibration File
4. Click Send Calibration to Arduino
5. Move sliders and compare the visualizer to the real arm
6. Tune geometry values in calibration.json until the visualizer tracks the real mechanism well

## Next upgrades after this

- sequence editor / playback timeline
- inverse kinematics for XYZ targets
- 3D rendering instead of 2D projections
- limit-aware path generation
