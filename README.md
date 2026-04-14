# 6-Axis Arduino Robot Arm Controller

### With Calibration, Forward Kinematics, Inverse Kinematics, and ARMLang

## Overview

Full-stack control system for a 6-axis servo robotic arm using Arduino +
Python GUI.

Includes: - Manual control - Forward kinematics visualization - Inverse
kinematics (XYZ) - Custom scripting language (ARMLang)

## Features

-   6-axis servo control
-   GUI with real-time sliders
-   FK visualization (side + top)
-   IK solver (XYZ + pitch)
-   Calibration system
-   ARMLang scripting (.armx)

## Installation

``` bash
pip install pyserial
```

## Run

``` bash
python python/robot_arm_gui.py
```

## Calibration

-   Adjust offsets and reversal in GUI
-   Save + send to Arduino
-   Edit geometry in `calibration.json`

## Inverse Kinematics

-   Enter X, Y, Z, pitch
-   Solve IK or Solve + Send
-   Toggle elbow-up if needed

## ARMLang (.armx)

### Commands

-   HOME, STOP
-   WAIT `<seconds>`{=html}
-   SPEED `<ms>`{=html}
-   MOVE j1 j2 j3 j4 j5 j6
-   XYZ x y z \[pitch\]
-   XYZU x y z \[pitch\]
-   WRIST `<angle>`{=html}
-   GRIP `<angle>`{=html}
-   LOG `<message>`{=html}
-   REPEAT / ENDREPEAT
-   label: / GOTO

### Example

``` text
LOG Start
SPEED 12
HOME
WAIT 2
XYZ 170 0 130 0
WAIT 1
GRIP 90
HOME
LOG Done
```
