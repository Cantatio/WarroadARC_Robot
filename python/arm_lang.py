"""
ARMLang: a tiny domain-specific language for scripting the robot arm.

Suggested extension: .armx
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Dict, Optional


@dataclass
class Command:
    op: str
    args: list
    line_no: int
    raw: str


class ArmScriptError(Exception):
    pass


class ArmScriptEngine:
    def __init__(self, app) -> None:
        self.app = app
        self.commands: List[Command] = []
        self.labels: Dict[str, int] = {}

    def load_file(self, path: str | Path) -> None:
        text = Path(path).read_text(encoding="utf-8")
        self.load_text(text)

    def load_text(self, text: str) -> None:
        self.commands = []
        self.labels = {}

        lines = text.splitlines()
        for idx, original in enumerate(lines, start=1):
            line = original.strip()
            if not line:
                continue
            if line.startswith("#") or line.startswith("//") or line.startswith(";"):
                continue

            if ":" in line and line.endswith(":"):
                label = line[:-1].strip().upper()
                if not label:
                    raise ArmScriptError(f"Line {idx}: empty label.")
                if label in self.labels:
                    raise ArmScriptError(f"Line {idx}: duplicate label '{label}'.")
                self.labels[label] = len(self.commands)
                continue

            parts = line.split()
            op = parts[0].upper()
            args = parts[1:]
            self.commands.append(Command(op=op, args=args, line_no=idx, raw=original))

    def run(self) -> None:
        ip = 0
        loop_stack = []

        while ip < len(self.commands):
            cmd = self.commands[ip]
            op = cmd.op
            args = cmd.args

            if op == "HOME":
                self.app.home_arm()

            elif op == "STOP":
                self.app.stop_arm()

            elif op == "WAIT":
                self._expect_args(cmd, 1)
                duration = float(args[0])
                time.sleep(duration)

            elif op == "SPEED":
                self._expect_args(cmd, 1)
                self.app.speed_var.set(int(float(args[0])))
                self.app.apply_speed()

            elif op == "MOVE":
                if len(args) != 6:
                    raise ArmScriptError(f"Line {cmd.line_no}: MOVE expects 6 joint values.")
                values = [int(float(a)) for a in args]
                self.app.set_joint_values(values)
                self.app.send_joint_targets()

            elif op == "POSE":
                self._expect_args(cmd, 1)
                name = " ".join(args)
                if name not in self.app.poses:
                    raise ArmScriptError(f"Line {cmd.line_no}: unknown pose '{name}'.")
                self.app.set_joint_values(self.app.poses[name])
                self.app.send_joint_targets()

            elif op == "XYZ":
                if len(args) not in (3, 4):
                    raise ArmScriptError(f"Line {cmd.line_no}: XYZ expects X Y Z [PITCH].")
                x = float(args[0]); y = float(args[1]); z = float(args[2])
                pitch = float(args[3]) if len(args) == 4 else 0.0
                solved = self.app.solve_ik_values(x, y, z, pitch, elbow_up=False)
                current = self.app.get_joint_values()
                current[0], current[1], current[2], current[3] = solved
                self.app.set_joint_values(current)
                self.app.send_joint_targets()

            elif op == "XYZU":
                if len(args) not in (3, 4):
                    raise ArmScriptError(f"Line {cmd.line_no}: XYZU expects X Y Z [PITCH].")
                x = float(args[0]); y = float(args[1]); z = float(args[2])
                pitch = float(args[3]) if len(args) == 4 else 0.0
                solved = self.app.solve_ik_values(x, y, z, pitch, elbow_up=True)
                current = self.app.get_joint_values()
                current[0], current[1], current[2], current[3] = solved
                self.app.set_joint_values(current)
                self.app.send_joint_targets()

            elif op == "GRIP":
                self._expect_args(cmd, 1)
                current = self.app.get_joint_values()
                current[5] = int(float(args[0]))
                self.app.set_joint_values(current)
                self.app.send_joint_targets()

            elif op == "WRIST":
                self._expect_args(cmd, 1)
                current = self.app.get_joint_values()
                current[4] = int(float(args[0]))
                self.app.set_joint_values(current)
                self.app.send_joint_targets()

            elif op == "LOG":
                message = " ".join(args)
                self.app.append_log(f"[SCRIPT] {message}")

            elif op == "GOTO":
                self._expect_args(cmd, 1)
                label = args[0].upper()
                if label not in self.labels:
                    raise ArmScriptError(f"Line {cmd.line_no}: unknown label '{label}'.")
                ip = self.labels[label]
                continue

            elif op == "REPEAT":
                self._expect_args(cmd, 1)
                count = int(args[0])
                if count <= 0:
                    raise ArmScriptError(f"Line {cmd.line_no}: REPEAT count must be > 0.")
                loop_stack.append({"start": ip + 1, "remaining": count})

            elif op == "ENDREPEAT":
                if not loop_stack:
                    raise ArmScriptError(f"Line {cmd.line_no}: ENDREPEAT without REPEAT.")
                top = loop_stack[-1]
                top["remaining"] -= 1
                if top["remaining"] > 0:
                    ip = top["start"]
                    continue
                loop_stack.pop()

            else:
                raise ArmScriptError(f"Line {cmd.line_no}: unknown command '{op}'.")

            ip += 1

        if loop_stack:
            raise ArmScriptError("Script ended before ENDREPEAT.")

    @staticmethod
    def _expect_args(cmd: Command, count: int) -> None:
        if len(cmd.args) != count:
            raise ArmScriptError(
                f"Line {cmd.line_no}: {cmd.op} expects {count} argument(s), got {len(cmd.args)}."
            )


EXAMPLE_SCRIPT = """# Example ARMLang program
LOG Starting scripted sequence
SPEED 12
HOME
WAIT 2.0

MOVE 90 110 70 90 90 60
WAIT 1.2

XYZ 170 0 130 0
WAIT 1.0

WRIST 120
GRIP 90
WAIT 0.8

REPEAT 2
    XYZ 160 40 120 0
    WAIT 0.7
    XYZ 160 -40 120 0
    WAIT 0.7
ENDREPEAT

LOG Sequence complete
HOME
"""
