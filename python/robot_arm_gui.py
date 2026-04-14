
import json, threading, time, tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from arm_lang import ArmScriptEngine, ArmScriptError, EXAMPLE_SCRIPT

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise SystemExit("pyserial is required.\nInstall with: pip install pyserial")

from kinematics import ArmGeometry, forward_kinematics, inverse_kinematics_xyz

APP_DIR = Path(__file__).resolve().parent
POSES_FILE = APP_DIR / "poses.json"
CALIBRATION_FILE = APP_DIR / "calibration.json"

DEFAULT_JOINTS = [
    {"name": "Base", "min": 10, "max": 170, "home": 90, "offset": 0, "reversed": False},
    {"name": "Shoulder", "min": 20, "max": 160, "home": 90, "offset": 0, "reversed": True},
    {"name": "Elbow", "min": 15, "max": 165, "home": 90, "offset": 0, "reversed": False},
    {"name": "Wrist Pitch", "min": 0, "max": 180, "home": 90, "offset": 0, "reversed": False},
    {"name": "Wrist Roll", "min": 0, "max": 180, "home": 90, "offset": 0, "reversed": False},
    {"name": "Gripper", "min": 30, "max": 120, "home": 60, "offset": 0, "reversed": True},
]

class ArmClient:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()
    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open
    def connect(self, port, baud=115200):
        with self.lock:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2.0)
            return self.read_available()
    def disconnect(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None
    def send(self, command, expect_reply=True):
        with self.lock:
            if not self.connected:
                raise RuntimeError("Not connected to Arduino.")
            self.ser.write((command.strip() + "\n").encode("utf-8"))
            self.ser.flush()
            if not expect_reply:
                return ""
            return self.ser.readline().decode("utf-8", errors="ignore").strip()
    def read_available(self):
        if not self.connected:
            return ""
        time.sleep(0.1)
        lines = []
        while self.ser.in_waiting:
            lines.append(self.ser.readline().decode("utf-8", errors="ignore").strip())
        return "\n".join(line for line in lines if line)

class RobotArmApp:
    def __init__(self, root):
        self.root = root
        self.root.title("6-Axis Robot Arm Controller + XYZ IK")
        self.root.geometry("1280x820")
        self.client = ArmClient()
        self.poses = {}
        self.pose_names = []
        self.last_sent = None
        self.geometry_cfg = ArmGeometry()

        self.status_var = tk.StringVar(value="Disconnected")
        self.port_var = tk.StringVar()
        self.speed_var = tk.IntVar(value=20)
        self.live_send_var = tk.BooleanVar(value=True)
        self.selected_pose_var = tk.StringVar()
        self.pose_var_text = tk.StringVar(value="X: 0.0 mm | Y: 0.0 mm | Z: 0.0 mm")

        self.xyz_x_var = tk.StringVar(value="180")
        self.xyz_y_var = tk.StringVar(value="0")
        self.xyz_z_var = tk.StringVar(value="120")
        self.xyz_pitch_var = tk.StringVar(value="0")
        self.elbow_up_var = tk.BooleanVar(value=False)
        self.script_path_var = tk.StringVar()
        self.script_engine = ArmScriptEngine(self)

        self.joints = self._load_calibration()
        self.scales, self.value_labels, self.offset_entries, self.reverse_vars = [], [], [], []

        self._build_ui()
        self._load_poses()
        self.refresh_ports()
        self.update_visualizer()

    def _load_calibration(self):
        if CALIBRATION_FILE.exists():
            try:
                data = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
                joints = data.get("joints", DEFAULT_JOINTS)
                geom = data.get("geometry", {})
                self.geometry_cfg = ArmGeometry(
                    base_height=float(geom.get("base_height", 80.0)),
                    shoulder_length=float(geom.get("shoulder_length", 110.0)),
                    elbow_length=float(geom.get("elbow_length", 110.0)),
                    wrist_length=float(geom.get("wrist_length", 70.0)),
                    tool_length=float(geom.get("tool_length", 40.0)),
                )
                return joints
            except Exception:
                pass
        return DEFAULT_JOINTS

    def _save_calibration(self):
        data = {"geometry": {
                "base_height": self.geometry_cfg.base_height,
                "shoulder_length": self.geometry_cfg.shoulder_length,
                "elbow_length": self.geometry_cfg.elbow_length,
                "wrist_length": self.geometry_cfg.wrist_length,
                "tool_length": self.geometry_cfg.tool_length,
            }, "joints": []}
        for i, joint in enumerate(self.joints):
            data["joints"].append({
                "name": joint["name"], "min": int(joint["min"]), "max": int(joint["max"]),
                "home": int(joint["home"]), "offset": int(self.offset_entries[i].get()),
                "reversed": bool(self.reverse_vars[i].get()),
            })
        CALIBRATION_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=12); outer.pack(fill="both", expand=True)
        left = ttk.Frame(outer); left.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(outer); right.pack(side="left", fill="both", expand=True, padx=(12, 0))

        top = ttk.LabelFrame(left, text="Connection", padding=10); top.pack(fill="x")
        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=24, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=6, sticky="w")
        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=6)
        ttk.Button(top, text="Connect", command=self.connect).grid(row=0, column=3, padx=6)
        ttk.Button(top, text="Disconnect", command=self.disconnect).grid(row=0, column=4, padx=6)
        ttk.Label(top, text="Status:").grid(row=0, column=5, padx=(18, 4), sticky="e")
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=6, sticky="w")

        motion = ttk.LabelFrame(left, text="Motion Controls", padding=10); motion.pack(fill="x", pady=(12, 0))
        ttk.Checkbutton(motion, text="Live send on slider move", variable=self.live_send_var).grid(row=0, column=0, sticky="w")
        ttk.Label(motion, text="Step interval (ms):").grid(row=0, column=1, padx=(20, 6), sticky="e")
        ttk.Spinbox(motion, from_=5, to=100, textvariable=self.speed_var, width=8).grid(row=0, column=2, sticky="w")
        ttk.Button(motion, text="Apply Speed", command=self.apply_speed).grid(row=0, column=3, padx=8)
        ttk.Button(motion, text="Send Joint Targets", command=self.send_joint_targets).grid(row=0, column=4, padx=8)
        ttk.Button(motion, text="HOME", command=self.home_arm).grid(row=0, column=5, padx=8)
        ttk.Button(motion, text="STOP", command=self.stop_arm).grid(row=0, column=6, padx=8)
        ttk.Button(motion, text="Query Status", command=self.query_status).grid(row=0, column=7, padx=8)

        ikf = ttk.LabelFrame(left, text="XYZ Inverse Kinematics", padding=10); ikf.pack(fill="x", pady=(12, 0))
        labels = [("X (mm)", self.xyz_x_var), ("Y (mm)", self.xyz_y_var), ("Z (mm)", self.xyz_z_var), ("Tool pitch (deg)", self.xyz_pitch_var)]
        for idx, (label, var) in enumerate(labels):
            ttk.Label(ikf, text=label).grid(row=0, column=idx*2, sticky="w")
            ttk.Entry(ikf, width=10, textvariable=var).grid(row=0, column=idx*2+1, padx=4)
        ttk.Checkbutton(ikf, text="Elbow-up branch", variable=self.elbow_up_var).grid(row=0, column=8, padx=(12, 4))
        ttk.Button(ikf, text="Solve IK", command=self.solve_ik_to_sliders).grid(row=0, column=9, padx=8)
        ttk.Button(ikf, text="Solve + Send", command=self.solve_ik_and_send).grid(row=0, column=10, padx=8)

        jf = ttk.LabelFrame(left, text="Joint Targets + Calibration", padding=10); jf.pack(fill="both", expand=True, pady=(12, 0))
        headers = ["Joint", "Target", "Value", "Range", "Offset", "Reversed"]
        for c, text in enumerate(headers): ttk.Label(jf, text=text).grid(row=0, column=c, sticky="w", padx=4, pady=(0, 6))
        for idx, joint in enumerate(self.joints):
            row = idx + 1
            ttk.Label(jf, text=joint["name"], width=12).grid(row=row, column=0, sticky="w", pady=6)
            scale = tk.Scale(jf, from_=joint["min"], to=joint["max"], orient="horizontal", length=340, resolution=1,
                             command=lambda _value, i=idx: self.on_slider_changed(i))
            scale.set(joint["home"]); scale.grid(row=row, column=1, padx=4, sticky="we")
            value_label = ttk.Label(jf, text=str(joint["home"]), width=8); value_label.grid(row=row, column=2, sticky="w")
            ttk.Label(jf, text=f"[{joint['min']}..{joint['max']}]").grid(row=row, column=3, sticky="w")
            offset_entry = ttk.Entry(jf, width=8); offset_entry.insert(0, str(joint.get("offset", 0))); offset_entry.grid(row=row, column=4, sticky="w")
            rev_var = tk.BooleanVar(value=bool(joint.get("reversed", False))); ttk.Checkbutton(jf, variable=rev_var).grid(row=row, column=5, sticky="w")
            self.scales.append(scale); self.value_labels.append(value_label); self.offset_entries.append(offset_entry); self.reverse_vars.append(rev_var)
        jf.columnconfigure(1, weight=1)

        actions = ttk.Frame(left); actions.pack(fill="x", pady=(8, 0))
        ttk.Button(actions, text="Save Calibration File", command=self.save_calibration_only).pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="Send Calibration to Arduino", command=self.send_calibration_to_arduino).pack(side="left")

        poses = ttk.LabelFrame(left, text="Poses", padding=10); poses.pack(fill="x", pady=(12, 0))
        ttk.Label(poses, text="Saved pose:").grid(row=0, column=0, sticky="w")
        self.pose_combo = ttk.Combobox(poses, textvariable=self.selected_pose_var, width=28, state="readonly")
        self.pose_combo.grid(row=0, column=1, padx=6, sticky="w")
        ttk.Button(poses, text="Save Current Pose", command=self.save_pose).grid(row=0, column=2, padx=6)
        ttk.Button(poses, text="Load to Sliders", command=self.load_pose_to_sliders).grid(row=0, column=3, padx=6)
        ttk.Button(poses, text="Send Pose", command=self.send_selected_pose).grid(row=0, column=4, padx=6)
        ttk.Button(poses, text="Delete Pose", command=self.delete_pose).grid(row=0, column=5, padx=6)
        ttk.Button(poses, text="Export Poses", command=self.export_poses).grid(row=0, column=6, padx=6)

        logf = ttk.LabelFrame(left, text="Log", padding=10); logf.pack(fill="both", expand=True, pady=(12, 0))
        self.log = tk.Text(logf, height=10, wrap="word"); self.log.pack(fill="both", expand=True)

        vf = ttk.LabelFrame(right, text="Forward Kinematics Visualizer", padding=10); vf.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(vf, width=560, height=560, background="white"); self.canvas.pack(fill="both", expand=True)
        ttk.Label(vf, textvariable=self.pose_var_text).pack(anchor="w", pady=(10, 0))
        ttk.Label(vf, text="Views: left = side projection (radius vs height), right = top projection (X/Y).").pack(anchor="w")

    def append_log(self, text):
        if text:
            self.log.insert("end", text + "\n"); self.log.see("end")
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get(): self.port_var.set(ports[0])
        self.append_log("Ports refreshed: " + (", ".join(ports) if ports else "none found"))
    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Connect", "Choose a serial port first."); return
        try:
            banner = self.client.connect(port); self.status_var.set(f"Connected to {port}"); self.append_log(f"Connected to {port}")
            if banner: self.append_log(banner)
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc)); self.status_var.set("Disconnected")
    def disconnect(self):
        self.client.disconnect(); self.status_var.set("Disconnected"); self.append_log("Disconnected.")
    def get_joint_values(self):
        return [int(scale.get()) for scale in self.scales]
    def set_joint_values(self, values):
        for i, value in enumerate(values):
            joint = self.joints[i]; value = int(round(value))
            value = max(int(joint["min"]), min(int(joint["max"]), value))
            self.scales[i].set(value); self.value_labels[i]["text"] = str(value)
        self.update_visualizer()
    def on_slider_changed(self, index):
        self.value_labels[index]["text"] = str(int(self.scales[index].get()))
        self.update_visualizer()
        if self.live_send_var.get(): self.send_joint_targets()
    def send_joint_targets(self):
        values = self.get_joint_values()
        if values == self.last_sent: return
        cmd = "M " + " ".join(map(str, values))
        try:
            reply = self.client.send(cmd); self.last_sent = values
            self.append_log("> " + cmd); self.append_log("< " + reply)
        except Exception as exc:
            self.append_log("Send failed: " + str(exc))
    def apply_speed(self):
        try:
            step_ms = int(self.speed_var.get()); reply = self.client.send(f"SPEED {step_ms}")
            self.append_log(f"> SPEED {step_ms}"); self.append_log("< " + reply)
        except Exception as exc:
            self.append_log("Speed update failed: " + str(exc))
    def home_arm(self):
        try:
            reply = self.client.send("HOME"); self.append_log("> HOME"); self.append_log("< " + reply)
            home_values = [j["home"] for j in self.joints]; self.set_joint_values(home_values); self.last_sent = home_values
        except Exception as exc:
            self.append_log("HOME failed: " + str(exc))
    def stop_arm(self):
        try:
            reply = self.client.send("STOP"); self.append_log("> STOP"); self.append_log("< " + reply)
        except Exception as exc:
            self.append_log("STOP failed: " + str(exc))
    def query_status(self):
        try:
            reply = self.client.send("STATUS"); self.append_log("> STATUS"); self.append_log("< " + reply)
        except Exception as exc:
            self.append_log("STATUS failed: " + str(exc))
    def browse_script(self):
        path = filedialog.askopenfilename(
            title="Open ARMLang script",
            filetypes=[("ARMLang files", "*.armx"), ("Text files", "*.txt"), ("All files", "*.*")],
        )
        if path:
            self.script_path_var.set(path)

    def write_example_script(self):
        path = filedialog.asksaveasfilename(
            title="Save example ARMLang script",
            defaultextension=".armx",
            filetypes=[("ARMLang files", "*.armx"), ("Text files", "*.txt"), ("All files", "*.*")],
        )
        if not path:
            return
        Path(path).write_text(EXAMPLE_SCRIPT, encoding="utf-8")
        self.script_path_var.set(path)
        self.append_log(f"Example script written: {path}")

    def solve_ik_values(self, x, y, z, pitch, elbow_up=False):
        return inverse_kinematics_xyz(
            x=x,
            y=y,
            z=z,
            geometry=self.geometry_cfg,
            tool_pitch_deg=pitch,
            elbow_up=bool(elbow_up),
        )

    def run_script_file(self):
        path = self.script_path_var.get().strip()
        if not path:
            messagebox.showerror("Run script", "Choose a script file first.")
            return
        try:
            self.append_log(f"[SCRIPT] Loading {path}")
            self.script_engine.load_file(path)
            self.script_engine.run()
            self.append_log("[SCRIPT] Finished successfully.")
        except ArmScriptError as exc:
            messagebox.showerror("Script error", str(exc))
            self.append_log(f"[SCRIPT] Error: {exc}")
        except Exception as exc:
            messagebox.showerror("Script run failed", str(exc))
            self.append_log(f"[SCRIPT] Failure: {exc}")

    def solve_ik_to_sliders(self):
        try:
            x = float(self.xyz_x_var.get()); y = float(self.xyz_y_var.get()); z = float(self.xyz_z_var.get()); pitch = float(self.xyz_pitch_var.get())
            solved = inverse_kinematics_xyz(x=x, y=y, z=z, geometry=self.geometry_cfg, tool_pitch_deg=pitch, elbow_up=bool(self.elbow_up_var.get()))
            current = self.get_joint_values(); current[0], current[1], current[2], current[3] = solved
            self.set_joint_values(current)
            self.append_log("IK solved: " + ", ".join(f"{n}={int(round(v))}" for n, v in zip(["Base","Shoulder","Elbow","WristPitch"], solved)))
        except Exception as exc:
            messagebox.showerror("IK solve failed", str(exc)); self.append_log("IK failed: " + str(exc))
    def solve_ik_and_send(self):
        self.solve_ik_to_sliders(); self.send_joint_targets()
    def save_calibration_only(self):
        try:
            self._save_calibration(); self.append_log(f"Calibration saved to: {CALIBRATION_FILE}")
        except Exception as exc:
            messagebox.showerror("Save calibration", str(exc))
    def send_calibration_to_arduino(self):
        try:
            self._save_calibration()
            for i in range(len(self.joints)):
                offset = int(self.offset_entries[i].get()); rev = 1 if self.reverse_vars[i].get() else 0
                r1 = self.client.send(f"OFFSET {i} {offset}"); r2 = self.client.send(f"REV {i} {rev}")
                self.append_log(f"> OFFSET {i} {offset}"); self.append_log("< " + r1)
                self.append_log(f"> REV {i} {rev}"); self.append_log("< " + r2)
            self.append_log("Calibration sent to Arduino.")
        except Exception as exc:
            self.append_log("Calibration send failed: " + str(exc))
    def _load_poses(self):
        if not POSES_FILE.exists():
            self.pose_combo["values"] = []; return
        try:
            data = json.loads(POSES_FILE.read_text(encoding="utf-8")); self.poses = data if isinstance(data, dict) else {}
        except Exception:
            self.poses = {}
        self.pose_names = sorted(self.poses.keys()); self.pose_combo["values"] = self.pose_names
        if self.pose_names and not self.selected_pose_var.get(): self.selected_pose_var.set(self.pose_names[0])
    def _save_poses_to_disk(self):
        POSES_FILE.write_text(json.dumps(self.poses, indent=2), encoding="utf-8"); self._load_poses()
    def save_pose(self):
        name = simple_prompt(self.root, "Save pose", "Pose name:")
        if not name: return
        self.poses[name] = self.get_joint_values(); self._save_poses_to_disk(); self.selected_pose_var.set(name); self.append_log(f"Pose saved: {name}")
    def load_pose_to_sliders(self):
        name = self.selected_pose_var.get().strip()
        if not name or name not in self.poses:
            messagebox.showerror("Load pose", "Choose a valid saved pose."); return
        self.set_joint_values(self.poses[name]); self.append_log(f"Loaded pose to sliders: {name}")
    def send_selected_pose(self):
        self.load_pose_to_sliders(); self.send_joint_targets()
    def delete_pose(self):
        name = self.selected_pose_var.get().strip()
        if not name or name not in self.poses:
            messagebox.showerror("Delete pose", "Choose a valid saved pose."); return
        del self.poses[name]; self._save_poses_to_disk(); self.append_log(f"Deleted pose: {name}")
    def export_poses(self):
        path = filedialog.asksaveasfilename(title="Export poses", defaultextension=".json", filetypes=[("JSON files", "*.json"), ("All files", "*.*")])
        if not path: return
        Path(path).write_text(json.dumps(self.poses, indent=2), encoding="utf-8"); self.append_log(f"Exported poses to: {path}")
    def update_visualizer(self):
        joints = self.get_joint_values() if self.scales else [90, 90, 90, 90, 90, 60]
        pts = forward_kinematics(joints, self.geometry_cfg); tool = pts[-1]
        self.pose_var_text.set(f"X: {tool[0]:.1f} mm | Y: {tool[1]:.1f} mm | Z: {tool[2]:.1f} mm")
        self.canvas.delete("all")
        w, h, pad = int(self.canvas.winfo_width() or 560), int(self.canvas.winfo_height() or 560), 30
        half = w // 2
        lx0, ly0, lx1, ly1 = pad, pad, half - pad, h - pad
        rx0, ry0, rx1, ry1 = half + pad, pad, w - pad, h - pad
        self.canvas.create_rectangle(lx0, ly0, lx1, ly1); self.canvas.create_text(lx0 + 6, ly0 + 6, anchor="nw", text="Side")
        self.canvas.create_rectangle(rx0, ry0, rx1, ry1); self.canvas.create_text(rx0 + 6, ry0 + 6, anchor="nw", text="Top")
        xs, ys, zs = [p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts]
        rs = [(p[0] ** 2 + p[1] ** 2) ** 0.5 for p in pts]
        max_r = max(200.0, max(abs(v) for v in rs) + 20.0)
        min_z, max_z = min(0.0, min(zs) - 20.0), max(200.0, max(zs) + 20.0)
        max_xy = max(200.0, max(max(abs(v) for v in xs), max(abs(v) for v in ys)) + 20.0)
        def map_side(r, z):
            sx = lx0 + (r / max_r) * (lx1 - lx0)
            sy = ly1 - ((z - min_z) / (max_z - min_z)) * (ly1 - ly0)
            return sx, sy
        def map_top(x, y):
            sx = rx0 + ((x + max_xy) / (2 * max_xy)) * (rx1 - rx0)
            sy = ry1 - ((y + max_xy) / (2 * max_xy)) * (ry1 - ry0)
            return sx, sy
        self.canvas.create_line(lx0, map_side(0, 0)[1], lx1, map_side(0, 0)[1], dash=(4, 3))
        ctop = map_top(0, 0)
        self.canvas.create_line(rx0, ctop[1], rx1, ctop[1], dash=(4, 3))
        self.canvas.create_line(ctop[0], ry0, ctop[0], ry1, dash=(4, 3))
        side_pts = [map_side(r, z) for r, z in zip(rs, zs)]
        top_pts = [map_top(x, y) for x, y in zip(xs, ys)]
        for arr in (side_pts, top_pts):
            for i in range(len(arr) - 1): self.canvas.create_line(*arr[i], *arr[i + 1], width=3)
            for i, p in enumerate(arr):
                self.canvas.create_oval(p[0]-4, p[1]-4, p[0]+4, p[1]+4, fill="black")
                self.canvas.create_text(p[0] + 8, p[1] - 8, anchor="sw", text=str(i))

def simple_prompt(parent, title, prompt):
    dialog = tk.Toplevel(parent); dialog.title(title); dialog.transient(parent); dialog.grab_set(); dialog.resizable(False, False)
    result = {"value": ""}
    ttk.Label(dialog, text=prompt).pack(padx=12, pady=(12, 4))
    entry = ttk.Entry(dialog, width=30); entry.pack(padx=12, pady=4); entry.focus_set()
    def ok(): result["value"] = entry.get().strip(); dialog.destroy()
    def cancel(): dialog.destroy()
    bf = ttk.Frame(dialog); bf.pack(padx=12, pady=12)
    ttk.Button(bf, text="OK", command=ok).pack(side="left", padx=4)
    ttk.Button(bf, text="Cancel", command=cancel).pack(side="left", padx=4)
    dialog.wait_window(); return result["value"]

def main():
    root = tk.Tk()
    style = ttk.Style(root)
    if "vista" in style.theme_names(): style.theme_use("vista")
    app = RobotArmApp(root)
    root.bind("<Configure>", lambda event: app.update_visualizer())
    root.mainloop()

if __name__ == "__main__":
    main()
