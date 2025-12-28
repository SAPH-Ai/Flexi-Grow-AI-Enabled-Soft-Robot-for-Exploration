#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 MASTER GUI — Updated for new servo controls (LEFT/RIGHT/STOP/CENTER & 2nd servo)
- Auto-discover MASTER by broadcasting "STATUS" on UDP port (default 4210).
- MASTER commands over UDP (same strings work from Serial Monitor):
    Relays:         R<idx>:<0|1>          e.g., R3:1
    Motor:          MOTOR F|R|B|C|D|S     e.g., MOTOR F 160
    Steering S1:    LEFT[:speed], RIGHT[:speed], STOP, CENTER
    Steering S2:    LEFT2[:speed], RIGHT2[:speed], STOP2, CENTER2
    Absolute:       S1:<angle>, S2:<angle>, or "<a>,<b>"
- Webcam panel (optional) using OpenCV (pip install opencv-python).

This file replaces the previous slave-related commands that started with "SLAVE ...".
"""

import socket
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

try:
    import cv2
except Exception:
    cv2 = None  # Webcam optional

APP_TITLE    = "ESP32 Master Controller (Auto-Discover + Webcam) — NEW Servo Controls"
DEFAULT_PORT = 4210
DISCOVERY_BCAST = "255.255.255.255"
DISCOVERY_INTERVAL = 1.0  # seconds


# -------------------- UDP Client --------------------
class UdpClient:
    def __init__(self, on_datagram):
        self.on_datagram = on_datagram
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # allow broadcast
        self.sock.bind(("", 0))  # ephemeral local port to receive replies
        self.running = False
        self.rx_thread = None

    def start(self):
        if self.running:
            return
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except:
            pass

    def _rx_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                if data:
                    self.on_datagram(data.decode(errors="replace").strip(), addr)
            except socket.timeout:
                continue
            except OSError:
                break

    def send(self, host, port, text):
        try:
            self.sock.sendto(text.encode(), (host, port))
            return True, None
        except Exception as e:
            return False, str(e)


# -------------------- GUI App --------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1280x780")
        self.minsize(1080, 680)

        # threading-safe inbound message queue
        self.msg_queue = queue.Queue()

        # UDP client
        self.client = UdpClient(self._enqueue_datagram)
        self.client.start()

        # Discovery state
        self.auto_connect = tk.BooleanVar(value=True)
        self.connected_ip = None
        self._last_status_ts = 0.0

        # -------- Top bar --------
        top = ttk.Frame(self, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Target IP:").pack(side=tk.LEFT)
        self.ip_var = tk.StringVar(value="")
        ttk.Entry(top, width=16, textvariable=self.ip_var).pack(side=tk.LEFT, padx=(4, 8))

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        ttk.Entry(top, width=7, textvariable=self.port_var).pack(side=tk.LEFT, padx=(4, 10))

        ttk.Checkbutton(top, text="Auto-connect", variable=self.auto_connect).pack(side=tk.LEFT, padx=(0, 10))

        self.status_lbl = ttk.Label(top, text="Status: idle")
        self.status_lbl.pack(side=tk.LEFT, padx=(0, 10))

        self.append_newline = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="Append \\n", variable=self.append_newline).pack(side=tk.LEFT, padx=(0, 10))

        ttk.Button(top, text="Sync STATUS", command=self.sync_status).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(top, text="Discover Now", command=self._send_broadcast_status).pack(side=tk.LEFT)

        # -------- Main split (left controls / right webcam) --------
        main = ttk.Frame(self, padding=8)
        main.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        right = ttk.Frame(main)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # ----- Controls grid (left) -----
        grid = ttk.Frame(left)
        grid.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        # Relays
        relay = ttk.LabelFrame(grid, text="Master Relays (R0..R8)", padding=8)
        relay.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        relay.columnconfigure(0, weight=1)
        relay.columnconfigure(1, weight=1)
        relay.columnconfigure(2, weight=1)

        self.relay_vars = [tk.IntVar(value=0) for _ in range(9)]
        for i in range(9):
            ttk.Checkbutton(
                relay,
                text=f"R{i}",
                variable=self.relay_vars[i],
                command=lambda idx=i: self.toggle_relay(idx),
            ).grid(row=i // 3, column=i % 3, sticky="ew", padx=6, pady=6)

        # Motor
        motor = ttk.LabelFrame(grid, text="Master Motor (L298N)", padding=8)
        motor.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        for c in range(5):
            motor.columnconfigure(c, weight=1)

        self.duty_var = tk.IntVar(value=160)
        ttk.Label(motor, text="Duty (0–255):").grid(row=0, column=0, sticky="w")
        self.duty_lbl = ttk.Label(motor, text=str(self.duty_var.get()))
        self.duty_lbl.grid(row=0, column=4, sticky="w", padx=(6, 0))

        self.duty_scale = ttk.Scale(
            motor, from_=0, to=255, orient=tk.HORIZONTAL,
            command=self._on_duty_change
        )
        self.duty_scale.grid(row=0, column=1, columnspan=3, sticky="ew", padx=6)
        self.duty_scale.set(self.duty_var.get())

        ttk.Button(motor, text="Forward",  command=self.motor_forward).grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        ttk.Button(motor, text="Reverse",  command=self.motor_reverse).grid(row=1, column=1, sticky="ew", padx=4, pady=4)
        ttk.Button(motor, text="Brake",    command=self.motor_brake).grid(row=1, column=2, sticky="ew", padx=4, pady=4)
        ttk.Button(motor, text="Coast",    command=self.motor_coast).grid(row=1, column=3, sticky="ew", padx=4, pady=4)
        ttk.Button(motor, text="Set Duty", command=self.motor_set_duty).grid(row=1, column=4, sticky="ew", padx=4, pady=4)
        ttk.Button(motor, text="Status",   command=self.sync_status).grid(row=2, column=0, columnspan=5, sticky="ew", padx=4, pady=4)

        # ---------- Servos (NEW continuous controls) ----------
        slave = ttk.LabelFrame(grid, text="Slave Servos via Master (ESP-NOW)", padding=8)
        slave.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=8, pady=8)
        for c in range(8):
            slave.columnconfigure(c, weight=1)

        # Speed control (applies to LEFT/RIGHT buttons)
        ttk.Label(slave, text="Speed (1..10):").grid(row=0, column=0, sticky="w")
        self.speed_var = tk.IntVar(value=5)
        self.speed_scale = ttk.Scale(slave, from_=1, to=10, orient=tk.HORIZONTAL,
                                     command=lambda v: self._on_speed_change(v))
        self.speed_scale.grid(row=0, column=1, columnspan=2, sticky="ew", padx=6)
        self.speed_lbl = ttk.Label(slave, text=str(self.speed_var.get()))
        self.speed_lbl.grid(row=0, column=3, sticky="w")

        # Servo 1: absolute angle + continuous controls
        ttk.Label(slave, text="Servo 1 (GPIO 15):").grid(row=1, column=0, sticky="w")
        self.servo1_var = tk.IntVar(value=90)
        self.servo1_lbl = ttk.Label(slave, text=str(self.servo1_var.get()))
        self.servo1_lbl.grid(row=1, column=3, sticky="w")
        self.servo1_scale = ttk.Scale(slave, from_=0, to=180, orient=tk.HORIZONTAL,
                                      command=lambda v: self._on_servo_change(1, v))
        self.servo1_scale.grid(row=1, column=1, columnspan=2, sticky="ew", padx=6)
        ttk.Button(slave, text="Set S1", command=lambda: self.send_line(f"S1:{self.servo1_var.get()}")).grid(row=1, column=4, sticky="ew", padx=4)

        ttk.Button(slave, text="LEFT",   command=lambda: self._steer_btn(1, "LEFT")).grid(row=1, column=5, sticky="ew", padx=2)
        ttk.Button(slave, text="RIGHT",  command=lambda: self._steer_btn(1, "RIGHT")).grid(row=1, column=6, sticky="ew", padx=2)
        ttk.Button(slave, text="STOP",   command=lambda: self._steer_btn(1, "STOP")).grid(row=1, column=7, sticky="ew", padx=2)
        ttk.Button(slave, text="CENTER", command=lambda: self._steer_btn(1, "CENTER")).grid(row=1, column=8, sticky="ew", padx=2)

        # Servo 2: absolute angle + continuous controls
        ttk.Label(slave, text="Servo 2 (GPIO 16):").grid(row=2, column=0, sticky="w")
        self.servo2_var = tk.IntVar(value=90)
        self.servo2_lbl = ttk.Label(slave, text=str(self.servo2_var.get()))
        self.servo2_lbl.grid(row=2, column=3, sticky="w")
        self.servo2_scale = ttk.Scale(slave, from_=0, to=180, orient=tk.HORIZONTAL,
                                      command=lambda v: self._on_servo_change(2, v))
        self.servo2_scale.grid(row=2, column=1, columnspan=2, sticky="ew", padx=6)
        ttk.Button(slave, text="Set S2", command=lambda: self.send_line(f"S2:{self.servo2_var.get()}")).grid(row=2, column=4, sticky="ew", padx=4)

        ttk.Button(slave, text="LEFT2",   command=lambda: self._steer_btn(2, "LEFT2")).grid(row=2, column=5, sticky="ew", padx=2)
        ttk.Button(slave, text="RIGHT2",  command=lambda: self._steer_btn(2, "RIGHT2")).grid(row=2, column=6, sticky="ew", padx=2)
        ttk.Button(slave, text="STOP2",   command=lambda: self._steer_btn(2, "STOP2")).grid(row=2, column=7, sticky="ew", padx=2)
        ttk.Button(slave, text="CENTER2", command=lambda: self._steer_btn(2, "CENTER2")).grid(row=2, column=8, sticky="ew", padx=2)

        # Absolute pair (A,B)
        ttk.Separator(slave).grid(row=3, column=0, columnspan=9, sticky="ew", pady=(6, 6))
        self.ab_pair = tk.StringVar(value="90,90")
        ttk.Label(slave, text="Angles pair (A,B):").grid(row=4, column=0, sticky="w")
        ttk.Entry(slave, textvariable=self.ab_pair, width=12).grid(row=4, column=1, sticky="w", padx=4)
        ttk.Button(slave, text="Send A,B", command=self._send_ab).grid(row=4, column=2, sticky="ew", padx=2)

        # Manual & Log
        bottom = ttk.Frame(left, padding=(0, 8, 0, 0))
        bottom.pack(side=tk.TOP, fill=tk.BOTH)
        ttk.Label(bottom, text="Manual command:").pack(side=tk.LEFT)
        self.manual_var = tk.StringVar(value="STATUS")
        entry = ttk.Entry(bottom, textvariable=self.manual_var)
        entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        entry.bind("<Return>", lambda e: self.send_manual())
        ttk.Button(bottom, text="Send", command=self.send_manual).pack(side=tk.LEFT)

        logf = ttk.Frame(left, padding=(0, 8, 0, 0))
        logf.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.log = tk.Text(logf, height=10, wrap="word")
        self.log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb = ttk.Scrollbar(logf, orient=tk.VERTICAL, command=self.log.yview)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.log.configure(yscrollcommand=sb.set)

        # ----- Webcam panel (right) -----
        cam_box = ttk.LabelFrame(right, text="Webcam", padding=8)
        cam_box.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        src_row = ttk.Frame(cam_box)
        src_row.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))
        ttk.Label(src_row, text="Source:").pack(side=tk.LEFT)
        self.cam_source_var = tk.StringVar(value="0")  # default USB cam index 0
        ttk.Entry(src_row, width=24, textvariable=self.cam_source_var).pack(side=tk.LEFT, padx=6)
        ttk.Button(src_row, text="Start", command=self._cam_start).pack(side=tk.LEFT, padx=(4, 2))
        ttk.Button(src_row, text="Stop", command=self._cam_stop).pack(side=tk.LEFT, padx=(2, 4))
        ttk.Label(src_row, text="(Use number for USB index or URL for IP stream)").pack(side=tk.LEFT)

        self.video_label = ttk.Label(cam_box)
        self.video_label.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.cap = None
        self._cam_running = False

        # timers & bindings
        self.after(50, self._process_queue)
        self.after(250, self._discovery_tick)
        self.bind("<Escape>", lambda e: self.quit())
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- Discovery ----------
    def _discovery_tick(self):
        now = time.time()
        if self.auto_connect.get():
            if now - self._last_status_ts > DISCOVERY_INTERVAL:
                self._send_broadcast_status()
        self.after(300, self._discovery_tick)

    def _send_broadcast_status(self):
        try:
            ok, err = self.client.send(DISCOVERY_BCAST, self.port_var.get(), "STATUS")
            if ok:
                self._append_log("→ (BCAST) STATUS")
                self.status_lbl.configure(text="Status: discovering…")
            else:
                self._append_log(f"× broadcast failed: {err}")
        except Exception as e:
            self._append_log(f"× broadcast error: {e}")

    # ---------- Relay ----------
    def toggle_relay(self, idx):
        self.send_line(f"R{idx}:{1 if self.relay_vars[idx].get() else 0}")

    # ---------- Motor ----------
    def _on_duty_change(self, value):
        try:
            val = int(float(value))
        except (TypeError, ValueError):
            val = 0
        self.duty_var.set(val)
        self.duty_lbl.configure(text=str(val))

    def motor_forward(self):  self.send_line(f"MOTOR F {self.duty_var.get()}")
    def motor_reverse(self):  self.send_line(f"MOTOR R {self.duty_var.get()}")
    def motor_brake(self):    self.send_line("MOTOR B")
    def motor_coast(self):    self.send_line("MOTOR C")
    def motor_set_duty(self): self.send_line(f"MOTOR D {self.duty_var.get()}")
    def sync_status(self):    self.send_line("STATUS")

    # ---------- Servos ----------
    def _on_speed_change(self, v):
        try:
            s = int(float(v))
        except Exception:
            s = 5
        s = max(1, min(10, s))
        self.speed_var.set(s)
        self.speed_lbl.configure(text=str(s))

    def _on_servo_change(self, which, value):
        try:
            val = max(0, min(180, int(float(value))))
        except (TypeError, ValueError):
            val = 0
        if which == 1:
            self.servo1_var.set(val)
            self.servo1_lbl.configure(text=str(val))
        else:
            self.servo2_var.set(val)
            self.servo2_lbl.configure(text=str(val))

    def _steer_btn(self, which, cmd):
        # which 1-> servo1, 2-> servo2
        sp = self.speed_var.get()
        if cmd in ("LEFT", "RIGHT", "LEFT2", "RIGHT2"):
            # attach :speed to directional commands
            self.send_line(f"{cmd}:{sp}")
        else:
            self.send_line(cmd)

    def _send_ab(self):
        text = self.ab_pair.get().strip()
        # Expect "A,B"
        if "," in text:
            self.send_line(text)
        else:
            messagebox.showwarning("Angles", "Enter as A,B (e.g., 120,45)")

    # ---------- Manual ----------
    def send_manual(self):
        line = self.manual_var.get().strip()
        if not line:
            return
        self.send_line(line)

    # ---------- I/O helpers ----------
    def net_target(self):
        host = self.ip_var.get().strip()
        port = self.port_var.get()
        return host, port

    def send_line(self, line):
        host, port = self.net_target()
        if not host:
            messagebox.showwarning("Missing IP", "Target IP is empty. Enable Auto-connect or enter IP.")
            return
        if self.append_newline.get():
            line += "\\n"
        ok, err = self.client.send(host, port, line)
        if ok:
            self._append_log(f"→ {host}:{port}  {line.strip()}")
        else:
            self._append_log(f"× send failed: {err}")
            messagebox.showerror("Send failed", err)

    # enqueue inbound datagrams from background thread
    def _enqueue_datagram(self, text, addr):
        self.msg_queue.put((text, addr))

    # process queue on Tk main thread
    def _process_queue(self):
        try:
            while True:
                text, addr = self.msg_queue.get_nowait()
                self._append_log(f"← {addr[0]}:{addr[1]}  {text}")
                # Auto-connect logic: STATUS reply locks onto the sender IP
                if text.startswith("STATUS"):
                    self._last_status_ts = time.time()
                    if self.auto_connect.get():
                        if not self.connected_ip or self.connected_ip != addr[0]:
                            self.connected_ip = addr[0]
                            self.ip_var.set(self.connected_ip)
                        self.status_lbl.configure(text=f"Status: connected to {self.connected_ip}")
                    # Optionally parse status for relays/duty
                    self._apply_status_line(text)
        except queue.Empty:
            pass
        self.after(50, self._process_queue)

    def _append_log(self, s):
        self.log.insert(tk.END, s + "\\n")
        self.log.see(tk.END)

    # ---------- STATUS parsing ----------
    def _apply_status_line(self, line):
        try:
            parts = line.split()
            rel_part = next((p for p in parts if p.startswith("RELAYS=")), None)
            duty_part = next((p for p in parts if p.startswith("duty=")), None)

            if rel_part:
                bits = rel_part.split("=", 1)[1].strip()
                for i in range(min(9, len(bits))):
                    self.relay_vars[i].set(1 if bits[i] == '1' else 0)

            if duty_part:
                d = int(duty_part.split("=", 1)[1])
                self.duty_scale.set(d)
                self._on_duty_change(d)
        except Exception:
            pass

    # ---------- Webcam ----------
    def _cam_start(self):
        if cv2 is None:
            messagebox.showerror("Webcam", "OpenCV not installed. Run: pip install opencv-python")
            return
        src = self.cam_source_var.get().strip()
        if src.isdigit():
            src_val = int(src)
        else:
            src_val = src  # URL
        try:
            if hasattr(self, 'cap') and self.cap is not None:
                self._cam_stop()
            self.cap = cv2.VideoCapture(src_val)
            if not self.cap or not self.cap.isOpened():
                self.cap = None
                messagebox.showerror("Webcam", f"Failed to open source: {src}")
                return
            self._cam_running = True
            self._cam_loop()
        except Exception as e:
            messagebox.showerror("Webcam", f"Error: {e}")

    def _cam_loop(self):
        if not getattr(self, '_cam_running', False) or getattr(self, 'cap', None) is None:
            return
        ok, frame = self.cap.read()
        if ok:
            try:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)
                # Fit to label keeping aspect
                lbl_w = max(320, self.video_label.winfo_width() or 640)
                lbl_h = max(240, self.video_label.winfo_height() or 360)
                img = img.resize(self._fit_rect(img.size, (lbl_w, lbl_h)), Image.LANCZOS)
                imgtk = ImageTk.PhotoImage(image=img)
                self.video_label.imgtk = imgtk
                self.video_label.configure(image=imgtk)
            except Exception:
                pass
        self.after(30, self._cam_loop)

    def _fit_rect(self, src, dst):
        sw, sh = src
        dw, dh = dst
        if sw == 0 or sh == 0:
            return (dw, dh)
        scale = min(dw / sw, dh / sh)
        return (max(1, int(sw * scale)), max(1, int(sh * scale)))

    def _cam_stop(self):
        self._cam_running = False
        try:
            if getattr(self, 'cap', None) is not None:
                self.cap.release()
        finally:
            self.cap = None
        self.video_label.configure(image="")
        self.video_label.imgtk = None

    # ---------- Lifecycle ----------
    def _on_close(self):
        try:
            self._cam_stop()
        except:
            pass
        try:
            self.client.stop()
        except:
            pass
        self.destroy()

    def quit(self):
        self._on_close()


if __name__ == "__main__":
    App().mainloop()
