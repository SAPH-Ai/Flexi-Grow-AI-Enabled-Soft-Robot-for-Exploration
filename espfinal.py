#!/usr/bin/env python3
import socket
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk, messagebox

APP_TITLE = "ESP32 Master UDP Controller (with Slave controls)"
DEFAULT_PORT = 4210

class UdpClient:
    def __init__(self, on_datagram):
        self.on_datagram = on_datagram
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.sock.bind(("", 0))  # bind for replies
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
        except Exception:
            pass

    def _rx_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                if not data:
                    continue
                self.on_datagram(data.decode(errors="replace").strip(), addr)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception:
                pass

    def send(self, host, port, text):
        try:
            self.sock.sendto(text.encode(), (host, port))
            return True, None
        except Exception as e:
            return False, str(e)

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("980x640")
        self.minsize(900, 580)

        self.msg_queue = queue.Queue()
        self.client = UdpClient(self.on_datagram)
        self.client.start()

        # Top connection frame
        top = ttk.Frame(self, padding=8); top.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(top, text="Target IP:").pack(side=tk.LEFT)
        self.ip_var = tk.StringVar(value="192.168.1.100")  # set to your ESP32 IP
        ttk.Entry(top, width=16, textvariable=self.ip_var).pack(side=tk.LEFT, padx=(4,10))
        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        ttk.Entry(top, width=7, textvariable=self.port_var).pack(side=tk.LEFT, padx=(4,10))
        self.append_newline = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="Append \\n", variable=self.append_newline).pack(side=tk.LEFT, padx=(0,10))
        ttk.Button(top, text="Sync from STATUS", command=self.sync_status).pack(side=tk.LEFT, padx=4)

        # Middle controls
        mid = ttk.Frame(self, padding=8); mid.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Relays
        relay = ttk.LabelFrame(mid, text="Relays (R0..R8)", padding=8)
        relay.grid(row=0, column=0, sticky="nsew", padx=(0,8), pady=(0,8))
        self.relay_vars = [tk.IntVar(value=0) for _ in range(9)]
        for i in range(9):
            ttk.Checkbutton(
                relay, text=f"R{i}", variable=self.relay_vars[i],
                command=lambda idx=i: self.toggle_relay(idx)
            ).grid(row=i//3, column=i%3, sticky="ew", padx=6, pady=6)
        for c in range(3): relay.columnconfigure(c, weight=1)

        # Motor
        # (changed label text only, to match new firmware pins)
        motor = ttk.LabelFrame(mid, text="Motor (ENB=33, IN3=26, IN4=25)", padding=8)
        motor.grid(row=0, column=1, sticky="nsew", padx=(0,8), pady=(0,8))
        self.duty_var = tk.IntVar(value=160)
        ttk.Label(motor, text="Duty (0-255):").grid(row=0, column=0, sticky="w")
        self.duty_scale = ttk.Scale(motor, from_=0, to=255, orient=tk.HORIZONTAL,
                                    command=lambda v: self._update_duty_label())
        self.duty_scale.set(self.duty_var.get())
        self.duty_scale.grid(row=0, column=1, columnspan=3, sticky="ew", padx=6)
        self.duty_lbl = ttk.Label(motor, text=str(self.duty_var.get())); self.duty_lbl.grid(row=0, column=4, sticky="w")
        ttk.Button(motor, text="Forward", command=self.motor_forward).grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        ttk.Button(motor, text="Reverse", command=self.motor_reverse).grid(row=1, column=1, sticky="ew", padx=4, pady=4)
        ttk.Button(motor, text="Brake",   command=self.motor_brake).grid(row=1, column=2, sticky="ew", padx=4, pady=4)
        ttk.Button(motor, text="Coast",   command=self.motor_coast).grid(row=1, column=3, sticky="ew", padx=4, pady=4)
        ttk.Button(motor, text="Set Duty",command=self.motor_set_duty).grid(row=1, column=4, sticky="ew", padx=4, pady=4)
        ttk.Button(motor, text="Status",  command=self.motor_status).grid(row=2, column=0, columnspan=5, sticky="ew", padx=4, pady=4)
        for c in range(5): motor.columnconfigure(c, weight=1)

        # Slave controls (Servos + Switches) -> all send "SLAVE <cmd>"
        slave = ttk.LabelFrame(mid, text="Slave Controls (Servos + Switches)", padding=8)
        slave.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(0,8))

        # Servo 1
        srow = 0
        ttk.Label(slave, text="Servo 1 (S1:angle)").grid(row=srow, column=0, sticky="w")
        self.servo1_var = tk.IntVar(value=90)
        self.servo1_scale = ttk.Scale(slave, from_=0, to=180, orient=tk.HORIZONTAL,
                                      command=lambda v: self._update_servo_label(1))
        self.servo1_scale.set(self.servo1_var.get())
        self.servo1_scale.grid(row=srow, column=1, columnspan=3, sticky="ew", padx=6)
        self.servo1_lbl = ttk.Label(slave, text=str(self.servo1_var.get())); self.servo1_lbl.grid(row=srow, column=4, sticky="w")
        ttk.Button(slave, text="Set S1", command=lambda: self.send_line(f"SLAVE S1:{self.servo1_var.get()}")).grid(row=srow, column=5, sticky="ew", padx=4)

        # Servo 2
        srow += 1
        ttk.Label(slave, text="Servo 2 (S2:angle)").grid(row=srow, column=0, sticky="w")
        self.servo2_var = tk.IntVar(value=90)
        self.servo2_scale = ttk.Scale(slave, from_=0, to=180, orient=tk.HORIZONTAL,
                                      command=lambda v: self._update_servo_label(2))
        self.servo2_scale.set(self.servo2_var.get())
        self.servo2_scale.grid(row=srow, column=1, columnspan=3, sticky="ew", padx=6)
        self.servo2_lbl = ttk.Label(slave, text=str(self.servo2_var.get())); self.servo2_lbl.grid(row=srow, column=4, sticky="w")
        ttk.Button(slave, text="Set S2", command=lambda: self.send_line(f"SLAVE S2:{self.servo2_var.get()}")).grid(row=srow, column=5, sticky="ew", padx=4)

        # Switches
        srow += 1
        ttk.Separator(slave).grid(row=srow, column=0, columnspan=6, sticky="ew", pady=(6,6))
        srow += 1
        self.sw1_var = tk.IntVar(value=0)
        self.sw2_var = tk.IntVar(value=0)
        ttk.Checkbutton(slave, text="SW1", variable=self.sw1_var, command=lambda: self._send_switch(1)).grid(row=srow, column=0, sticky="w", padx=4)
        ttk.Checkbutton(slave, text="SW2", variable=self.sw2_var, command=lambda: self._send_switch(2)).grid(row=srow, column=1, sticky="w", padx=4)

        for c in range(6):
            slave.columnconfigure(c, weight=1)

        # Slave manual passthrough
        srow += 1
        self.slave_var = tk.StringVar(value="PING")
        ttk.Entry(slave, textvariable=self.slave_var).grid(row=srow, column=0, columnspan=5, sticky="ew", padx=(0,6))
        ttk.Button(slave, text="Send Raw (SLAVE ...)", command=self.send_slave).grid(row=srow, column=5, sticky="ew")

        # Log + manual send
        bottom = ttk.Frame(self, padding=8); bottom.pack(side=tk.BOTTOM, fill=tk.BOTH)
        self.log = tk.Text(bottom, height=10, wrap="word"); self.log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb = ttk.Scrollbar(bottom, orient=tk.VERTICAL, command=self.log.yview); sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.log.configure(yscrollcommand=sb.set)

        manual = ttk.Frame(self, padding=(0,8,0,0)); manual.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Label(manual, text="Manual command:").pack(side=tk.LEFT)
        self.manual_var = tk.StringVar(value="STATUS")
        entry = ttk.Entry(manual, textvariable=self.manual_var); entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        entry.bind("<Return>", lambda e: self.send_manual())
        ttk.Button(manual, text="Send", command=self.send_manual).pack(side=tk.LEFT)

        self.after(50, self.process_queue)
        self.bind("<Escape>", lambda e: self.quit())
        mid.columnconfigure(0, weight=1); mid.columnconfigure(1, weight=1)
        mid.rowconfigure(0, weight=1);    mid.rowconfigure(1, weight=0)

    # ---------- Utilities ----------
    def net_target(self):
        host = self.ip_var.get().strip()
        try:
            port = int(self.port_var.get())
        except Exception:
            port = DEFAULT_PORT
        return host, port

    def log_line(self, text):
        ts = time.strftime("%H:%M:%S")
        self.log.insert(tk.END, f"[{ts}] {text}\n")
        self.log.see(tk.END)

    def on_datagram(self, text, addr):
        self.msg_queue.put((text, addr))

    def process_queue(self):
        try:
            while True:
                text, addr = self.msg_queue.get_nowait()
                self.log_line(f"← {addr[0]}:{addr[1]}  {text}")
                # Parse master STATUS for relays/motor
                if text.startswith("STATUS "):
                    self._apply_status_line(text)
                # Parse slave OK responses to keep UI in sync
                elif text.startswith("OK "):
                    self._apply_slave_ok(text)
        except queue.Empty:
            pass
        self.after(50, self.process_queue)

    def send_line(self, line):
        host, port = self.net_target()
        if self.append_newline.get():
            line = line + "\n"
        ok, err = self.client.send(host, port, line)
        if ok:
            self.log_line(f"→ {host}:{port}  {line.strip()}")
        else:
            self.log_line(f"× send failed: {err}")
            messagebox.showerror("Send failed", err)

    # ---------- Relay ----------
    def toggle_relay(self, idx):
        state = self.relay_vars[idx].get()
        self.send_line(f"R{idx}:{1 if state else 0}")

    # ---------- Motor ----------
    def _update_duty_label(self):
        val = int(float(self.duty_scale.get()))
        self.duty_var.set(val)
        self.duty_lbl.configure(text=str(val))

    def motor_forward(self):
        self.send_line(f"MOTOR F {self.duty_var.get()}")

    def motor_reverse(self):
        self.send_line(f"MOTOR R {self.duty_var.get()}")

    def motor_brake(self):
        self.send_line("MOTOR B")

    def motor_coast(self):
        self.send_line("MOTOR C")

    def motor_set_duty(self):
        self.send_line(f"MOTOR D {self.duty_var.get()}")

    def motor_status(self):
        self.send_line("MOTOR S")

    # ---------- Slave controls ----------
    def _update_servo_label(self, which):
        if which == 1:
            val = int(float(self.servo1_scale.get()))
            self.servo1_var.set(val); self.servo1_lbl.configure(text=str(val))
        else:
            val = int(float(self.servo2_scale.get()))
            self.servo2_var.set(val); self.servo2_lbl.configure(text=str(val))

    def _send_switch(self, which):
        if which == 1:
            self.send_line(f"SLAVE SW1:{1 if self.sw1_var.get() else 0}")
        else:
            self.send_line(f"SLAVE SW2:{1 if self.sw2_var.get() else 0}")

    def send_slave(self):
        text = self.slave_var.get().strip()
        if text:
            if not text.upper().startswith("SLAVE "):
                text = "SLAVE " + text
            self.send_line(text)

    # Manual
    def send_manual(self):
        text = self.manual_var.get().strip()
        if text:
            self.send_line(text)

    # ---------- STATUS parsing (from master) ----------
    def _apply_status_line(self, line):
        try:
            parts = line.split()
            rel_part = next((p for p in parts if p.startswith("RELAYS=")), None)
            duty_part = None
            for p in parts:
                if "duty=" in p:
                    duty_part = p
                    break
            if rel_part and rel_part.startswith("RELAYS="):
                bits = rel_part.split("=", 1)[1].strip()
                if len(bits) >= 9:
                    for i in range(9):
                        self.relay_vars[i].set(1 if bits[i] == '1' else 0)
            if duty_part:
                try:
                    d = int(duty_part.split("=", 1)[1])
                    self.duty_scale.set(d); self._update_duty_label()
                except Exception:
                    pass
        except Exception:
            pass

    # ---------- Slave OK parsing ----------
    def _apply_slave_ok(self, line):
        # Expected formats from slave:
        # "OK S1 <angle>"
        # "OK S2 <angle>"
        # "OK SW1 0/1"
        # "OK SW2 0/1"
        try:
            parts = line.strip().split()
            if len(parts) >= 3 and parts[0] == "OK":
                tag = parts[1].upper()
                val = parts[2]
                if tag == "S1":
                    try:
                        a = int(val); a = max(0, min(180, a))
                        self.servo1_scale.set(a); self._update_servo_label(1)
                    except Exception:
                        pass
                elif tag == "S2":
                    try:
                        a = int(val); a = max(0, min(180, a))
                        self.servo2_scale.set(a); self._update_servo_label(2)
                    except Exception:
                        pass
                elif tag == "SW1":
                    self.sw1_var.set(1 if val == "1" else 0)
                elif tag == "SW2":
                    self.sw2_var.set(1 if val == "1" else 0)
        except Exception:
            pass

    def sync_status(self):
        self.send_line("STATUS")

    # ---------- Lifecycle ----------
    def quit(self):
        try:
            self.client.stop()
        except Exception:
            pass
        super().quit()

if __name__ == "__main__":
    app = App()
    app.mainloop()
