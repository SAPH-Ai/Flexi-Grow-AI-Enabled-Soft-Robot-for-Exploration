# 📡 ESP32 Master Control Node

![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![Framework](https://img.shields.io/badge/Framework-Arduino-teal)
![Protocol](https://img.shields.io/badge/Protocol-ESP--NOW_%7C_UDP-orange)

This firmware transforms an ESP32 into a **Master Control Hub**. It establishes a central WiFi Hotspot to receive commands from a GUI or Mobile App and orchestrates the movement of motors, relays, and remote Slave units.



## 🧠 System Overview

* **Role:** System Master (SoftAP + Server).
* **Connectivity:** * **WiFi:** Creates `ESP-MASTER` hotspot (UDP Port `4210`).
    * **Wireless Link:** Controls a remote Slave unit via **ESP-NOW**.
    * **Wired:** Manages 9 Relays and an L298N Motor Driver directly.
* **Interface:** Accepts plain-text ASCII commands via Serial Monitor or UDP packets.

---

## 🔌 Hardware Pin Map

### ⚡ Relay Bank (9-Channel)
| Relay | GPIO Pin | Function |
| :--- | :--- | :--- |
| **R0** | `27` | General Output |
| **R1** | `32` | General Output |
| **R2** | `04` | General Output |
| **R3** | `05` | General Output |
| **R4** | `18` | General Output |
| **R5** | `19` | General Output |
| **R6** | `21` | General Output |
| **R7** | `22` | General Output |
| **R8** | `23` | General Output |

### 🏎️ DC Motor (L298N Driver)
| Signal | GPIO Pin | Description |
| :--- | :--- | :--- |
| **IN3** | `26` | Direction A |
| **IN4** | `25` | Direction B |
| **ENB** | `33` | PWM Speed (LEDC Ch0) |

---

## 🎮 Command API

Send these strings via **Serial Terminal** (115200 baud) or **UDP** to `192.168.4.1:4210`.

### 1. Motor Control
| Command | Action | Example |
| :--- | :--- | :--- |
| `MOTOR F<duty>` | Forward (Duty 0-255) | `MOTOR F 200` |
| `MOTOR R<duty>` | Reverse (Duty 0-255) | `MOTOR R 150` |
| `MOTOR B` | Hard Brake | `MOTOR B` |
| `MOTOR C` | Coast / Stop | `MOTOR C` |

### 2. Relay Switching
Format: `R<Index>:<State>`
* **Turn ON Relay 3:** `R3:1`
* **Turn OFF Relay 0:** `R0:0`

### 3. Remote Servo Control (via ESP-NOW)
Commands sent here are wirelessly bridged to the Slave unit.

**Continuous Rotation:**
* `LEFT`, `RIGHT`, `STOP`, `CENTER`
* **With speed:** `LEFT:8` (Speed 1-10)

**Absolute Positioning:**
* **Servo 1:** `S1:90` (Move S1 to 90°)
* **Servo 2:** `S2:45` (Move S2 to 45°)
* **Dual Move:** `90,180` (Move S1 to 90°, S2 to 180°)

---

## 📡 Wireless Protocols

### 📶 WiFi SoftAP Config
* **SSID:** `ESP-MASTER`
* **Password:** `12345678`
* **IP Address:** `192.168.4.1`

### ⚡ ESP-NOW Bridge
The Master transmits binary structs to the Slave MAC (`EC:E3:34:22:79:18`).

* **Magic Byte `0xA5`:** Continuous movement commands.
* **Magic Byte `0xA6`:** Single absolute servo commands.
* **Raw Pair:** Direct angle pair (no magic byte) for synchronized movement.

---

## 🛠️ Setup Instructions

1.  **Hardware:** Wire your relays and motor driver according to the Pin Map above.
2.  **Software:** Open in Arduino IDE or PlatformIO.
3.  **Config:**
    * Ensure board is selected as **ESP32 Dev Module**.
    * Verify `SLAVE_PEER_MAC` matches your actual Slave device.
4.  **Flash:** Upload the code.
5.  **Connect:** Join the WiFi network `ESP-MASTER` with your phone or PC to start sending commands.

---
