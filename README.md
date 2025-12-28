# 🌱 Flexi-Grow: The Future of Soft Robotics

![Project Status](https://img.shields.io/badge/Status-Prototype_Ready-brightgreen)
![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20Python-blue)
![License](https://img.shields.io/badge/License-MIT-orange)

## 📋 Table of Contents
- [About the Project](#-about-the-project)
- [Why Soft Robotics?](#-why-soft-robotics)
- [Core Features](#-core-features)
- [System Architecture](#-system-architecture)
- [Tech Stack](#-tech-stack)
- [How It Works](#-how-it-works)
- [Use Cases](#-use-cases)
- [Future Vision](#-future-vision)

---

## 📖 About the Project

**Flexi-Grow** is a pneumatically driven soft-growing robotic platform designed to navigate environments where traditional rigid robots fail. Inspired by the natural growth of vines, Flexi-Grow extends its body to squeeze through narrow cracks, twisted pipes, and unstable rubble.

Equipped with **AI-based object detection**, real-time **FPV video**, and a flexible body that eliminates friction, it bridges critical gaps in disaster response, infrastructure inspection, and archaeological exploration.

---

## ❓ Why Soft Robotics?

Traditional robots are rigid, bulky, and can cause secondary collapses in disaster zones. Flexi-Grow solves this by being:
* **Adaptable:** "Grows" through spaces rather than rolling or walking.
* **Safe:** exerts minimal pressure on surroundings, making it ideal for fragile ruins or trapped survivors.
* **Accessible:** reaches deep into narrow pipes and burrows that are inaccessible to humans and rovers.

---

## ✨ Core Features

* **🎈 Pneumatic Growth:** Extends up to several meters by inflating its soft nylon body from a compact chamber.
* **👁️ AI Vision:** Front-mounted FPV camera with AI (YOLO/TensorFlow) to detect survivors, cracks, or artifacts in real-time.
* **🎮 Wireless Control:** Remote operation via a Python-based GUI over UDP/Wi-Fi.
* **🦾 Modular Tip:** Active steering and a robotic gripper for manipulating objects at the tip.
* **🔋 Low Power:** Innovative valve system traps pressure, significantly reducing energy consumption compared to traditional pneumatics.

---

## ⚙️ System Architecture

The system is split into three main units:

1.  **Main Controlling Unit:** The "Brain" (Laptop/PC) running the Python GUI for operator control and video feed.
2.  **Pneumatic Controlling Unit:** The "Heart" containing the air compressor, solenoid valves, and relays managed by an **ESP32**.
3.  **Environment Exploring Unit:** The "Body" containing the soft growing tube, active steering tendons, and the wireless **ESP-01** tip module for camera/gripper control.

---

## 🛠 Tech Stack

### Software
* **Controller:** Python (Tkinter GUI, UDP Networking)
* **Vision:** OpenCV, YOLOv8
* **Firmware:** C++ (Arduino Framework for ESP32)

### Hardware
* **Microcontrollers:** ESP32 (Master), ESP-01 (Tip Slave)
* **Actuators:** L298N Motor Drivers, Servo Motors (Steering), Solenoid Valves (Air Control)
* **Power:** 12V LiPo Battery, Air Compressor (85W)
* **Materials:** Parachute Nylon (Soft Body), Acrylic & Aluminum (Chassis)

---

## 🕹️ How It Works

1.  **Inflation:** The operator sends a `GROW` command. The compressor activates, and valves open to push air into the soft body.
2.  **Extension:** The body unfurls from the pressurized chamber, "growing" forward.
3.  **Steering:** To turn, pneumatic artificial muscles on the sides are pressurized, shortening one side and bending the robot.
4.  **Interaction:** The operator views the live feed. If a target is seen, they can use the gripper to interact or collect samples.

---

## 🌍 Use Cases

### 🚑 Disaster Relief
* **Scenario:** A building collapses after an earthquake.
* **Solution:** Flexi-Grow navigates through unstable rubble voids to locate survivors without disturbing the debris pile.

### 🏭 Infrastructure
* **Scenario:** A deep sewage pipe has a suspected blockage or gas leak.
* **Solution:** The robot enters the pipe, identifies the damage via AI, and transmits the location without human entry.

### 🏺 Archaeology
* **Scenario:** An ancient tomb opening is too small and fragile for entry.
* **Solution:** Flexi-Grow enters non-invasively, mapping the interior and inspecting artifacts without touching them.

---

## 🚀 Future Vision

* **Fully Autonomous AI:** Self-navigation through complex mazes without human input.
* **Space Exploration:** exploring lava tubes on Mars.
* **Medical Endoscopy:** Miniaturized versions for safe internal surgery.

---

*Developed by Team SAPHAI - Innovating for a Safer Tomorrow.*
