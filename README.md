# uav_control - Autonomous Drone Waypoint & Servo Mission

An integrated ROS 2 package for autonomous UAV offboard control, multi-waypoint navigation, and dual-servo payload mechanism actuation using **PX4 Autopilot** and **QGroundControl**.

---

## 📌 Overview

This repository controls a multi-step autonomous drone mission. The system utilizes a modular control architecture where high-level state decisions are separated from low-level execution nodes.

### 🚁 Mission Workflow
The automated flight pipeline executes the following sequence:
1. **Takeoff**: UAV launches autonomously to mission altitude.
2. **Waypoint 1**:
   - Navigates to **Waypoint 1**.
   - Descends to **1 meter above ground level (AGL)**.
   - Rotates two servos to **90°**.
   - Hovers for **5 seconds**.
   - Rotates two servos back to **0°**.
   - Ascends back to the original **Waypoint 1** altitude.
3. **Waypoint 2**:
   - Navigates to **Waypoint 2**.
   - Descends to **1m AGL** $\rightarrow$ Rotates servos **90°** $\rightarrow$ Hovers **5s** $\rightarrow$ Resets servos to **0°** $\rightarrow$ Ascends to Waypoint 2 altitude.
4. **Waypoint 3**:
   - Navigates to **Waypoint 3**.
   - Descends to **1m AGL** $\rightarrow$ Rotates servos **90°** $\rightarrow$ Hovers **5s** $\rightarrow$ Resets servos to **0°** $\rightarrow$ Ascends to Waypoint 3 altitude.
5. **Return to Home (RTL)**: Automatically returns to the takeoff location and lands safely.

---

## 📂 Repository Structure

```text
uav_control/
├── test_servo/           # Firmware & test scripts for Arduino Nano to test & calibrate servos
├── mission_control.py    # High-level state machine; orchestrates offboard & servo nodes
├── offboard_control.py   # Receives movement commands from mission_control and executes PX4 Offboard control
├── servo_control.py      # Receives servo commands from mission_control and controls physical servos
└── README.md             # Repository documentation
```

---

## 🛠️ Prerequisites & Setup Guides

Before building and running this repository, make sure your environment is properly configured. Detailed step-by-step setup guides are available below:

| Component | Description | Guide Link |
| :--- | :--- | :--- |
| **ROS 2 Jazzy** | Installation guide for ROS 2 Jazzy on Ubuntu 24.04 LTS | 📖 [ROS 2 Jazzy Setup Guide](https://app.notion.com/p/ROS-2-Jazzy-3916d47c0bd78051ab8dd649b5ed7c83?v=3026d47c0bd78080959f000c9900247b&source=copy_link) |
| **ROS 2 & PX4** | Setup guide for PX4-Autopilot, `px4_msgs`, and Micro XRCE-DDS Agent | 📖 [ROS 2 - PX4 Setup Guide](https://app.notion.com/p/ROS-2-PX4-3916d47c0bd780f38081c4ae5a1a21a3?v=3026d47c0bd78080959f000c9900247b&source=copy_link) |
| **QGroundControl** | Download and installation guide for QGroundControl (GCS) | 📖 [QGroundControl Download Guide](https://app.notion.com/p/QGroundControl-3926d47c0bd780f0becdd85df5016264?v=3026d47c0bd78080959f000c9900247b&source=copy_link) |

---

## 👤 Author

* **Gia Nghi Luu** - *Electronics & Telecommunications Engineering Student*
* GitHub: [@nghiluu-engr](https://github.com/nghiluu-engr)
* LinkedIn: [Gia Nghi Lưu](https://www.linkedin.com/in/gia-nghi-l%C6%B0u-65b166223/)
