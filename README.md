# 🤖 Autonomous Mobile Robot for Warehouse Automation

This project is a graduation-level autonomous mobile robot built for automating package transport within a warehouse environment. The robot is designed to navigate autonomously, lift packages to adjustable heights using a scissor lift, and pick them up using a suction-based gripping mechanism. It integrates low-level control via Arduino and high-level decision-making and navigation via ROS 2 on a Linux-based Mini PC.

---

## 📦 Hardware Components

| Component                   | Function                                                                 |
|----------------------------|--------------------------------------------------------------------------|
| **Arduino Mega**           | Controls motors, encoders, linear actuator, suction system, and sensors |
| **Mini PC (Linux + ROS 2)**| Runs SLAM, navigation stack, MQTT communication, and system coordination |
| **2x DC Motors + Encoders**| Differential drive and odometry                                          |
| **Caster Wheels**          | Passive wheels for balance and support                                  |
| **LiDAR Sensor**           | Enables mapping and localization using SLAM                             |
| **Scissor Lift**           | Mechanism for vertical movement using a linear actuator                 |
| **Suction Gripper**        | Uses vacuum pump and suction cup to grip packages                       |

---

## 🧠 Software Architecture

### ROS 2 (Foxy/Humble recommended)

- **SLAM Toolbox**  
  Used for real-time mapping and localization.

- **AMCL**  
  Adaptive Monte Carlo Localization for localization on pre-built maps.

- **Nav2**  
  Navigation stack for autonomous path planning and obstacle avoidance.

- **Custom ROS 2 Nodes:**
  - `serial_interface`: Communicates with Arduino over USB serial
  - `mqtt_client`: Subscribes to MQTT topics to receive task commands
  - `lift_control`: Controls the linear actuator via ROS messages
  - `gripper_control`: Controls the vacuum pump/gripper via ROS service

---

## 🔌 Communication

- **Serial Communication (USB)**  
  USB link between Mini PC and Arduino for low-level control and sensor feedback.

- **MQTT Protocol**  
  Used to communicate with a central warehouse server to receive pickup/delivery tasks.

---
