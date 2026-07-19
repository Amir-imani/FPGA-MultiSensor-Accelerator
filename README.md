# FPGA-MultiSensor-Accelerator
Zero-DSP Multi-Sensor Hardware Accelerator for Robotics

# 🚀 Zero-DSP Multi-Sensor Hardware Accelerator for Robotics

[![FPGA](https://img.shields.io/badge/FPGA-Xilinx_Spartan--6-blue.svg)](#)
[![Language](https://img.shields.io/badge/Language-Verilog_HDL-orange.svg)](#)
[![Performance](https://img.shields.io/badge/Fmax-115.4_MHz-success.svg)](#)
[![DSP Usage](https://img.shields.io/badge/DSP_Blocks-0%25-brightgreen.svg)](#)

A high-performance, fully pipelined, and multiplierless (Zero-DSP) hardware accelerator designed for real-time sensor fusion and adaptive filtering of LiDAR and IMU data in autonomous robotics.

---

## 📌 Description

### Overview
Processing high-throughput sensor data (LiDAR point clouds and IMU streams) on conventional Von Neumann CPUs often leads to significant computational bottlenecks and unpredictable OS jitter. This project introduces an industrial-grade, highly modular RTL architecture that offloads Cartesian coordinate transformations and adaptive noise filtering to an FPGA. 

By eliminating the need for dedicated hardware multipliers (DSP slices), this IP Core ensures extreme area-efficiency while achieving an operating frequency of **115.399 MHz** on a legacy Spartan-6 device—translating to a throughput of over 115 million samples per second.

### 🌟 Key Features
*   **Multiplierless Architecture (Zero-DSP):** All filtering and trigonometric operations are implemented using shift-and-add techniques and Look-Up Tables (LUTs), reserving precious DSP slices for higher-level AI/Vision tasks.
*   **Multi-Architecture CORDIC:** Features three selectable CORDIC architectures, including a Model-Based Design (MBD) version via MATLAB HDL Coder and a hand-crafted 13-stage explicit pipeline to bypass synthesis tool limitations (Retiming clustering).
*   **Robust Clock Domain Crossing (CDC):** Implements a Toggle-Based Two-Way Handshake synchronizer to safely interface asynchronous, slow-varying LiDAR data with the high-speed system clock without metastability issues.
*   **Adaptive Shift-Based Kalman Filter:** A 1D tracking filter that dynamically adjusts its Kalman gains ($\alpha, \beta$) using barrel shifters based on the innovation error magnitude, suppressing spike outliers in a single clock cycle.
*   **Distributed Arithmetic (DA) FIR Filter:** A 4-tap low-pass filter for IMU vibration rejection utilizing DA to compute convolution entirely via 16x16 LUTs.

### ⚙️ Architecture Selection (CORDIC)
The project is built with modularity in mind. You can easily switch between CORDIC architectures by modifying the `` `define `` macros in `CORDIC_HDL_Core_fixpt.v`:

```verilog
// `define CORDIC_V1_COMBINATIONAL  // V1: MATLAB Auto-Generated (Combinational, ~28 MHz)
// `define CORDIC_V2_MBD_RETIMING   // V2: MATLAB Distributed Pipelining (~30 MHz)
`define CORDIC_V3_HANDCRAFTED       // V3: Hand-crafted 13-Stage Pipeline (~115.4 MHz)
```

