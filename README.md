# ParkingScenes: A Structured Dataset for End-to-End Autonomous Parking in Simulation

**ParkingScenes** is a comprehensive multimodal dataset specifically designed for **end-to-end autonomous parking** in the **CARLA** simulator. It features structured and repeatable parking trajectories generated using a **Hybrid A\*** planner and **Model Predictive Control (MPC)** across diverse parking layouts and pedestrian conditions.

> 🔗 **Project Homepage**: [https://github.com/haonan-ai/ParkingScenes](https://github.com/haonan-ai/ParkingScenes)  
> 📦 **Dataset Download**: [Baidu Drive](https://pan.baidu.com/s/1bPbPc0RCrA2IMFQD46KyLQ?pwd=1234) (Access Code: `1234`)

## 🌟 Features

- ✅ 704 **structured parking episodes** with synchronized sensor and control data  
- ✅ Covers **reverse-in** and **parallel parking** scenarios in urban-style environments  
- ✅ Includes both **pedestrian-present** and **pedestrian-free** conditions  
- ✅ Provides multimodal data: RGB, depth, BEV, vehicle motion, and control signals  
- ✅ Built on **CARLA 0.9.11** for reproducible and scalable simulation  
- ✅ Fully open-sourced **data collection pipeline**

## ⚙️ Environment Setup

To set up the simulation environment and data collection tools:

```bash
# Clone the repository
git clone https://github.com/haonan-ai/ParkingScenes.git
cd ParkingScenes

# Create and activate the conda environment
conda env create -f environment.yml
conda activate ParkingScenes

# Set up CARLA and required dependencies
chmod +x setup_carla.sh
./setup_carla.sh
```

## 🚘 Data Collection Usage
To launch the CARLA simulator and start collecting data:

Step 1: Open the first terminal in the project directory:
```bash
./carla/CarlaUE4.sh
```
Step 2: Open a second terminal (with ParkingScenes environment activated):
```bash
python3 collect.py
```

## 📚 Citation
📌 Citation will be added once the paper is officially accepted.

## 📬 Contact
Haonan Chen – chenhaonan2024@ia.ac.cn

## 🙏 Acknowledgements

This project builds upon and improves the following open-source repositories:

- [**MotionPlanning** by zhm-real](https://github.com/zhm-real/MotionPlanning):  
  Used and extended the **Hybrid A\*** global path planning algorithm.

- [**AutomatedValetParking** by wenqing-2021](https://github.com/wenqing-2021/AutomatedValetParking):  
  Adopted and improved the **Model Predictive Control (MPC)** for trajectory tracking.

- [**e2e-parking-carla** by qintonguav](https://github.com/qintonguav/e2e-parking-carla):  
  Modified the **data collection pipeline**, replacing manual keyboard-based parking with fully automated control.
