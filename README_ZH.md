# 简介
## Unitree mujoco
这是一个基于 `Unitree SDK2` 和 `Mujoco` 开发的仿真器。此版本是原项目的**分支（FORK）**，进行了大量修改。
该仿真器旨在与 [Go2-Control Wrapper](https://github.com/7Swaize/go2-control.git) 配合使用。**所有**依赖信息、安装方法和 API 文档均列于该仓库中。
手柄功能在技术上仍予以保留。
## 目录结构
- `iceoryx_interfaces/`：IPC（进程间通信）所需的共享 Iceoryx2 客户端-服务器配置
- `simulate/resources/`：用于加载机器人模型和场景的资源文件
- `simulate/src/cpp/`：使用 C++ 编写的核心 Mujoco 仿真程序
- `simulate/src/python/`：连接 [Go2-Control Wrapper](https://github.com/7Swaize/go2-control.git) 与 C++ 仿真程序之间的 Python 桥接层
## 内部支持的 Unitree SDK2 消息类型：
**当前版本仅支持底层（low-level）开发，主要用于控制器的仿真到实机（sim-to-real）验证**
- `LowCmd`：电机控制指令
- `LowState`：电机状态信息
- `SportModeState`：机器人位置与速度数据
- `IMUState`：位于 `rt/secondary_imu` 话题下的躯干 IMU 状态（仅限 G1）
注：
1. 电机编号与实际机器人硬件一致，具体细节可参见 [Unitree 官方文档](https://support.unitree.com/home/zh/developer)。
2. 在实际机器人硬件中，内置运动控制服务关闭后，`SportModeState` 消息将无法读取。但仿真器保留了该消息，以便用户利用位置与速度信息来分析所开发的控制程序。
## 消息（DDS IDL）类型说明
- Unitree Go2、B2、H1、B2w、Go2w 机器人的底层通信使用 unitree_go idl。
- Unitree G1、H1-2 机器人的底层通信使用 unitree_hg idl。
# 配置
## C++ 仿真器
C++ 仿真器的配置文件位于 `simulate/resources/config/`：
- `global.yaml`：C++ 仿真的配置文件
# 参考文献
若没有为 Go2 机器人开发的 Phase-Guided Terrain Traversal（PGTT，相位引导地形穿越）工作，本项目的平移运动功能将无法实现，详见[此处](https://github.com/NtagkasAlex/phase_guided_terrain_traversal)。
**引用**
\`\`\`bibtex
@inproceedings{ntagkas2025pgtt,
  title={PGTT: Phase-Guided Terrain Traversal for Perceptive Legged Locomotion},
  author={Ntagkas, Alexandros and Kiourt, Chairi and Chatzilygeroudis, Konstantinos},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2026}
}
\`\`\`
