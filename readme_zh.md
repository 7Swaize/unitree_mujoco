# 简介
## Unitree mujoco
这是一个基于 `Unitree SDK2` 和 `Mujoco` 开发的仿真器。这是原版的一个 **分支（FORK）**，其中做了大量改动。
该仿真器旨在与 [Go2-Control Wrapper](https://github.com/7Swaize/go2-control.git) 配合使用。**所有**依赖信息、安装说明以及 API 文档均在该项目中列出。
手柄（Joystick）功能在技术上仍予以保留。
## 目录结构
- `iceoryx_interfaces/`：IPC 所需的共享 Iceoryx2 客户端-服务器配置
- `simulate/resources/`：用于加载机器人模型和场景的资源
- `simulate/src/cpp/`：使用 C++ 编写的核心 Mujoco 仿真代码
- `simulate/src/python/`：连接 [Go2-Control Wrapper](https://github.com/7Swaize/go2-control.git) 与 C++ 仿真程序的 Python 桥接层
## 内部支持的 Unitree SDK2 消息：
**当前版本仅支持底层开发，主要用于控制器的仿真到实机（sim to real）验证**
- `LowCmd`：电机控制指令
- `LowState`：电机状态信息
- `SportModeState`：机器人位置和速度数据
- `IMUState`：位于 `rt/secondary_imu` 话题的躯干 IMU 状态（仅限 G1）

注意：
1. 电机编号与实际机器人硬件相对应。具体细节可参考 [Unitree 官方文档](https://support.unitree.com/home/zh/developer)。
2. 在实际机器人硬件上，关闭内置运动控制服务后 `SportModeState` 消息将无法读取。但仿真器保留了该消息，以便用户利用位置和速度信息来分析所开发的控制程序。

## 消息（DDS IDL）类型说明
- Unitree Go2、B2、H1、B2w、Go2w 机器人使用 unitree_go idl 进行底层通信。
- Unitree G1、H1-2 机器人使用 unitree_hg idl 进行底层通信。

# 配置
## C++ 仿真器
C++ 仿真器的配置文件位于 `simulate/resources/config/`：
- `camera.yaml`：Go2 仿真器相机的配置文件
- `global.yaml`：C++ 仿真的配置文件
