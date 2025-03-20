# RoboScan_Auto_Tracker_Collect
This repository is used for the automatic data collection of the three-axis calibration algorithm in the RoboScan project. The robotic arm used is Aubo i10, and the tracker used is Leica AT500.

## 目录
- [介绍](#介绍)
- [特性](#特性)
- [安装](#安装)
- [使用](#使用)
- [贡献](#贡献)
- [许可证](#许可证)

## 介绍
本项目提供 AUBO AUI-10 机械臂的 ROS1 驱动程序，支持仿真（Gazebo）和真实硬件控制。适用于机器人开发者、研究人员以及工业自动化应用。

## 特性
- 兼容 ROS1，支持 MoveIt! 运动规划
- 提供 Gazebo 仿真环境
- 通过 `aubo_ros_driver` 连接真实机械臂
- 支持 TCP/IP 通信控制

## 安装
### 依赖项
请确保已安装以下软件：
- ROS1 (Melodic/Noetic)
- Gazebo 9+
- MoveIt!

### 安装步骤
1. 克隆仓库：
   ```sh
   git clone https://github.com/yourusername/aubo_ros_driver.git
2. 编译代码：

cd aubo_ros_driver
catkin_make
source devel/setup.bash
## 许可证
本项目基于 MIT 许可证发布。请查看 [LICENSE](LICENSE) 文件了解详情。

## Demo
![Demo](docs/demo.gif)

## 参考资料
- [AUBO 官方文档](https://www.aubo-robotics.com)
- [ROS Wiki](http://wiki.ros.org)

  
