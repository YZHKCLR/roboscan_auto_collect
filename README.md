<<<<<<< HEAD

=======
# RoboScanAutoTrackerCollect
本仓库用于 RoboScan 项目中三轴标定算法的自动数据采集。使用的机械臂为 Aubo i10，使用的跟踪仪为 Leica AT500。

## 目录
- [RoboScanAutoTrackerCollect](#roboscanautotrackercollect)
  - [目录](#目录)
  - [介绍](#介绍)
  - [特性](#特性)
  - [安装](#安装)
    - [依赖项](#依赖项)
    - [安装步骤](#安装步骤)
    - [使用](#使用)
  - [许可证](#许可证)
  - [Demo](#demo)
  - [参考资料](#参考资料)

## 介绍
本项目提供 AUBO AUI-10 机械臂的 ROS1 驱动程序，支持仿真（Gazebo）和真实硬件控制。适用于机器人开发者、研究人员以及工业自动化应用。

## 特性
- 操作系统是ubuntu20.04
- 使用 ROS1(noetic)，利用MoveIt!运动规划
- 提供 Gazebo 仿真环境
- 通过 `aubo_python_control` 连接真实机械臂
- 与Leica跟踪仪采集数据的模块(该模块集成在windows11上) 通过TCP/IP 通信实现

## 安装
### 依赖项
请确保已安装以下软件：
- ROS1 (Noetic)
- Gazebo 9+
- MoveIt!

### 安装步骤
1.建立工作空间
2.根据参考文献中aubo_ros_driver 官方文档，仅安装所需要的依赖项
3.将本仓库提供的软件包拷贝到工作空间的src文件夹下面
4.将aubo_python_control下面的所有so文件全部放到python3的路径下,保证运行python3时能够找到

### 使用
依次运行下列指令
roslaunch aubo_gazebo aubo_bringup.launch robot_model:=aubo_i10
roslaunch aubo_moveit_config moveit_planning_execution.launch sim:=true robot_model:=aubo_i10
roslaunch aubo_moveit_config moveit_rviz.launch         //可有可無

rosrun roboscan_auto_collect test_robotcontrol.py
rosrun roboscan_auto_collect test_leica_cmd_client.py
等待windows那边开启leica的控制程序后运行下面指令
python3 test_auto_collect_marker.py

## 许可证
## Demo
![Demo]()

## 参考资料
- [aubo_ros_driver 官方文档](https://github.com/AuboRobot/aubo_ros_driver)


>>>>>>> Initial commit
