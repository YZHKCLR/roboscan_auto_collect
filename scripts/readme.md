在terminal依次运行以下命令<br>
roslaunch aubo_gazebo aubo_bringup.launch<br> robot_model:=aubo_i10<br>
roslaunch aubo_moveit_config moveit_planning_execution.launch sim:=true robot_model:=aubo_i10<br>
roslaunch aubo_moveit_config moveit_rviz.launch (可有可无,并不重要,除非想看仿真中自动采集时的原理)<br>
rosrun roboscan_auto_collect test_leica_cmd_client<br>
等待windows上leica程序启动后运行<br>
python3 test_auto_collect_marker.py<br>