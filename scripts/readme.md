# aii

roslaunch aubo_gazebo aubo_bringup.launch robot_model:=aubo_i10
roslaunch aubo_moveit_config moveit_planning_execution.launch sim:=true robot_model:=aubo_i10
roslaunch aubo_moveit_config moveit_rviz.launch         //可有可無

rosrun roboscan_auto_collect test_robotcontrol.py
rosrun roboscan_auto_collect test_leica_cmd_client.py
python3 test_auto_collect_marker.py















Notes:
test_move_aubo.py: move the aubo robot in gazebo based on Moveit
test_move_aubo_topic.py: move the aubo robot in gazebo based on topic
