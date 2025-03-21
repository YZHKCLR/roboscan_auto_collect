#!/usr/bin/env python3

import rospy
import sys
import os
import moveit_commander
from geometry_msgs.msg import Pose
import yaml
import numpy as np
import pandas as pd
from visual_kinematics.RobotSerial import *
import matplotlib.pyplot as plt
import random
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time
from moveit_commander.move_group import RobotTrajectory
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_msgs.msg import JointConstraint, Constraints
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import csv

def parse_yaml_file(filename):
    with open(filename, 'r') as yaml_file:
        parsed_yaml = yaml.safe_load(yaml_file)
    return parsed_yaml


def get_dh_params(manipulator_type):
    switch={
        # Aubo_i10_dh_params
        1: np.array([[0.1632, 0., 0.5 * pi, 0.],
                          [0., 0.647, pi, 0.5 * pi],
                          [0., 0.6005, pi, 0.],
                          [0.2013, 0., -0.5 * pi, -0.5 * pi],
                          [0.1025, 0., 0.5 * pi, 0.],
                          [0.094, 0., 0., 0.]]),
        2: np.array([[0.550, 0.170, 0.5*pi, 0.0],
                          [0.0, 0.880, 0.0, 0.5*pi],
                          [0.0, 0.190, 0.5*pi, 0.0],
                          [0.810, 0.0, -0.5*pi, 0.0],
                          [0.00, 0.0, 0.5*pi, 0.0],               #
                          [0.115, 0.0, 0.0, 0.0]])
    }
    return switch.get(manipulator_type,1)

def get_joint_limits(manipulator_type):
    switch={
        # Aubo_i10 joint limits of each groups, unit: rad
        1: np.array([[-1.2,pi/2.0],[-pi/2.0,pi/4.0],[0, 2.5],[-pi,pi],[-pi,pi],[-pi,pi]]),
        2: np.array([[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]])
    }
    return switch.get(manipulator_type,1)

def get_work_space_limits(manipulator_type):
    switch={
        # Aubo_i10 work space,limit in X, Y and Z directions. unit: mm
        0: np.array([[-100, 100],[-100, 100],[-100, 100]]),
        1: np.array([[0.2, 1.2],[-1.2, 1.2],[0.2, 1.2]])
    }
    return switch.get(manipulator_type,1)
def draw_group_points():
    # Set font globally to Times New Roman
    plt.rcParams['font.family'] = 'Liberation Serif'# these fonts are substitute for times new roman. e.g.:'Liberation Serif'#'DejaVu Serif'
    plt.rcParams['font.size'] = 12  # Set to desired font size
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    for group_pos_point in group_pos_points:
        ax.scatter(group_pos_point[1], group_pos_point[2], group_pos_point[3], marker='o')
    
    # Plot the coordinate
    ax.plot([0,1], [0,0], [0,0], label='3D Line', color='r', linewidth=2)
    ax.plot([0,0], [0,1], [0,0], label='3D Line', color='g', linewidth=2)
    ax.plot([0,0], [0,0], [0,1], label='3D Line', color='b', linewidth=2)

    # Plot the task space line
    x_min=work_space[0][0]
    x_max=work_space[0][1]
    y_min=work_space[1][0]
    y_max=work_space[1][1]
    z_min=work_space[2][0]
    z_max=work_space[2][1]

    ax.plot([x_min,x_min,x_min,x_min,x_min], [y_max,y_min,y_min,y_max,y_max], [z_min,z_min,z_max,z_max,z_min], linestyle='--',label='3D Line', color='k', linewidth=2)
    ax.plot([x_max,x_max,x_max,x_max,x_max], [y_max,y_min,y_min,y_max,y_max], [z_min,z_min,z_max,z_max,z_min], linestyle='--',label='3D Line', color='k', linewidth=2)
    ax.plot([x_min,x_max,x_max,x_min,x_min], [y_max,y_max,y_max,y_max,y_max], [z_min,z_min,z_max,z_max,z_min], linestyle='--',label='3D Line', color='k', linewidth=2)
    ax.plot([x_min,x_max,x_max,x_min,x_min], [y_min,y_min,y_min,y_min,y_min], [z_min,z_min,z_max,z_max,z_min], linestyle='--',label='3D Line', color='k', linewidth=2)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.view_init(elev=30, azim=70)
    plt.axis('equal')
    plt.show()



def translation_matrix(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1],
    ])

def angle_between_vectors(u, v):
    # Calculate the dot product
    dot_product = np.dot(u, v)
    # Calculate the magnitudes of the vectors
    norm_u = np.linalg.norm(u)
    norm_v = np.linalg.norm(v)
    # Calculate the cosine of the angle
    cos_theta = dot_product / (norm_u * norm_v)
    # Ensure the value is within the valid range for arccos
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    # Calculate the angle in radians
    theta = np.arccos(cos_theta)

    return theta 

def rotation_matrix_to_euler_angles(matrix, order='xyz', degrees=False):
    """
    Convert a rotation matrix to Euler angles.
    
    Parameters:
    - matrix: 3x3 numpy array representing the rotation matrix.
    - order: Order of rotations (e.g., 'xyz', 'zyx').
    - degrees: If True, return angles in degrees; otherwise, radians.
    
    Returns:
    - Euler angles in the specified order.
    """
    # Create a Rotation object
    rotation = R.from_matrix(matrix)
    # Convert to Euler angles
    euler_angles = rotation.as_euler(order, degrees=degrees)
    return euler_angles

def euler_angles_to_rotation_matrix(euler_angles, order='xyz', degrees=False):
    """
    Convert Euler angles to a rotation matrix.
    
    Parameters:
    - euler_angles: List or numpy array of Euler angles (e.g., [roll, pitch, yaw]).
    - order: Order of rotations (e.g., 'xyz', 'zyx').
    - degrees: If True, input angles are in degrees; otherwise, radians.
    
    Returns:
    - 3x3 numpy array representing the rotation matrix.
    """
    # Create a Rotation object
    rotation = R.from_euler(order, euler_angles, degrees=degrees)
    # Convert to a rotation matrix
    rotation_matrix = rotation.as_matrix()
    return rotation_matrix

def draw_line_in_rviz(marker_pub, tracker_point, marker_point, Opacity):

    line_marker = Marker()
    line_marker.header.frame_id = "world"
    line_marker.type = Marker.LINE_STRIP  # Can also use Marker.LINE_LIST
    line_marker.action = Marker.ADD

    line_marker.scale.x = 0.005  # Line thickness
    line_marker.color.r = 1.0
    line_marker.color.g = 1.0
    line_marker.color.b = 0.0
    line_marker.color.a = Opacity  # Opacity=1 means white and Opacity=0 means transparent

    # Define points of the line
    point1 = Point(tracker_point[0],tracker_point[1],tracker_point[2])
    point2 = Point(marker_point[0],marker_point[1],marker_point[2])

    line_marker.points.append(point1)
    line_marker.points.append(point2)
 
    marker_pub.publish(line_marker)
 
def check_and_add_vector(vectors, new_vector, delta):
    # Check the 2-norm between the new vector and all previous vectors
    for v in vectors:
        dist = np.linalg.norm(np.array(new_vector) - np.array(v))
        if dist <= delta:
            print(f"Condition violated: New vector and one of the previous vectors have a distance of {dist:.4f}")
            return False  # Condition violated
    
    # If all checks pass, add the new vector
    vectors.append(new_vector)
    return True  # Condition satisfied

def check_joint_collision(num_each_group,joint_positions, group, group_name="manipulator"):
    """
    Check if a set of joint positions causes any collision.

    :param joint_positions: A list or array of joint positions (angles).
    :group: group for check
    :param group_name: The name of the robot's planning group (e.g., "manipulator").
    :return: True if no collision, False if collision detected.
    """

    # Set the joint target to the desired joint positions
    group.set_joint_value_target(joint_positions)

    # Perform a collision check by planning the pose
    # This will try to plan the motion from the current state to the target state.
    plan= group.plan()
    plan_=plan[1]
    plan_points=plan_.joint_trajectory.points
    joint_thresh=0.001
    

    # If planning is successful, the robot is able to reach the target joint positions without collision
    if plan[0]:
        plan_point_error_sum=np.array([0,0,0,0,0,0])
        for plan_point in plan_points:
            plan_point_error_sum=plan_point_error_sum+np.abs(np.array(joint_positions)-np.array(plan_point.positions))
        plan_point_error_arg=plan_point_error_sum/len(plan_points)
        if num_each_group==0:
            return True,plan_
        else:
            if num_each_group>0 and plan_point_error_arg[0]<joint_thresh and plan_point_error_arg[1]<joint_thresh and plan_point_error_arg[2]<joint_thresh:
                #print('plan_point_error_arg',plan_point_error_arg)
                return True,plan_
            else:
                return False,plan_
    else:
        #print("The joint positions cannot be reached due to collision!")
        return False,plan_

# This scheme is not workable
def create_path_constraints(joint_name, joint_idx, com_joints, moveit_group): 
    # create joint constraint for the first joint
    joint_constraint = JointConstraint()
    joint_constraint.joint_name = joint_name  # joint name
    joint_constraint.position = com_joints[joint_idx]  # fixed position
    joint_constraint.tolerance_above = 0.1 
    joint_constraint.tolerance_below = 0.1
    joint_constraint.weight = 0.0

    # add constraint to the plan request
    constraints = Constraints()
    constraints.joint_constraints.append(joint_constraint)

    # 设置约束
    moveit_group.set_path_constraints(constraints)

def callback_real_joint_state(msg):
    global real_joint_state
    global is_robot
    real_joint_state=msg.data
    is_robot=True
    print('real_joint_state',real_joint_state)
    print('is_robot',is_robot)
    

def callback_real_marker_position(msg):
    global real_marker_position
    global is_leica
    real_marker_position=msg.data
    is_leica=True
    print('real_maker_position',real_marker_position)
    print('is_leica',is_leica)

def waitfor_gazebo_robot_move(joint_cmd,thresh):
    delta=1
    global gazebo_joint_state
    while delta>thresh:
        sum=0
        for i in range(6):
            sum=sum+abs(gazebo_joint_state[i]-joint_cmd[i])
        delta=sum
        time.sleep(0)

def joint_state_callback(msg):
    global gazebo_joint_state
    temp_state=msg.position
    gazebo_joint_state=[temp_state[1],temp_state[2],temp_state[0],temp_state[3],temp_state[4],temp_state[5]]


is_leica=False          #默认leica没有采集完毕
is_robot=False          #默认robot没有运动完毕
real_joint_state=[0,0,0,0,0,0]
real_marker_position=[0,0,0]   
gazebo_joint_state=[0,0,0,0,0,0]


# 共享变量，用于存储计算结果
result = False
# 事件，用于通知线程停止
stop_event = threading.Event()

if __name__ == "__main__":
    
 
    ### Step1: load parameter from yaml file
    yaml_file = "/home/yz/catkin_ws/src/roboscan_auto_collect/config/config.yaml"                                         ### Note that "../config/config.yaml" means moves to previous directory level
 
    parsed_data = parse_yaml_file(yaml_file)
 
    Pos_marker_tip_init=parsed_data.get("Pos_marker_tip_init", [])
    Att_marker_tip_init=parsed_data.get("Att_marker_tip_init", [])
    Pos_tracker_base_init=parsed_data.get("Pos_tracker_base_init", [])
    Att_tracker_base_init=parsed_data.get("Att_tracker_base_init", [])
    Group_num_joints=parsed_data.get("Group_num_joints", [])
    Group_step_joints=parsed_data.get("Group_step_joints", [])
    Num_each_group=parsed_data.get("Num_each_group")
    Group_center_joints=parsed_data.get("Group_center_joints", [])
    Manipulator_type=parsed_data.get("Manipulator_type")
    Group_start_joints=parsed_data.get("Group_start_joints")
    Marker_line_angle_thresh=parsed_data.get("Marker_line_angle_thresh")
    Delta=parsed_data.get("Delta")
    Move_type=parsed_data.get("Move_type")
    Group_start_idx=parsed_data.get("Group_start_idx")

    ### Initialize the node
    rospy.init_node("auto_collect_data_node", anonymous=True)


    ### define publishers
    rospy.set_param('/stop_collect', False)  # Set a parameter
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    marker_position_cmd_pub = rospy.Publisher("/marker_position_cmd", Float64MultiArray, queue_size=10)   # 发布消息给leica，让他运动
    path_points_cmd_pub = rospy.Publisher("/path_points_cmd", RobotTrajectory, queue_size=10)       # 发布路径点给机械臂，让他运动
    arm_controller_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)         #发布消息让gazebo中的机械臂尽快运动到指定位置
    ### define subscribers,一旦接收到
    rospy.Subscriber('/real_joint_state', Float64MultiArray, callback_real_joint_state)
    rospy.Subscriber('/real_marker_position', Float64MultiArray, callback_real_marker_position)
    # 订阅 gazebo的 /joint_states 话题
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    # Initialize MoveIt commander
    moveit_commander.roscpp_initialize(sys.argv)
    # Instantiate the move group for the manipulator
    robot = moveit_commander.RobotCommander()
    scene = PlanningSceneInterface()

    # Create a MoveGroupCommander for the manipulator
    moveit_group = moveit_commander.MoveGroupCommander("manipulator")
    # Set the velocity and acceleration scaling factors
    moveit_group.set_max_velocity_scaling_factor(1.0)  # 50% of max velocity
    moveit_group.set_max_acceleration_scaling_factor(1.0)  # 50% of max acceleration

    Link0Ttracker=np.eye(4)
    Link0_Rmtx_tracker=euler_angles_to_rotation_matrix(Att_tracker_base_init, order='xyz', degrees=False)
    Link0Ttracker[:3,:3]=Link0_Rmtx_tracker
    Link0Ttracker[:3,3]= Pos_tracker_base_init

    trackerTLink0=np.linalg.inv(Link0Ttracker)

    print('Link0Ttracker',Link0Ttracker)
    print('trackerTLink0',trackerTLink0)

    dh_params = get_dh_params(Manipulator_type)

    robot_partial = RobotSerial(dh_params[:4][:])
    print('robot_partial',robot_partial)
    robot_overall = RobotSerial(dh_params)
    

    ### Step2: form joint groups, and verify its feasibility, the tip should lie in a task space which should  
    # positions of each group [idx_group, x, y, z]
    group_idx=0
    group_joints=[]   
    group_pos_points=[]
    work_space=get_work_space_limits(0) # 0 表示沒有工作空間限制
                                                   
    for dir_j1 in range(Group_num_joints[0]):
        for dir_j2 in range(Group_num_joints[1]):
            for dir_j3 in range(Group_num_joints[2]):
                group_j1=Group_start_joints[0]+dir_j1*Group_step_joints[0]                                          # joint1 from small to big
                group_j2=Group_start_joints[1]+dir_j2*Group_step_joints[1]                                          # joint2 from small to big
                group_j3=Group_start_joints[2]+dir_j3*Group_step_joints[2]                                          # joint3 from small to big
                robot_partial.forward(np.array([group_j1,group_j2,group_j3,0]))
                Ts = robot_partial.ts
                Link0TLink4 = np.eye(4)
                for j in range(0, 4):
                    Link0TLink4 = Link0TLink4.dot(Ts[j].t_4_4)

                if work_space[0][0]<Link0TLink4[0,3] <work_space[0][1]:
                    if work_space[1][0]<Link0TLink4[1,3]<work_space[1][1]:
                        if work_space[2][0]<Link0TLink4[2,3]<work_space[2][1]:
                            group_joints.append([group_idx, group_j1, group_j2, group_j3])
                            group_pos_points.append([group_idx,Link0TLink4[0,3],Link0TLink4[1,3],Link0TLink4[2,3]])
                            group_idx+=1

    # draw_group_points()
    # step3: generate 
    # Generate random numbers for joint4 joint5 and joint6 between -pi and pi
    Effective_group_num= len(group_pos_points)
    stop_collect=False
    # Specify the folder name
    folder_name1 = "Data/data_aubo"
    folder_name2 = "Data/data_leika"
    folder_name3 = "Data/data_aubo_cmd"
    folder_name4 = "Data/data_leika_cmd"

    # Create the folder
    os.makedirs(folder_name1, exist_ok=True)
    os.makedirs(folder_name2, exist_ok=True)
    os.makedirs(folder_name3, exist_ok=True)
    os.makedirs(folder_name4, exist_ok=True)
    
     
    for idx in range(Effective_group_num):
        print('idx',idx)
        num_each_group=0     
        vectors=[]
        new_vector=[]
        joint_states=[]
        marker_positions=[]   
        joint_position_cmd=[]
        marker_position_cmd=[]


        while num_each_group<Num_each_group and stop_collect==False:
            stop_collect=rospy.get_param('/stop_collect', False)                ### defaul value is False
            range_scale=17.0/18.0
            group_joint=group_joints[idx]
            group_j1 = group_joint[1]
            group_j2 = group_joint[2]
            group_j3 = group_joint[3]
            group_j4 = random.uniform(-pi*range_scale, pi*range_scale)
            group_j5 = random.uniform(-pi*range_scale, pi*range_scale)
            group_j6 = random.uniform(-pi*range_scale, pi*range_scale)
            com_joints=[group_j1,group_j2,group_j3,group_j4,group_j5,group_j6]
            
            robot_overall.forward(np.array(com_joints))
            Ts = robot_overall.ts
            Link0TLink6 = np.eye(4)
            for j in range(6):
                Link0TLink6 = Link0TLink6.dot(Ts[j].t_4_4)

            Link6TMarker=np.eye(4)
            Link6TMarker[:3,:3]= euler_angles_to_rotation_matrix(Att_marker_tip_init, order='xyz', degrees=False) 
            Link6TMarker[:3,3]= Pos_marker_tip_init
            
            Link0TMarker=Link0TLink6@Link6TMarker
            Zaxis_Marker=Link0TMarker[:3,2]
            Pos_Marker=Link0TMarker[:3,3]
            dir_marker2tracker=Pos_tracker_base_init-Pos_Marker
            marker_line_angle=angle_between_vectors(Zaxis_Marker,dir_marker2tracker)
            new_vector=[group_j4,group_j5,group_j6]
 
            # 先满足连线夹角关系与关节距离关系，然后再判断能否规划，这样就不会出现规划time out的警告，并且规划的效率大大提升了，并不需要多线程解决这个问题
            if marker_line_angle< Marker_line_angle_thresh and check_and_add_vector(vectors, new_vector, Delta):
                is_success, plan_res=check_joint_collision(num_each_group,com_joints, moveit_group, group_name="manipulator")
                if is_success:

                    # 将计算的路点发送给aubo i10机械臂
                    path_points_cmd_pub.publish(plan_res)       #将规划好的路径点发送给机械臂


                    # 利用moveit让机械臂实现在gazebo和rviz中的仿真
                    moveit_group.execute(plan_res, wait=True)
                    #moveit_group.go(com_joints,wait=True)
                    #moveit_group.move(com_joints,wait=True)
      
                    # # 创建 JointTrajectory 消息
                    # arm_controller_msg = JointTrajectory()

                    # # 设置消息的 header
                    # arm_controller_msg.header = Header()
                    # arm_controller_msg.header.seq = 0
                    # arm_controller_msg.header.stamp.secs = 0
                    # arm_controller_msg.header.stamp.nsecs = 0
                    # arm_controller_msg.header.frame_id = ''

                    # # 设置关节名称
                    # arm_controller_msg.joint_names = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']

                    # # 创建并设置轨迹点
                    # point = JointTrajectoryPoint()
                    # point.positions = com_joints
                    # point.time_from_start.secs = 0
                    # point.time_from_start.nsecs = 1000

                    # # 添加轨迹点到消息中
                    # arm_controller_msg.points.append(point)

                    # # 发布消息
                    # arm_controller_pub.publish(arm_controller_msg)
                    # waitfor_gazebo_robot_move(com_joints,0.0006) #等待gazebo中的机械臂运动到位置
                    while is_robot==False and stop_collect==False:
                        stop_collect=rospy.get_param('/stop_collect', False)                ### defaul value is False
                        time.sleep(0.5)
                        print('is_leica',is_leica)
                        print('is_robot',is_robot)
                        print('robot motion is not completed')

                    # 计算marker位置，发送给leica
                    trackerTmarker=trackerTLink0@Link0TMarker
                    tracker_tvec_marker=trackerTmarker[:3,3]                ### This marker position in tracker coordinate should be sent to tracker
                    marker_position_cmd.append(1000.0*tracker_tvec_marker.T)     ### unit: mm
                    marker_position_cmd_msg = Float64MultiArray()
                    marker_position_cmd_msg.data=1000.0*tracker_tvec_marker.T
                    marker_position_cmd_pub.publish(marker_position_cmd_msg)

                              

                    # 在RVIZ中画出tracker-marker连线
                    tracker_point=Pos_tracker_base_init
                    draw_line_in_rviz(marker_pub, tracker_point, Pos_Marker,1)
                    time.sleep(0.2) 
                    draw_line_in_rviz(marker_pub, tracker_point, Pos_Marker,0)
                    time.sleep(0.2) 
    


                    # ######### 
                    #is_leica=True
                    #is_robot=True


                    # 判断机械臂是否运行结束以及leica是否采集结束，如果二者条件均满足才能跳出循环进行下面的程序,或者停止采集才可以跳出循环
            
                    while (is_leica==False or is_robot==False) and stop_collect==False:
                        stop_collect=rospy.get_param('/stop_collect', False)                ### defaul value is False
                        time.sleep(0.5)
                        print('is_leica',is_leica)
                        print('is_robot',is_robot)
                        print('leica collection is not completed')

     
                    is_leica=False
                    is_robot=False
                    joint_states.append(real_joint_state)     #當換成實際機械臂時，這個數值就是實際機械臂的關節
                    marker_positions.append(real_marker_position)

                    num_each_group+=1
                    print("num_each_group",num_each_group)
 
        # Save the array to a text file
        filename='Data/data_aubo/group'+str(Group_start_idx+idx)+'.txt'
        with open('Data/data_aubo/group'+str(Group_start_idx+idx)+'.txt', 'w') as file:
            print(filename)
            for row in joint_states:
                print("bbb")
                file.write(', '.join(map(str, row)) + '\n')

        # Save the array to a text file
        with open('Data/data_leika/group'+str(Group_start_idx+idx)+'.csv', 'w') as file:
            print("ccc")
            writer = csv.writer(file, delimiter=';')  # 設置分隔符為分號
            column_headers=['X  [mm]', 'Y  [mm]', 'Z  [mm]']
            # 寫入標題行
            writer.writerow(column_headers)
            writer.writerows(marker_positions)
        print("ccc")


    # Shut down MoveIt commander
    moveit_commander.roscpp_shutdown()
            


    

 
 





 