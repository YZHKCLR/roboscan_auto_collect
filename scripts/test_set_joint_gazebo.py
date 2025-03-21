#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelConfiguration
from std_msgs.msg import Float64

def set_joint_positions(model_name, joint_names, joint_positions):
    """
    设置 Gazebo 中机械臂的关节状态
    """
    rospy.wait_for_service('/gazebo/set_model_configuration')
    try:
        set_model_config = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        resp = set_model_config(
            model_name=model_name,
            urdf_param_name="robot_description",  # URDF 参数名称
            joint_names=joint_names,
            joint_positions=joint_positions
        )
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def main():
    # 初始化节点
    rospy.init_node('set_gazebo_joint_positions')

    # 设置 Gazebo 中机械臂的关节状态
    model_name = "aubo_i10"  # 替换为你的机器人模型名称
    joint_names = ["shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint","wrist3_joint"]  # 替换为你的关节名称
    joint_positions = [0.0, 0.5, -1.0, 1.5, 0.0, 1]  # 替换为你的目标关节角度

    success = set_joint_positions(model_name, joint_names, joint_positions)
    if success:
        rospy.loginfo("关节状态设置成功！")
    else:
        rospy.logerr("关节状态设置失败！")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

