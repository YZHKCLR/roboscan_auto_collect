#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_aubo_joints():
    # Initialize the node
    rospy.init_node("aubo_joint_control_node", anonymous=True)

    # Create a publisher to the joint trajectory controller
    pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)

    # Wait for the publisher to be ready
    rospy.sleep(1)

    # Define the joint names (adjust if necessary based on your controller)
    joint_names = [
        "shoulder_joint",
        "upperArm_joint",
        "foreArm_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint"
    ]

    # Create a JointTrajectory message
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = joint_names

    # Define target joint positions (in radians)
    target_joint_positions = [0.0, -1.0, 1.0, -1.0, 0.5, 0.0]

    # Create a JointTrajectoryPoint
    point = JointTrajectoryPoint()
    point.positions = target_joint_positions
    point.time_from_start = rospy.Duration(2.0)  # Move within 2 seconds

    # Add the point to the trajectory message
    trajectory_msg.points.append(point)

    # Publish the joint trajectory message
    rospy.loginfo("Sending joint trajectory command...")
    pub.publish(trajectory_msg)

    # Keep the node alive until the movement completes
    rospy.sleep(2.5)

if __name__ == "__main__":
    try:
        move_aubo_joints()
    except rospy.ROSInterruptException:
        pass
