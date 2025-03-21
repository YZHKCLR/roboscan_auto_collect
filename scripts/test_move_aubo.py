#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose

def move_aubo_to_pose():
    # Initialize the node
    rospy.init_node("aubo_move_node", anonymous=True)

    # Initialize MoveIt commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create a MoveGroupCommander for the manipulator
    group = moveit_commander.MoveGroupCommander("manipulator")
    '''# motion planning in cartesian space
    # Set the reference frame
    group.set_pose_reference_frame("world")

    # Allow replanning if the motion plan fails
    group.allow_replanning(True)

    # Set tolerance levels
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.01)

    # Define the target pose
    target_pose = Pose()
    target_pose.orientation.w = 1.0
    target_pose.position.x = 0.4  # Adjust as needed
    target_pose.position.y = 0.2  # Adjust as needed
    target_pose.position.z = 0.5  # Adjust as needed

    # Set the target pose
    group.set_pose_target(target_pose)

    # Plan and execute the motion
    rospy.loginfo("Planning and executing the motion...")
    plan = group.go(wait=True)

    # Check if the motion was successful
    if plan:
        rospy.loginfo("Motion executed successfully!")
    else:
        rospy.logerr("Motion execution failed.")

    # Stop the robot and clear targets
    group.stop()
    group.clear_pose_targets()
    # motion planning in cartesian space'''


    # Get the current joint values
    current_joints = group.get_current_joint_values()
    rospy.loginfo(f"Current joint values: {current_joints}")

    # Define the target joint positions (in radians)
    # Example: [shoulder_joint, upper_arm_joint, forearm_joint, wrist1_joint, wrist2_joint, wrist3_joint]
    target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Set the target joint values
    group.go(target_joint_positions, wait=True)

    # Ensure that the robot has stopped moving
    group.stop()

    # Get and log the final joint positions
    final_joints = group.get_current_joint_values()
    rospy.loginfo(f"Final joint values: {final_joints}")





    # Shut down MoveIt commander
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        move_aubo_to_pose()
    except rospy.ROSInterruptException:
        pass
