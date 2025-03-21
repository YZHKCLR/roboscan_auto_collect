#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

def publish_frame_marker():
    rospy.init_node("frame_marker_publisher", anonymous=True)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set the position and orientation
        marker.pose.position.x = 3.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0

        # Set the scale and color
        marker.scale.x = 0.5  # Length of the arrow
        marker.scale.y = 0.1  # Arrow thickness
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Opacity

        marker_pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_frame_marker()
    except rospy.ROSInterruptException:
        pass
