#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def draw_line_in_rviz():
    rospy.init_node("line_marker_publisher", anonymous=True)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

    line_marker = Marker()
    line_marker.header.frame_id = "world"
    line_marker.type = Marker.LINE_STRIP  # Can also use Marker.LINE_LIST
    line_marker.action = Marker.ADD

    line_marker.scale.x = 0.01  # Line thickness
    line_marker.color.r = 1.0
    line_marker.color.g = 1.0
    line_marker.color.b = 1.0
    line_marker.color.a = 1.0  # Opacity

    # Define points of the line
    point1 = Point(0.0, 0.0, 0.0)
    point2 = Point(1.0, 1.0, 1.0)

    line_marker.points.append(point1)
    line_marker.points.append(point2)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_pub.publish(line_marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        draw_line_in_rviz()
    except rospy.ROSInterruptException:
        pass
