#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
import math

class DualGNSSHeading:
    
    def __init__(self):

        print("dual_gps_to_heading init")

        self.Subscriber_Gps_front_left = rospy.Subscriber("gps_front/fix", NavSatFix, self.callback_gps_front_left)
        self.Subscriber_Gps_rear_right = rospy.Subscriber("gps/fix", NavSatFix, self.callback_gps_rear_right)
        self.pose_publisher = rospy.Publisher("robot_pose", Pose, queue_size=10)
        self.heading_marker_pub = rospy.Publisher("/heading_marker", Marker, queue_size=10) # vivisualise the heading as a marker in rviz


        self.lat_front_left = 0.0
        self.long_front_left = 0.0
        self.lat_rear_right = 0.0
        self.long_rear_right = 0.0
        
    def callback_gps_front_left(self, data):
        self.lat_front_left = data.latitude
        self.long_front_left = data.longitude

    def callback_gps_rear_right(self, data):
        self.lat_rear_right = data.latitude
        self.long_rear_right = data.longitude

        # https://stackoverflow.com/questions/45676694/calculate-heading-with-two-gps-devices
        if self.lat_front_left != 0.0 and self.long_front_left != 0.0 and self.lat_rear_right != 0.0 and self.long_rear_right != 0.0:
            d_long = self.long_rear_right - self.long_front_left
            y = math.sin(math.radians(d_long)) * math.cos(math.radians(self.lat_rear_right))
            x = math.cos(math.radians(self.lat_front_left)) * math.sin(math.radians(self.lat_rear_right)) - math.sin(math.radians(self.lat_front_left)) * math.cos(math.radians(self.lat_rear_right)) * math.cos(math.radians(d_long))
            heading = math.degrees(math.atan2(y, x))

            # Create a Pose message to represent the heading
            heading_pose = Pose()
            heading_pose.position = Point(0.0, 0.0, 0.0)  # Set the position to (0, 0, 0)
            heading_quaternion = Quaternion()
            heading_quaternion.x = 0.0
            heading_quaternion.y = 0.0
            heading_quaternion.z = math.sin(math.radians(heading) / 2.0)
            heading_quaternion.w = math.cos(math.radians(heading) / 2.0)
            heading_pose.orientation = heading_quaternion

            # Publish the heading as a Pose
            self.pose_publisher.publish(heading_pose)

            # Create a Marker message to represent the heading as an arrow in rviz
            heading_marker = Marker()
            heading_marker.header.frame_id = "base_link"
            heading_marker.header.stamp = rospy.Time.now()
            heading_marker.type = Marker.ARROW
            heading_marker.action = Marker.ADD
            heading_marker.pose = heading_pose
            heading_marker.scale.x = 1.0  # Set the scale as needed for the arrow
            heading_marker.scale.y = 0.1
            heading_marker.scale.z = 0.1
            heading_marker.color.a = 1.0  # Set the alpha (transparency)
            heading_marker.color.r = 0.0  # Set the color (red)
            heading_marker.color.g = 0.0  # Set the color (green)
            heading_marker.color.b = 1.0  # Set the color (blue)

            # Publish the heading as a Marker
            self.heading_marker_pub.publish(heading_marker)
        else:            
            print("Error dual_gps_to_heading Lat Long is 0.0")

#=========================================
if __name__ == '__main__':
    rospy.init_node('dual_gnss_heading')
    try:
        dualGNSSHeading = DualGNSSHeading()
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass
 
#=========================================