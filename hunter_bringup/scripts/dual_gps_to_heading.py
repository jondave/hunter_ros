#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose, Point, Quaternion
import math

class DualGNSSHeading:
    
    def __init__(self):

        print("dual_gps_to_heading init")

        self.Subscriber_Gps1 = rospy.Subscriber("gps/fix", NavSatFix, self.callback_gps1)
        self.Subscriber_Gps2 = rospy.Subscriber("gps_front/fix", NavSatFix, self.callback_gps2)
        self.publisher = rospy.Publisher("robot_pose", Pose, queue_size=10)

        self.lat1 = 0.0
        self.long1 = 0.0
        self.lat2 = 0.0
        self.long2 = 0.0
        
    def callback_gps1(self, data):
        self.lat1 = data.latitude
        self.long1 = data.longitude

    def callback_gps2(self, data):
        self.lat2 = data.latitude
        self.long2 = data.longitude

        # https://stackoverflow.com/questions/45676694/calculate-heading-with-two-gps-devices
        if self.lat1 != 0.0 and self.long1 != 0.0 and self.lat2 != 0.0 and self.long2 != 0.0:
            d_long = self.long2 - self.long1
            y = math.sin(math.radians(d_long)) * math.cos(math.radians(self.lat2))
            x = math.cos(math.radians(self.lat1)) * math.sin(math.radians(self.lat2)) - math.sin(math.radians(self.lat1)) * math.cos(math.radians(self.lat2)) * math.cos(math.radians(d_long))
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
            self.publisher.publish(heading_pose)
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