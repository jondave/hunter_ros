#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
import math
import csv
import datetime

# start time and end time
# total autonomous distance travelled
# locations of interventions (current edge or node)
# time to fix interventions
# which navigation method running when interverntion happened
# ratio of autonomous time vs intervertion time
# calculate mean time/distance between interventions

class SaveCSV:
    
    def __init__(self):

        print("save_csv init")

        self.current_pose = None
        self.previous_pose = None
        self.distance_traveled = 0

        self.Subscriber_Pose = rospy.Subscriber('/robot_pose', Pose, self.pose_callback)
        self.Subscriber_Odom = rospy.Subscriber("odometry/filtered/global", Odometry, self.odometry_callback)

        # run this on ROS shutdown
        rospy.on_shutdown(self.shutdown_callback)

        # Get the current timestamp
        current_time = datetime.datetime.now()
        timestamp = current_time.strftime("%Y%m%d_%H%M%S")

        # Generate the CSV file name with the timestamp
        self.csv_file_name = f'data_{timestamp}.csv'

        # Write headdings to CSV file
        with open(self.csv_file_name, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            # Write column headers
            csv_writer.writerow(['X', 'Y', 'Distance'])
            # Write start timestamp
            csv_writer.writerow(timestamp)

    def pose_callback(self, pose_msg):
        # calculate distacne travelled from pose
        self.previous_pose = self.current_pose
        self.current_pose = pose_msg

        if self.previous_pose is not None:
            # Calculate the Euclidean distance between two poses
            dx = self.current_pose.position.x - self.previous_pose.position.x
            dy = self.current_pose.position.y - self.previous_pose.position.y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            # Add the distance to the total distance traveled
            self.distance_traveled += distance

            # Print the distance traveled
            #print("Distance Traveled: {:.2f} meters".format(self.distance_traveled))
            self.save_to_csv()


    def odometry_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.save_to_csv()

    def save_to_csv(self):
        # Open the CSV file in write mode with newline=''
        with open(self.csv_file_name, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            # Write the data to the CSV file
            combined_data = zip(self.x, self.y, self.distance_traveled)
            csv_writer.writerows(combined_data)
            print("Saved data to csv file: " + str(combined_data))

    def shutdown_callback(self):
        rospy.loginfo("ROS node is shutting down")
        # Write the shutdown timestamp
        shutdown_time = datetime.datetime.now()
        shutdown_timestamp = shutdown_time.strftime("%Y%m%d_%H%M%S")
        with open(self.csv_file_name, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(shutdown_timestamp)


#=========================================
if __name__ == '__main__':
    rospy.init_node('save_csv')
    try:
        saveCSV = SaveCSV()
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass
 
#=========================================