#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageCombinerNode:
    def __init__(self):
        rospy.init_node('image_combiner_node', anonymous=True)

        self.bridge = CvBridge()

        # Subscribe to the two image topics
        self.image1_sub = rospy.Subscriber('/image_topic1', Image, self.image1_callback)
        self.image2_sub = rospy.Subscriber('/image_topic2', Image, self.image2_callback)

        # Publish the combined image
        self.combined_image_pub = rospy.Publisher('/combined_image_topic', Image, queue_size=10)

    def image1_callback(self, data):
        # Convert the ROS image message to OpenCV format
        self.cv_image1 = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Process the image as needed
        # ...

        # Combine the processed image with the other image
        # combined_image = cv2.hconcat([self.cv_image1, self.cv_image2])

        # Convert the combined image back to ROS format
        # combined_image_msg = self.bridge.cv2_to_imgmsg(combined_image, 'bgr8')

        # Publish the combined image
        # self.combined_image_pub.publish(combined_image_msg)

    def image2_callback(self, data):
        # Convert the ROS image message to OpenCV format
        self.cv_image2 = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Process the image as needed
        # ...

        # Combine the processed image with the other image
        combined_image = cv2.hconcat([self.cv_image1, self.cv_image2])

        # Convert the combined image back to ROS format
        combined_image_msg = self.bridge.cv2_to_imgmsg(combined_image, 'bgr8')

        # Publish the combined image
        self.combined_image_pub.publish(combined_image_msg)

    def run(self):
        # Run the node until it's shutdown
        rospy.spin()

if __name__ == '__main__':
    image_combiner_node = ImageCombinerNode()
    image_combiner_node.run()
