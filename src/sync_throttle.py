#!/usr/bin/env python3
# coding=utf-8

"""
Synchronized image throttling node for stereo cameras.
This node ensures that left and right camera images are synchronized
and published at the desired throttle rate (approximately 4Hz).
"""

import rospy
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import cv2
import time

class SynchronizedThrottle:
    def __init__(self):
        # Initialize the node
        rospy.init_node('sync_throttle_node')
        
        # Get parameters
        self.throttle_rate = rospy.get_param('~throttle_rate', 4.0)  # Hz
        self.left_image_topic = rospy.get_param('~left_image_topic', '/stereo_camera/left')
        self.right_image_topic = rospy.get_param('~right_image_topic', '/stereo_camera/right')
        self.throttled_left_topic = rospy.get_param('~throttled_left_topic', '/stereo_camera/left/throttled')
        self.throttled_right_topic = rospy.get_param('~throttled_right_topic', '/stereo_camera/right/throttled')
        
        rospy.loginfo(f"Input left topic: {self.left_image_topic}")
        rospy.loginfo(f"Input right topic: {self.right_image_topic}")
        rospy.loginfo(f"Output left topic: {self.throttled_left_topic}")
        rospy.loginfo(f"Output right topic: {self.throttled_right_topic}")
        rospy.loginfo(f"Throttle rate: {self.throttle_rate} Hz")
        
        # Create publishers for throttled images
        self.left_pub = rospy.Publisher(self.throttled_left_topic, Image, queue_size=5)
        self.right_pub = rospy.Publisher(self.throttled_right_topic, Image, queue_size=5)
        
        # Set up time synchronizer for the two cameras
        self.left_sub = message_filters.Subscriber(self.left_image_topic, Image)
        self.right_sub = message_filters.Subscriber(self.right_image_topic, Image)
        
        # Use ApproximateTimeSynchronizer to sync messages that arrive at approximately the same time
        # Queue size 10, slop 0.05 seconds (50ms)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], 10, 0.001)
        self.ts.registerCallback(self.sync_callback)
        
        # Keep track of last publishing time to throttle
        self.last_publish_time = 0
        self.min_publish_interval = 1.0 / self.throttle_rate
        
        rospy.loginfo("Synchronized throttle node initialized successfully")
    
    def sync_callback(self, left_msg, right_msg):
        # Check if enough time has passed to publish again
        current_time = time.time()
        if current_time - self.last_publish_time < self.min_publish_interval:
            return
        
        # Publish synchronized images
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        
        # Update the last publish time
        self.last_publish_time = current_time
        
        # Log publishing at debug level
        rospy.logdebug(f"Published synchronized images at time {rospy.Time.now().to_sec()}")
        rospy.logdebug(f"Left image timestamp: {left_msg.header.stamp.to_sec()}")
        rospy.logdebug(f"Right image timestamp: {right_msg.header.stamp.to_sec()}")

if __name__ == '__main__':
    try:
        node = SynchronizedThrottle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
