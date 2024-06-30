#! /usr/bin/env python

import math
import threading
import numpy as np
from geometry_msgs.msg import QuaternionStamped
import rospy
from std_msgs.msg import Float32, Bool

class EncoderEstimator:
    """To estimate the encoder information.

    This class response to: receive the encoder data and eublish it in the same time,
                            reset the estimater state
    """
    def __init__(self):
        self.w_l = 0
        self.w_r = 0
        self._value_lock = threading.Lock()
        rospy.init_node('EncoderEstimator', anonymous=True)
        rospy.Subscriber("/wl", Float32, self.left_wheel_callback)
        rospy.Subscriber("/wr", Float32, self.right_wheel_callback)
        rospy.Subscriber("/reset", Bool, self.reset_callback)
        self.pub = rospy.Publisher('/encoder', QuaternionStamped, queue_size=10)
        self.etimated_encoder = QuaternionStamped()

    def left_wheel_callback(self, data):
        """Store the left encoder data into the class variable
        """
        with self._value_lock:
            self.w_l = data.data

    def right_wheel_callback(self, data):
        """Store the right encoder data into the class variable
        """
        with self._value_lock:
            self.w_r = data.data

    def reset_callback(self, data):
        """Reset the estimator after receive reset message
        """
        if data.data:
            self.w_r = 0
            self.w_l = 0

    def update(self):
        self.etimated_encoder.quaternion.x = self.w_r
        self.etimated_encoder.quaternion.y = self.w_l
        self.pub.publish(self.etimated_encoder)

if __name__ == "__main__":
    try:
        estimator = EncoderEstimator()
        while not rospy.is_shutdown():
            estimator.update()
            rospy.Rate(20).sleep()  # 10Hz (1/10s delay)
    except rospy.ROSInterruptException:
        pass
