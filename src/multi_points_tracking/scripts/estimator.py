#! /usr/bin/env python

import math
import threading
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
from std_msgs.msg import Float32, Bool

class Estimator:
    """To estimate the robot pose.

    This class response to: receive the encoder data and estimate the robot pose using dead-rocking, 
                            reset the estimater state

    Attributes:
        basewidth: Half of the distance between two wheels
        l: The distance between two wheels.
        r: The wheel radius. 
        k: The covariance coefficient
    """
    def __init__(self, basewidth, r):
        self.l = basewidth*2
        self.r = r
        self.X = 0
        self.Y = 0
        self.Theta = 0
        self.sigma = np.zeros((3,3))
        self.w_l = 0
        self.w_r = 0
        self._value_lock = threading.Lock()
        self.k = 0.01
        rospy.init_node('Estimator', anonymous=True)
        self.last_time = rospy.Time.now()
        rospy.Subscriber("/wl", Float32, self.left_wheel_callback)
        rospy.Subscriber("/wr", Float32, self.right_wheel_callback)
        rospy.Subscriber("/reset", Bool, self.reset_callback)
        self.pub = rospy.Publisher('/robot_posewithcovariance', PoseWithCovarianceStamped, queue_size=10)
        self.estiamted_pose = PoseWithCovarianceStamped()
        self.estiamted_pose.pose.pose.position.z = self.r
        self.estiamted_pose.pose.pose.orientation.w = 1
        self.estiamted_pose.header.frame_id = "world"

    def norm_angle(self, angle):
        """Swap the angle into [-pi, pi).

        Args:
            angle: The angle need to be swapped.

        Returns:
            The angle has been sawpped into [-pi, pi).
        """
        while angle >= math.pi:
            angle = angle - 2*math.pi
        while angle < -math.pi:
            angle = angle + 2*math.pi
        return angle

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
            self.X = 0
            self.Y = 0
            self.Theta = 0
            self.sigma = np.zeros((3,3))        

    def update(self, wr, wl):
        """Update the class properties(robot state).

        This method is used to update the class properties(robot state) according to the 
        new encoders data, using dead-rockning method and calculate the estimation covariance.

        Args:
            wr: The newest right encoder data.
            wl: The newest left encoder data.

        """
        dt = (rospy.Time.now() - self.last_time).to_sec()
        self.last_time = rospy.Time.now()
        dl = dt * self.r * wl # change of angle
        dr = dt * self.r * wr
        delta_theta = (dr - dl) / self.l
        delta_s = 0.5 * (dr + dl)
        cos_th = math.cos(self.Theta)
        sin_th = math.sin(self.Theta)
        self.X += delta_s * cos_th
        self.Y += delta_s * sin_th
        self.Theta += delta_theta
        self.Theta = self.norm_angle(self.Theta) # norm
        
        # Calculate the covariance
        H_k = np.eye(3)
        H_k[0,2] = -delta_s * sin_th
        H_k[1,2] = delta_s*cos_th
        Gup = np.zeros([3,2])
        Gup[0,:] = cos_th
        Gup[1,:] = sin_th
        Gup[2,0] = 2/self.l
        Gup[2,1] = -2/self.l
        Gup = Gup * 0.5 * dt * self.r
        sigma_u = np.eye(2)
        sigma_u[0,0] = self.k * abs(wr)
        sigma_u[1,1] = self.k * abs(wl)
        self.sigma = np.dot(np.dot(H_k, self.sigma), H_k.T) + np.dot(np.dot(Gup, sigma_u), Gup.T)
        
        #Update the PoseWithCovarianceStamped message for Rviz visualization
        self.estiamted_pose.header.stamp = rospy.Time.now()
        self.estiamted_pose.pose.pose.position.x = self.X
        self.estiamted_pose.pose.pose.position.y = self.Y
        self.estiamted_pose.pose.pose.orientation.z = math.sin(self.Theta/2)
        self.estiamted_pose.pose.pose.orientation.w = math.cos(self.Theta/2)
        self.estiamted_pose.pose.covariance[0] = self.sigma[0,0]
        self.estiamted_pose.pose.covariance[1] = self.sigma[0,1]
        self.estiamted_pose.pose.covariance[6] = self.sigma[1,0]
        self.estiamted_pose.pose.covariance[7] = self.sigma[1,1]
        self.estiamted_pose.pose.covariance[5] = self.sigma[0,2]
        self.estiamted_pose.pose.covariance[30] = self.sigma[2,0]
        self.estiamted_pose.pose.covariance[35] = self.sigma[2,2]

if __name__ == "__main__":
    try:
        estimator = Estimator(0.09, 0.05)
        while not rospy.is_shutdown():
            estimator.update(estimator.w_r, estimator.w_l)
            estimator.pub.publish(estimator.estiamted_pose)  
            rospy.Rate(10).sleep()   #10Hz (1/10s delay)
    except rospy.ROSInterruptException:
        pass

    