#ifndef ENCODER_TEST_H
#define ENCODER_TEST_H
#include <string>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

using namespace std;

class EncoderTest
{
public:
    EncoderTest();
    ~EncoderTest();
    
    void Test();
private:
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node;

    ros::Subscriber Sub_Pose, Sub_True,Sub_End;
    ros::Publisher g_robot_vel_pub;

    double d_, angle_;
    double t_d_, t_angle_;
    bool is_init_;

    void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped &EstimatedPose);

    void TrueCallback(const geometry_msgs::PoseStamped &true_ptr);

    void EndCallback(const std_msgs::Int8 &end_state);
};

#endif