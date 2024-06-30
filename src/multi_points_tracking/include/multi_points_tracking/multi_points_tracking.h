#ifndef MULTI_POINTS_TRACKING_H
#define MULTI_POINTS_TRACKING_H
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

class MultiPointsTracking
{
public:
    MultiPointsTracking();
    ~MultiPointsTracking();
    /**
    * @brief Load the goals
    *
    * @param goals_path the path of goals
    */
    void LoadGoals(const string& goals_path);
    /**
    * @brief encoder callback
    *
    * @param encoder_ptr the ptr of encoder
    */
    void LeftWheelCallback(const std_msgs::Float32 &encoder_ptr);
    /**
    * @brief left encoder callback
    *
    * @param encoder_ptr the ptr of encoder
    */
    void RightWheelCallback(const std_msgs::Float32 &encoder_ptr);
    /**
    * @brief calculate the pose using the encoder data
    *
    */
    void ComputePose();
    /**
    * @brief get all parameters
    * 
    */
    void GetParameter();
    /**
    * @brief add the Encoder data
    *
    * @param enl 
    * @param enr
    */
    void addEncoder(const double &enl, const double &enr);
    /**
    * @brief get all parameters
    */
    void control_vel();
    /**
    * @brief pose callback function
    * 
    * @param EstimatedPose
    */
    void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped &EstimatedPose);
    /**
    * @brief pose callback function
    * 
    * @param EstimatedPose
    */
    void InitialCallback(const std_msgs::Int8 &initial_state);
private:
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node;
    ros::Publisher g_robot_vel_pub;
    ros::Publisher odom_traj_pub;
    ros::Subscriber Sub_Pose;
    ros::Subscriber encoder_wr;
    ros::Subscriber encoder_wl;
    ros::Subscriber initial_sub;
    
    string goals_path_;
    string encoder_wr_topic, encoder_wl_topic;
    string pose_topic;
    double kl_, kr_, b_;
    int goal_num = 0;
    double odom_pub_rate;
    // whether the system is initiated
    bool is_init_;
    std::vector<cv::Point2d> goals;

    // last encoder values (left & right)
    double enl; 
    double enr;
    ros::Time last_time_;
    double last_enl_, last_enr_;

    Eigen::Vector3d mu_;
    nav_msgs::Path odom_path;

    double ARRIVE_THRESHOLD;
    double k_d, k_theta;
    void normAngle(double &angle)
    {
        const static double PI = 3.1415926;
        static double Two_PI = 2.0 * PI;
        if (angle >= PI)
            angle -= Two_PI;
        if (angle < -PI)
            angle += Two_PI;
    }
};

#endif