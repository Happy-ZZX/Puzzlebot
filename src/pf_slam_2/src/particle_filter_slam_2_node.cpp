#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "pf_slam_2/particle_filter_slam_2.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reslam_odom");
    ParticleFilter pfslam;
    ros::spin();
    return 0;
}