#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <std_msgs/Int8.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start");
    ros::Publisher g_robot_vel_pub, end_state_pub;
    ros::NodeHandle node_handle;

    g_robot_vel_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    end_state_pub = node_handle.advertise<std_msgs::Int8>("/test_end", 1);

    ros::Time time = ros::Time::now();
    ros::Time time2 = ros::Time::now();
    while ((time2 - time).toSec() <= 10)
    {
        geometry_msgs::Twist input;
        input.linear.x = 0.1;
        input.angular.z = 0.0;
        g_robot_vel_pub.publish(input);
        time2 = ros::Time::now();
    }

    geometry_msgs::Twist input;
    input.linear.x = 0.0;
    input.angular.z = 0.0;
    g_robot_vel_pub.publish(input);

    std_msgs::Int8 msg;
    msg.data = 1;
    end_state_pub.publish(msg);

    return 0;
}