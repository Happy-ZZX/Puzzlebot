#include<multi_points_tracking/Encoder_test.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_tracking");
    EncoderTest test;
    ros::spin();
    return 0;
}


EncoderTest::~EncoderTest(){};

EncoderTest::EncoderTest():private_node("~")
{
    ROS_INFO_STREAM("\033[1;32m----> Multi Point tracking started.\033[0m");
    is_init_ = false;
    Sub_Pose = node_handle.subscribe("/robot_posewithcovariance", 1, &EncoderTest::PoseCallback, this);
    Sub_End = node_handle.subscribe("/test_end",1, &EncoderTest::EndCallback, this);

    Sub_True = node_handle.subscribe("/true_pose",1, &EncoderTest::TrueCallback, this);
    g_robot_vel_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel",1,this);
}

void EncoderTest::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped &EstimatedPose)
{
    double x = EstimatedPose.pose.pose.position.x;
    double y = EstimatedPose.pose.pose.position.y;
    d_ = sqrt(x*x + y*y);
    angle_ = EstimatedPose.pose.pose.orientation.z;
}

void EncoderTest::TrueCallback(const geometry_msgs::PoseStamped &true_ptr)
{
    geometry_msgs::PoseStamped True_pose_ = true_ptr;
    double x = true_ptr.pose.position.x;
    double y = true_ptr.pose.position.y;
    t_d_ = sqrt(x * x + y * y);

    double w = true_ptr.pose.orientation.w;
    double x2 = true_ptr.pose.orientation.x;
    double y2 = true_ptr.pose.orientation.y;
    double z = true_ptr.pose.orientation.z;
    Eigen::Quaterniond quaternion(w,x2,y2,z);
    Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(0,1,2);
    t_angle_ = eulerAngle(2);
}

void EncoderTest::EndCallback(const std_msgs::Int8 &end_state)
{
    if (end_state.data == 1)
    {
        ROS_INFO("Test finished");
        ROS_ERROR_STREAM("Finishe <<<<<<<<<<<<<<<<<<<<" <<  std::endl);
        const static double PI = 3.1415926;
        static double Two_PI = 2.0 * PI;
        angle_ = angle_ * Two_PI;

        if (angle_ >= PI)
            angle_ -= Two_PI;
        if (angle_ < -PI)
            angle_ += Two_PI;

        ROS_INFO_STREAM("The esitimated pose is d =" << d_ << " angle = " << angle_ << endl);

        if (t_angle_ >= PI)
            t_angle_ -= Two_PI;
        if (t_angle_ < -PI)
            t_angle_ += Two_PI;
        ROS_INFO_STREAM("The real pose is d =" << t_d_ << " angle = " << (t_angle_ + PI););
    }
}


void EncoderTest::Test()
{
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
    ROS_INFO("Test finished");
    ROS_ERROR_STREAM("Finishe <<<<<<<<<<<<<<<<<<<<" <<  std::endl);
    ROS_INFO_STREAM("The esitimated pose is d =" << d_ << " angle = " << angle_ << endl);
    ROS_INFO_STREAM("The real pose is d =" << t_d_ << "angle is " << t_angle_;);
}