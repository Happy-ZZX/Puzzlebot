#include <multi_points_tracking/multi_points_tracking.h>

MultiPointsTracking::~MultiPointsTracking(){};

MultiPointsTracking::MultiPointsTracking():private_node("~")
{
    ROS_INFO_STREAM("\033[1;32m----> Multi Point tracking started.\033[0m");
    GetParameter();
    LoadGoals(goals_path_);
    is_init_ = false;

    g_robot_vel_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel",1,this);
    Sub_Pose = node_handle.subscribe(pose_topic, 1, &MultiPointsTracking::PoseCallback, this);
    initial_sub = node_handle.subscribe("initial_state", 1, &MultiPointsTracking::InitialCallback, this);
    odom_traj_pub = node_handle.advertise<nav_msgs::Path>("/wheel_odom_traj",10,this);
}
    

void MultiPointsTracking::LoadGoals(const string& goals_path)
{
    ROS_INFO("\n loading from: %s", goals_path.c_str());
    
    std::ifstream f(goals_path);
    std::string line;
    if (!f.good())
    {
        ROS_ERROR("%s - %s", strerror(errno), goals_path.c_str());
        return;
    }
    std::getline(f, line);
    while (std::getline(f, line))
    {
        std::istringstream s(line);
        cout << line << endl;
        cv::Point2d goal;
        char first;
        // Read first character to see whether it's a comment
        if (!(s >> goal.x))
        {
            ROS_ERROR("Not enough data in line: %s; "
                      "No x coordinate provided",
                      line.c_str());
            continue;
        }
        if (!(s >> goal.y))
        {
            ROS_ERROR("Not enough data in line: %s; "
                      "No y coordinate provided",
                      line.c_str());
            continue;
        }
        goals.push_back(goal);
    }
    ROS_INFO("loading %s complete", goals_path.c_str());
}

void MultiPointsTracking::GetParameter()
{
    node_handle.getParam("/multi_points_tracking_node/topic/encoder_wr", encoder_wr_topic);
    node_handle.getParam("/multi_points_tracking_node/topic/encoder_wl", encoder_wl_topic);
    node_handle.getParam("/multi_points_tracking_node/topic/pose_topic", pose_topic);
    node_handle.getParam("/multi_points_tracking_node/odom/kl", kl_);
    node_handle.getParam("/multi_points_tracking_node/odom/kr", kr_);
    node_handle.getParam("/multi_points_tracking_node/odom/b", b_);
    node_handle.getParam("/multi_points_tracking_node/file_path/goals_path_", goals_path_);
    node_handle.getParam("/multi_points_tracking_node/controller/threshold", ARRIVE_THRESHOLD);
    node_handle.getParam("/multi_points_tracking_node/controller/k_d", k_d);
    node_handle.getParam("/multi_points_tracking_node/controller/k_theta", k_theta);
    node_handle.param<double>("odom_pub_rate", odom_pub_rate, 100);
}

void MultiPointsTracking::LeftWheelCallback(const std_msgs::Float32 &encoder_ptr)
{
    //float data = encoder_ptr->data;
    enl = encoder_ptr.data;
    //ROS_INFO("enl=%0.2f", enl);
    ComputePose();
    control_vel();
}

void MultiPointsTracking::RightWheelCallback(const std_msgs::Float32 &encoder_ptr)
{
    //float data = encoder_ptr->data;
    enr = encoder_ptr.data;
    //ROS_INFO("enr=%0.2f", enr);
    control_vel();
    ComputePose();
    control_vel();
}

void MultiPointsTracking::InitialCallback(const std_msgs::Int8 &initial_state)
{
    if (initial_state.data == 1)
    {
        is_init_ = true;
    }
    else{
        return;
    }
}
void MultiPointsTracking::ComputePose()
{
    if (!is_init_ && enl != -0.5 && enr != -0.5)
    {
        last_enl_ = enl;
        last_enr_ = enr;
        last_time_ = ros::Time::now();
        mu_(0, 0) = 0;
        mu_(1, 0) = 0;
        mu_(2, 0) = 0;
        return;
    }
    else if(!is_init_ && enl == 0)
    {
        return;
    }
    else if(!is_init_ && enr == 0)
    {
        return;
    }

    double dt = (ros::Time::now() - last_time_).toSec();
    last_time_ = ros::Time::now();

    double delta_enl = enl * dt;
    double delta_enr = enr * dt;

    double delta_s_l = kl_ * delta_enl;
    double delta_s_r = kr_ * delta_enr;
    //ROS_INFO("b_ =%0.2f", delta_enl);
    double delta_theta = (delta_s_r - delta_s_l) / 2 * b_;
    double delta_s = (delta_s_l + delta_s_r) / 2;
    double temp_angle = mu_(2, 0) + 0.5 * delta_theta;
    mu_(0, 0) += delta_s * cos(temp_angle);
    mu_(1, 0) += delta_s * sin(temp_angle);
    mu_(2, 0) += delta_theta;
    normAngle(mu_(2, 0));

    ROS_INFO("position=(%0.2f,%0.2f) direction=%0.2f", mu_(0,0), mu_(1,0), mu_(2,0));
    last_enl_ = enl;
    last_enr_ = enr;
}

void MultiPointsTracking::control_vel()
{
    geometry_msgs::Twist input;
    if(goal_num < goals.size())
    {
        double ex = goals[goal_num].x - mu_(0);
        //ROS_INFO("goal=(%0.2f, %0.2f)", goals[goal_num].x , goals[goal_num].y);
        //ROS_INFO("goal=(%0.2f, %0.2f)", goals[goal_num].x , goals[goal_num].y);
        double ey = goals[goal_num].y - mu_(1);
        double etheta = atan2(ey , ex);
        normAngle(etheta);
        etheta -= mu_(2);
        double edistance = ex * ex + ey * ey;
        edistance = sqrt(edistance);
        if (edistance < ARRIVE_THRESHOLD)
        {
            goal_num ++;
        }
        else
        {
            input.linear.x = min (k_d * edistance , 1.0);
            input.angular.z = min (k_theta * etheta, 1.0);
            ROS_INFO("Navigation finished");
        }
    }
    else{
        input.linear.x = 0;
        input.angular.z = 0;
    }
    g_robot_vel_pub.publish(input);
}

void MultiPointsTracking::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped &EstimatedPose)
{
    if (is_init_ != 1)
    {
        return;   
    }
    geometry_msgs::Twist input;
    if(goal_num < goals.size())
    {
        double ex = goals[goal_num].x - EstimatedPose.pose.pose.position.x;
        //ROS_INFO("goal=(%0.2f, %0.2f)", goals[goal_num].x , goals[goal_num].y);
        //ROS_INFO("goal=(%0.2f, %0.2f)", goals[goal_num].x , goals[goal_num].y);
        double ey = goals[goal_num].y - EstimatedPose.pose.pose.position.y;
        double etheta = atan2(ey , ex);
        etheta -= 2 * asin(EstimatedPose.pose.pose.orientation.z);
        normAngle(etheta);
        double edistance = ex * ex + ey * ey;
        edistance = sqrt(edistance);
        if (edistance < ARRIVE_THRESHOLD)
        {
            ROS_INFO("goal=(%0.2f, %0.2f) fnished", goals[goal_num].x , goals[goal_num].y);
            goal_num ++;
        }
        else
        {
            input.linear.x = min (k_d * edistance , 1.0);
            input.angular.z = min (k_theta * etheta, 1.0);
            //ROS_INFO("Navigation finished");
        }
    }
    else{
        input.linear.x = 0;
        input.angular.z = 0;
        ROS_INFO("Navigation finished");
    }
    ros::Time time = ros::Time::now();
    odom_path.header.frame_id = "world";
    odom_path.header.stamp = time;
    geometry_msgs::PoseStamped odom_msg;
    odom_msg.header.frame_id = "world";
    odom_msg.header.stamp = time;
    odom_msg.pose.position.x = EstimatedPose.pose.pose.position.x;
    odom_msg.pose.position.y = EstimatedPose.pose.pose.position.y;
    odom_msg.pose.position.z = 0.;
    tf2::Quaternion q;
    q.setRPY(0.,0.,2 * asin(EstimatedPose.pose.pose.orientation.z));
    q.normalize();
    odom_msg.pose.orientation = tf2::toMsg(q);
    odom_path.poses.push_back(odom_msg);
    odom_traj_pub.publish(odom_path);
    g_robot_vel_pub.publish(input);
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "start_tracking");
    MultiPointsTracking tracking;
    ros::spin();
    return 0;
}