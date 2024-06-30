#ifndef ARUCO_EKF_SLAM_H
#define ARUCO_EKF_SLAM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/aruco.hpp>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "tf2_ros/buffer.h"
#include <tf2/buffer_core.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_eigen/tf2_eigen.h>

using namespace Eigen;
using namespace cv;
using namespace std;

class ArucoMarker
{
public:
    ArucoMarker() {}
    ArucoMarker(const int &aruco_id, const double &x, const double &y, const double &theta, const Eigen::Matrix3d &observe_covariance)
        : aruco_id_(aruco_id), x_(x), y_(y), theta_(theta), observe_covariance_(observe_covariance), aruco_index_(-1) {}

    int aruco_id_;
    int aruco_index_;
    double x_;
    double y_;
    double theta_;
    Eigen::Matrix3d observe_covariance_;
    Eigen::Vector3d last_observation_; /**< Value of last observation contain delta_x, delta_y, delta_theta */
    /**
     * @brief used for the sort of the detected markers, the markers will be sorted according to
     * the sequence of been added to the map
     */
    friend bool operator<(const ArucoMarker &a, const ArucoMarker &b)
    {
        return a.aruco_index_ > b.aruco_index_;
    }

    friend bool operator==(const ArucoMarker &a, const ArucoMarker &b)
    {
        return a.aruco_id_ == b.aruco_id_;
    }
}; // an obervation contains: aruco_id, r, phi.

class ArUcoEKFSLAM
{
private:
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node;

    ros::Publisher g_landmark_pub;          /**< publish detected map */
    ros::Publisher g_robot_pose_pub;        /**< publish current robot pose */
    ros::Publisher g_detected_markers_pub;  /**< publish markers detected now */
    ros::Publisher g_true_pose_pub;         /**< publish true pose*/
    ros::Publisher initial_state_pub;
    image_transport::Publisher g_img_pub;   /**< publish detected image */

    ros::Subscriber encoder_sub;
    ros::Subscriber image_sub;
    ros::Subscriber true_sub;

    tf2_ros::Buffer g_tfBuffer_;            /**< get robot parameters */
    string image_topic, encoder_topic;

    // whether the SLAM system is initiated
    bool is_init_;
    bool camera_inited_ = false;

    // the system parameters which is read from default.yaml
    cv::Mat K_, dist_;                    
    double k_r_;            
    double k_phi_;          

    int n_markers_;
    int marker_size_;
    double marker_length_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat marker_img_;
    visualization_msgs::MarkerArray detected_markers_;
    visualization_msgs::MarkerArray detected_map_;
    std::map<int, int> aruco_id_map; // pair<int, int>{aruco_id, position_i}
    std::vector<ArucoMarker> last_observed_marker_;
    std::vector<cv::Point3f> objectPoints_ = {cv::Vec3f(-marker_length_ / 2.f, marker_length_ / 2.f, 0), cv::Vec3f(marker_length_ / 2.f, marker_length_ / 2.f, 0), cv::Vec3f(marker_length_ / 2.f, -marker_length_ / 2.f, 0), cv::Vec3f(-marker_length_ / 2.f, -marker_length_ / 2.f, 0)};

    // the mean and covariance of extended state 
    Eigen::MatrixXd mu_;            
    Eigen::MatrixXd sigma_;     
    std::vector<int> aruco_ids_; 

    double Q_k,                                           /**< Error coefficient of encoder */
        R_x,                                              /**< Error coefficient of observation x */
        R_y,                                              /**< Error coefficient of observation y */
        R_theta;                                          /**< Error coefficient of observation y */
    double kl,                                            /**< Left wheel radius */
        kr,                                               /**< Right wheel radius */
        b;                                                /**< Half of robot wheelbase */
    ros::Time last_time_;
    ros::Time start_time_;
    float USEFUL_DISTANCE_THRESHOLD_;
    Eigen::Matrix4d T_r_c_; 
    std::vector<double> r2c_translation;
    std::vector<double> r2c_rotation;
    geometry_msgs::TransformStamped transformStamped_r2c; /**< Calculated transfor matrix for robot base to camera optical */
    double marker_length;                                 /**< Length of the aruco markers */
    int markers_dictionary;                               /**< \enum cv::aruco::PREDEFINED_DICTIONARY_NAME */
    geometry_msgs::PoseStamped True_pose_;

    void GetTopicName();

    void GetParameter();

    void addEncoder(const double &enl, const double &enr);

    void addImage(const Mat &img);

    bool GenerateMarker(int id, double length, double x, double y, double z, tf2::Quaternion q, visualization_msgs::Marker &marker_, std_msgs::ColorRGBA color, ros::Duration lifetime);

    int getObservations(const cv::Mat &img, std::vector<ArucoMarker> &obs);

    void getTransformStamped(const std::string &target_frame, const std::string &source_frame, geometry_msgs::TransformStamped &transform_stamped);

    void CalculateCovariance(const cv::Vec3d &tvec, const cv::Vec3d &rvec, const std::vector<cv::Point2f> &marker_corners, Eigen::Matrix3d &covariance);

    void fillTransform(tf2::Transform &transform_, const cv::Vec3d &rvec, const cv::Vec3d &tvec);

    bool checkLandmark(const int &aruco_id, int &landmark_idx);

    void setCameraParameters(const std::pair<cv::Mat, cv::Mat> &cameraparameters)
    {
        K_ = cameraparameters.first;
        dist_ = cameraparameters.second;
    }

    void normAngle(double &angle)
    {
        const static double PI = 3.1415926;
        static double Two_PI = 2.0 * PI;
        if (angle >= PI)
            angle -= Two_PI;
        if (angle < -PI)
            angle += Two_PI;
    }

    visualization_msgs::MarkerArray toRosDetectedMarkers() { return detected_markers_; };

    visualization_msgs::MarkerArray toRosMarker(double scale);

    geometry_msgs::PoseWithCovarianceStamped toRosPose()
    {
        geometry_msgs::PoseWithCovarianceStamped rpose;
        rpose.header.frame_id = "world";
        rpose.pose.pose.position.x = mu_(0);
        rpose.pose.pose.position.y = mu_(1);
        rpose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mu_(2));

        rpose.pose.covariance.at(0) = sigma_(0, 0);
        rpose.pose.covariance.at(1) = sigma_(0, 1);
        rpose.pose.covariance.at(6) = sigma_(1, 0);
        rpose.pose.covariance.at(7) = sigma_(1, 1);
        rpose.pose.covariance.at(5) = sigma_(0, 2);
        rpose.pose.covariance.at(30) = sigma_(2, 0);
        rpose.pose.covariance.at(35) = sigma_(2, 2);
        rpose.header.stamp = ros::Time::now();
        return rpose;
    }

    Eigen::MatrixXd &mu() { return mu_; }
    Eigen::MatrixXd &sigma() { return sigma_; }
    cv::Mat markedImg() { return marker_img_; }

public:
    ArUcoEKFSLAM();

    ~ArUcoEKFSLAM(){};

    void ImageCallback(const sensor_msgs::ImageConstPtr &img_ptr);

    //void ImageCallback(const sensor_msgs::ImageConstPtr &img_ptr, const sensor_msgs::CameraInfoConstPtr &cinfo);

    void EncoderCallback(const geometry_msgs::QuaternionStamped::ConstPtr &encoder_ptr);

    void TrueCallback(const geometry_msgs::PoseStamped &true_ptr);
};

#endif