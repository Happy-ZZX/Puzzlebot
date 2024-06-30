#ifndef ARUCO_PF_SLAM_H
#define ARUCO_PF_SLAM_H

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

#include "pf_slam/particle.h"

using namespace Eigen;
using namespace cv;
using namespace std;

class ParticleFilter
{
private:
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node;

    ros::Publisher g_landmark_pub;          /**< publish detected map */
    ros::Publisher g_robot_pose_pub;        /**< publish current robot pose */
    ros::Publisher g_true_pose_pub;         /**< publish true pose*/
    ros::Publisher initial_state_pub;
    image_transport::Publisher g_img_pub;   /**< publish detected image */

    ros::Subscriber encoder_sub;
    ros::Subscriber image_sub;
    ros::Subscriber true_sub;

    tf2_ros::Buffer g_tfBuffer_;            /**< get robot parameters */
    string image_topic, encoder_topic;

    bool is_init_;
    // the mean and covariance of extended state 
    Eigen::Vector3d mu_;                    /**< Mean of state 3 */
    Eigen::Matrix3d sigma_;                 /**< Covariance of state */
    std::vector<int> aruco_ids_; 
    int N;                                  /**< Number of particles*/
    struct Particle BestParticle;           /**< Partcle with greatest weight*/ 
    int BestParticle_ID;
    std::vector<struct Particle> Particles; /**< Particles*/
    std::map<int, int> aruco_id_map; // pair<int, int>{aruco_id, position_i}
    
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat marker_img_;
    std::vector<ArucoMarker> last_observed_marker_;
    std::vector<cv::Point3f> objectPoints_ = {cv::Vec3f(-marker_length / 2.f, marker_length / 2.f, 0), cv::Vec3f(marker_length / 2.f, marker_length / 2.f, 0), cv::Vec3f(marker_length / 2.f, -marker_length / 2.f, 0), cv::Vec3f(-marker_length / 2.f, -marker_length / 2.f, 0)};

    // the system parameters which is read from default.yaml
    cv::Mat K_, dist_;
    double kl,                                            /**< Left wheel radius */
        kr,                                               /**< Right wheel radius */
        b;                                                /**< Half of robot wheelbase */
    ros::Time last_time_;
    float USEFUL_DISTANCE_THRESHOLD_;
    std::vector<double> r2c_translation;
    std::vector<double> r2c_rotation;
    geometry_msgs::TransformStamped transformStamped_r2c; /**< Calculated transfor matrix for robot base to camera optical */
    double marker_length;                                 /**< Length of the aruco markers */
    int markers_dictionary;                               /**< \enum cv::aruco::PREDEFINED_DICTIONARY_NAME */
    visualization_msgs::MarkerArray detected_markers_;
    geometry_msgs::PoseStamped True_pose_;

    void GetTopicName();

    void GetParameter();

    void getTransformStamped(const std::string &target_frame, const std::string &source_frame, geometry_msgs::TransformStamped &transform_stamped);

    void GenerateParticles();

    void addEncoder(const double &enl, const double &enr);

    void addImage(const Mat &img);

    void ResampleParticles();

    int getObservations(const cv::Mat &img, std::vector<ArucoMarker> &obs);

    bool checkLandmark(const int &aruco_id, int &landmark_idx);

    void MaxWeightPaticle();

    void CalculateCovariance(const cv::Vec3d &tvec, const cv::Vec3d &rvec, const std::vector<cv::Point2f> &marker_corners, Eigen::Matrix3d &covariance);


    void fillTransform(tf2::Transform &transform_, const cv::Vec3d &rvec, const cv::Vec3d &tvec);

    bool GenerateMarker(int id, double length, double x, double y, double z, tf2::Quaternion q, visualization_msgs::Marker &marker_, std_msgs::ColorRGBA color, ros::Duration lifetime);

    void normAngle(double &angle)
    {
        const static double PI = 3.1415926;
        static double Two_PI = 2.0 * PI;
        if (angle >= PI)
            angle -= Two_PI;
        if (angle < -PI)
            angle += Two_PI;
    }

    visualization_msgs::MarkerArray toRosMarker(double scale);

    geometry_msgs::PoseWithCovarianceStamped toRosPose()
    {
        geometry_msgs::PoseWithCovarianceStamped rpose;
        rpose.header.frame_id = "world";
        rpose.pose.pose.position.x = BestParticle.pose(0);
        rpose.pose.pose.position.y = BestParticle.pose(1);
        rpose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(BestParticle.pose(2));

        rpose.pose.covariance.at(0) = BestParticle.cov(0, 0);
        rpose.pose.covariance.at(1) = BestParticle.cov(0, 1);
        rpose.pose.covariance.at(6) = BestParticle.cov(1, 0);
        rpose.pose.covariance.at(7) = BestParticle.cov(1, 1);
        rpose.pose.covariance.at(5) = BestParticle.cov(0, 2);
        rpose.pose.covariance.at(30) = BestParticle.cov(2, 0);
        rpose.pose.covariance.at(35) = BestParticle.cov(2, 2);
        rpose.header.stamp = ros::Time::now();

        return rpose;
    }

    cv::Mat markedImg() { return marker_img_; }
public:
    ParticleFilter();
    ~ParticleFilter() {};
    void ImageCallback(const sensor_msgs::ImageConstPtr &img_ptr);
    void EncoderCallback(const geometry_msgs::QuaternionStamped::ConstPtr &encoder_ptr);
    void TrueCallback(const geometry_msgs::PoseStamped &true_ptr);
};
#endif