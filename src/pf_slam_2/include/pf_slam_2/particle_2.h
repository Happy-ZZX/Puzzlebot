#ifndef PARTICAL_2_H
#define PARTICAL_2_H

#include <math.h>
#include <random>
#include <armadillo>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <armadillo>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoMarker
{
public:
    ArucoMarker() {}
    ArucoMarker(const int &aruco_id, const double &x, const double &y, const double &theta, const Eigen::Vector3d &observe_mean, const Eigen::Matrix3d &observe_covariance)
        : aruco_id_(aruco_id), x_(x), y_(y), theta_(theta), observe_covariance_(observe_covariance), observe_mean_(observe_mean), aruco_index_(-1) {}

    int aruco_id_;
    int aruco_index_;
    double x_;
    double y_;
    double theta_;
    Eigen::Vector3d observe_mean_;
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

class Particle
{
private:
    /**
     * @brief normalize the robot angle 
     *
     * @param angle
     */
    void normAngle(double &angle);
    /**
    * @brief calculate the multi normal variable distribution probability 
    *
    * @param value vector
    * @param mean value vector
    * @param covariance value
    */
    double multi_normal(Eigen::Vector3d &x, Eigen::Vector3d &mean, Eigen::Matrix3d &cov);
    

public:
    Eigen::Vector3d pose;
    Eigen::Matrix3d cov;
    double weight;
    std::map<int, struct ArucoMarker> particle_map_;

    Particle (const Eigen::Vector3d &pose_, Eigen::Matrix3d &cov_, const double &weight_);

    Particle(){};
    
    ~Particle(){};
    /**
    * @brief add encoder data and update particle
    *
    * @param delta_x
    * @param delta_y
    * @param delta_theta
    */
    void addencoder (const double &delta_x, const double &delta_y, const double &delta_theta, Eigen::MatrixXd &encoder_noise);
    /**
     * @brief update the particle weight
     *
     * @param observed_marker
     */
    void update(const ArucoMarker &observed_marker);

    void sample(const Eigen::Matrix3d &EncoderNoise);
    
    void insert(const ArucoMarker &observed_marker);
};
#endif