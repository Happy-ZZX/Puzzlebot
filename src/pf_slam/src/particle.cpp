#include "pf_slam/particle.h"
#include <ros/console.h>

Particle::Particle (const Eigen::Vector3d &pose_, Eigen::Matrix3d &cov_, const double &weight_)
{
    pose = pose_;
    cov = cov_;
    weight = weight_;
    particle_map_.erase(particle_map_.begin(),particle_map_.end());
}

void Particle::addencoder(const double &delta_x, const double &delta_y, const double &delta_theta, Eigen::MatrixXd &encoder_noise)
{
    /***** sample from the current movement noise distribution *****/
    // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::default_random_engine generator(seed);

    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<double> dist(delta_x, encoder_noise(0));
    pose(0) += dist(generator);
    // pose(0) += delta_x + dist(generator);
    std::mt19937 generator2(rd());
    // std::default_random_engine generator2(seed);
    std::normal_distribution<double> dist2(delta_y, encoder_noise(1));
    pose(1) += dist2(generator2);
    // pose(1) += delta_y + dist2(generator2);
    std::mt19937 generator3(rd());
    // std::default_random_engine generator3(seed);
    std::normal_distribution<double> dist3(delta_theta, encoder_noise(2));
    pose(2) += dist3(generator3) ;
    // ROS_ERROR_STREAM ("radom number is " << dist(generator));
    normAngle(pose(2));
}

//normalize the robot angle
void Particle::normAngle(double &angle)
{
    /* swap angle to (-pi-pi)*/
    const static double PI = 3.14159265358979323846;
    const static double Two_PI = 2.0 * PI;
    if (angle >= PI)
        angle -= Two_PI;
    if (angle < -PI)
        angle += Two_PI;
}

void Particle:: insert(const ArucoMarker &observed_marker)
{
    float sinth = sin(pose(2));
    float costh = cos(pose(2));
    double x  = pose(0) + costh * observed_marker.x_ - sinth * observed_marker.y_;
    double y  = pose(1) + sinth * observed_marker.x_ + costh * observed_marker.y_;
    double theta = pose(2) + observed_marker.theta_;
    normAngle(theta);
    int aruco_id = observed_marker.aruco_id_;
    Eigen::Matrix<double, 3, 3> Gmi;
            Gmi << costh, sinth, 0,
                -sinth, costh, 0,
                0, 0, 1;
    Eigen::Matrix3d covariance = Gmi.inverse() * observed_marker.observe_covariance_ * Gmi.inverse().transpose();
    Eigen::Vector3d mean;
    mean << x, y ,theta;
    ArucoMarker NewMaker(aruco_id, x, y, theta, mean, covariance);
    particle_map_.insert(std::pair<int, struct ArucoMarker> {particle_map_.size(), NewMaker});
    //ROS_ERROR_STREAM("insert new makers:" <<  observed_marker.aruco_id_ << std::endl);
}

void Particle:: update(const ArucoMarker &observed_marker)
{
    double prob = exp(-70);
    int ID = observed_marker.aruco_index_;
    //ROS_ERROR_STREAM ("the obs ID is " << observed_marker.aruco_id_ << " the marker in map is " << particle_map_.at(ID).aruco_id_);
    Eigen::Vector3d predicted_ob;
    float sinth = sin(pose(2));
    float costh = cos(pose(2));
    double global_delta_x = particle_map_.at(ID).x_ - pose[0];
    double global_delta_y = particle_map_.at(ID).y_ - pose[1];
    double global_delta_theta = particle_map_.at(ID).theta_ - pose[2];
    normAngle(global_delta_theta);
    predicted_ob[0] = costh * global_delta_x + sinth * global_delta_y;
    predicted_ob[1] = -sinth * global_delta_x + costh * global_delta_y;
    predicted_ob[2] = global_delta_theta;

    Eigen::Vector3d current_ob;
    current_ob[0] = observed_marker.x_;
    current_ob[1] = observed_marker.y_;
    current_ob[2] = observed_marker.theta_;

    //ROS_ERROR_STREAM("observed distance" << current_ob << std::endl << "predcicted distance" << predicted_ob << std::endl);
    Eigen::Matrix3d JaccobianGm;
    JaccobianGm << costh, sinth, 0, 
                    -sinth, costh, 0,
                    0, 0, 1;
    Eigen::Matrix3d adj_cov;
    adj_cov = JaccobianGm * particle_map_.at(ID).observe_covariance_ * JaccobianGm.transpose() + observed_marker.observe_covariance_;
            
    //update the covariance based on the 
    Eigen::Matrix3d K;
    K = particle_map_.at(ID).observe_covariance_ * JaccobianGm.transpose() * adj_cov.inverse();
    Eigen::Vector3d new_maker_mean;
    Eigen::Matrix3d new_maker_cov;
    new_maker_mean = particle_map_.at(ID).observe_mean_ + K * (current_ob - predicted_ob);
    new_maker_cov = (Eigen::Matrix3d::Identity() - K * JaccobianGm) * particle_map_.at(ID).observe_covariance_;
    prob = multi_normal(current_ob, predicted_ob, adj_cov);
    ArucoMarker NewMaker(observed_marker.aruco_id_, new_maker_mean[0], new_maker_mean[1], new_maker_mean[2], new_maker_mean, new_maker_cov);
    particle_map_.erase(ID);
    particle_map_.insert(std::pair<int, ArucoMarker> {ID, NewMaker});
    weight = weight * prob;
    // ROS_ERROR_STREAM(observed_marker.aruco_id_ << " the prob is " << prob << " /n " << weight);
}

// void Particle:: update(const std::vector<ArucoMarker> &observed_marker)
// {
//     for (int i = 0; i < observed_marker.size(); i++)
//     {
//         // if the map is empty or the landmark is new 
//         if ( observed_marker[i].aruco_index_ == -1)
//         {
//             float sinth = sin(pose(2));
//             float costh = cos(pose(2));
//             double x  = pose(0) + costh * observed_marker[i].x_ - sinth * observed_marker[i].y_;
//             double y  = pose(1) + sinth * observed_marker[i].x_ + costh * observed_marker[i].y_;
//             double theta = pose(2) + observed_marker[i].theta_;
//             int aruco_id = observed_marker[i].aruco_id_;
//             Eigen::Matrix3d covariance = observed_marker[i].observe_covariance_;
//             Eigen::Vector3d mean;
//             mean << x, y ,theta;
//             ArucoMarker NewMaker(aruco_id, x, y, theta, mean, covariance);
//             particle_map_.insert(std::pair<int, ArucoMarker> {observed_marker[i].aruco_id_, NewMaker});
//             ROS_ERROR_STREAM("insert new makers:" <<  observed_marker[i].aruco_id_ << std::endl);
//             continue;
//         }
//         else
//         {
//             continue;
//         }
//         // the landmark is already been observed
//         else
//         {
//             double prob = exp(-70);
//             int ID = observed_marker[i].aruco_id_;
//             //ArucoMarker map_marker = particle_map_[observed_marker[i].aruco_id_];
//             Eigen::Vector3d predicted_ob;
//             predicted_ob[0] = particle_map_[ID].x_ - pose[0];
//             predicted_ob[1] = particle_map_[ID].y_ - pose[1];
//             predicted_ob[2] = particle_map_[ID].theta_ - pose[2];
            
//             Eigen::Vector3d current_ob;
//             float sinth = sin(pose(2));
//             float costh = cos(pose(2));
//             current_ob[0] = costh * observed_marker[i].x_ - sinth * observed_marker[i].y_;
//             current_ob[1] = sinth * observed_marker[i].x_ + costh * observed_marker[i].y_;
//             current_ob[2] = observed_marker[i].theta_;
//             ROS_INFO_STREAM("observed distance" << current_ob << std::endl << "predcicted distance" << predicted_ob << std::endl);
//             double &theta = pose[2];
//             double sintheta = sin(theta);
//             double costheta = cos(theta);
//             Eigen::Matrix3d JaccobianGm;
//             JaccobianGm << costheta, sintheta, 0, 
//                            -sintheta, costheta, 0,
//                            0, 0, 1;
//             Eigen::Matrix3d adj_cov;
//             adj_cov = JaccobianGm * particle_map_[ID].observe_covariance_ * JaccobianGm.transpose() + observed_marker[i].observe_covariance_;
//             prob = multi_normal(current_ob, predicted_ob, adj_cov);
            
//             //update the covariance based on the 
//             Eigen::Matrix3d K;
//             K = observed_marker[i].observe_covariance_ * JaccobianGm.transpose() * adj_cov.inverse();
//             Eigen::Vector3d new_maker_mean;
//             Eigen::Matrix3d new_maker_cov;
//             new_maker_mean = observed_marker[i].observe_mean_ + K * (current_ob - predicted_ob);
//             new_maker_cov = (Eigen::Matrix3d::Identity() - K * JaccobianGm) * adj_cov;
//             ArucoMarker NewMaker(observed_marker[i].aruco_id_, new_maker_mean[0], new_maker_mean[1], new_maker_mean[2], new_maker_mean, new_maker_cov);
//             particle_map_.erase(ID);
//             particle_map_.insert(std::pair<int, ArucoMarker> {observed_marker[i].aruco_id_, NewMaker});
//             // particle_map_[observed_marker[i].aruco_id_].observe_mean_ = new_maker_mean;
//             // particle_map_[observed_marker[i].aruco_id_].observe_covariance_ = new_maker_cov;
//             // particle_map_[observed_marker[i].aruco_id_].x_ = new_maker_mean[0];
//             // particle_map_[observed_marker[i].aruco_id_].y_ = new_maker_mean[1];
//             // particle_map_[observed_marker[i].aruco_id_].theta_ = new_maker_mean[2];
//             weight = weight * prob;
//             continue;
//         }
//     }
// }

double Particle::multi_normal(Eigen::Vector3d &x, Eigen::Vector3d &mean, Eigen::Matrix3d &cov)
{
    Eigen::Matrix3d  cov_2pi = cov;
    double den = sqrt( 8 * M_PI * M_PI * M_PI * cov_2pi.determinant());
    Eigen::Vector3d delta_x = x - mean;
    double num = exp(-0.5 * delta_x.transpose() * cov.inverse() * delta_x);
    return num/den;
}
