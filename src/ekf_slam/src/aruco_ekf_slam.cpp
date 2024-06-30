#include <ekf_slam/aruco_ekf_slam.h>

ArUcoEKFSLAM::ArUcoEKFSLAM():private_node("~")
{
    ROS_INFO_STREAM("\033[1;32m----> EKFSLAM started.\033[0m");
    image_transport::ImageTransport it(node_handle);
    GetTopicName();
    GetParameter();
    is_init_ = false;
    last_observed_marker_.clear();
    detected_map_.markers.clear();
    mu_.resize(3, 1);
    mu_.setZero();
    sigma_.resize(3, 3);
    sigma_.setZero();
    dictionary_ = cv::aruco::getPredefinedDictionary(
        static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(markers_dictionary));

    encoder_sub = node_handle.subscribe(encoder_topic, 1, &ArUcoEKFSLAM::EncoderCallback, this);
    image_sub = node_handle.subscribe(image_topic, 1, &ArUcoEKFSLAM::ImageCallback, this);
    true_sub = node_handle.subscribe("true_pose", 1, &ArUcoEKFSLAM::TrueCallback, this);

    g_landmark_pub = node_handle.advertise<visualization_msgs::MarkerArray>("ekf_slam/landmark", 1, this);
    g_robot_pose_pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_slam/pose", 1, this);
    initial_state_pub = node_handle.advertise<std_msgs::Int8>("initial_state", 1, this);
    g_true_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("ekf_slam/true_pose",1,this);
    g_img_pub = it.advertise("ekf_slam/image", 1, this);
};

void ArUcoEKFSLAM::GetTopicName()
{
    node_handle.getParam("/aruco_ekf_slam/topic/image", image_topic);
    node_handle.getParam("/aruco_ekf_slam/topic/encoder", encoder_topic);
}

void ArUcoEKFSLAM::GetParameter()
{
    double fx, fy, cx, cy, k1, k2, p1, p2, k3;
    node_handle.getParam("/aruco_ekf_slam/camera/fx", fx);
    node_handle.getParam("/aruco_ekf_slam/camera/fy", fy);
    node_handle.getParam("/aruco_ekf_slam/camera/cx", cx);
    node_handle.getParam("/aruco_ekf_slam/camera/cy", cy);
    node_handle.getParam("/aruco_ekf_slam/camera/k1", k1);
    node_handle.getParam("/aruco_ekf_slam/camera/k2", k2);
    node_handle.getParam("/aruco_ekf_slam/camera/p1", p1);
    node_handle.getParam("/aruco_ekf_slam/camera/p2", p2);
    node_handle.getParam("/aruco_ekf_slam/camera/k3", k3);

    K_ = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    dist_ = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);

    node_handle.getParam("/aruco_ekf_slam/odom/kl", kl);
    node_handle.getParam("/aruco_ekf_slam/odom/kr", kr);
    node_handle.getParam("/aruco_ekf_slam/odom/b", b);
    node_handle.getParam("/aruco_ekf_slam/covariance/Q_k", Q_k); // encoder error coefficient
    node_handle.getParam("/aruco_ekf_slam/covariance/R_x", R_x); // encoder error coefficient
    node_handle.getParam("/aruco_ekf_slam/covariance/R_y", R_y); // encoder error coefficient
    node_handle.getParam("/aruco_ekf_slam/covariance/R_theta", R_theta); // encoder error coefficient
    node_handle.getParam("/aruco_ekf_slam/aruco/markers_dictionary", markers_dictionary);
    node_handle.getParam("/aruco_ekf_slam/aruco/marker_length", marker_length_);

    std::string robot_frame_base, camera_frame_optical;
    node_handle.getParam("/aruco_ekf_slam/frame/robot_frame_base", robot_frame_base);
    node_handle.getParam("/aruco_ekf_slam/frame/camera_frame_optical", camera_frame_optical);
    node_handle.getParam("/aruco_ekf_slam/const/USEFUL_DISTANCE_THRESHOLD_", USEFUL_DISTANCE_THRESHOLD_);
    
    node_handle.getParam("/aruco_ekf_slam/robot/r2c_translation", r2c_translation);
    node_handle.getParam("/aruco_ekf_slam/robot/r2c_rotation", r2c_rotation);

    
    getTransformStamped(robot_frame_base, camera_frame_optical, transformStamped_r2c);
    ROS_INFO_STREAM("\033[1;32m----> Parameter Loaded.\033[0m");
}

void ArUcoEKFSLAM::getTransformStamped(const std::string &target_frame, const std::string &source_frame, geometry_msgs::TransformStamped &transform_stamped)
{
    transform_stamped.transform.translation.x = r2c_translation[0];
    transform_stamped.transform.translation.y = r2c_translation[1];
    transform_stamped.transform.translation.z = r2c_translation[2];
    transform_stamped.transform.rotation.x = r2c_rotation[0];
    transform_stamped.transform.rotation.y = r2c_rotation[1];
    transform_stamped.transform.rotation.z = r2c_rotation[2];
    transform_stamped.transform.rotation.w = r2c_rotation[3];
}

void ArUcoEKFSLAM::TrueCallback(const geometry_msgs::PoseStamped &true_ptr)
{
    True_pose_ = true_ptr;
    // ROS_INFO_STREAM("\033[1;32m----> transformStamped_r2c is.\033[0m" << transformStamped_r2c);
    // ROS_INFO_STREAM(" The True pose is " << True_pose_.pose.position.x);
}

void ArUcoEKFSLAM::addEncoder(const double &enl, const double &enr)
{
    // ROS_INFO_STREAM("Encoder data reveived");
    if (is_init_ == false)
    {
        last_time_ = ros::Time::now();
        start_time_ = ros::Time::now();
        is_init_ = true;
        std_msgs::Int8 msg;
        msg.data = 1;
        initial_state_pub.publish(msg);
        return;
    }
    double dt = (ros::Time::now() - last_time_).toSec();
    last_time_ = ros::Time::now();

    /* calculate change of distance */
    double delta_enl = dt * enl; // change of angle
    double delta_enr = dt * enr;
    double delta_s_l = kl * delta_enl;
    double delta_s_r = kr * delta_enr;

    double delta_theta = (delta_s_r - delta_s_l) / (2 * b);
    double delta_s = (delta_s_l + delta_s_r) / 2;

    /***** update mean value *****/
    double tmp_th = mu_(2) + 0.5 * delta_theta;
    double cos_tmp_th = cos(tmp_th);
    double sin_tmp_th = sin(tmp_th);

    double temp_angle = mu_(2, 0) + 0.5 * delta_theta;
    mu_(0, 0) += delta_s * cos(temp_angle);
    mu_(1, 0) += delta_s * sin(temp_angle);
    mu_(2, 0) += delta_theta;
    normAngle(mu_(2, 0));

    /***** update covariance *****/
    Eigen::Matrix3d H_xi;
    H_xi << 1.0, 0.0, -delta_s * sin_tmp_th,
        0.0, 1.0, delta_s * cos_tmp_th,
        0.0, 0.0, 1.0;

    Eigen::Matrix<double, 3, 2> wkh;
    wkh << cos_tmp_th, cos_tmp_th, sin_tmp_th, sin_tmp_th, 1 / b, -1 / b;
    wkh = (0.5 * kl * dt) * wkh;

    int N = mu_.rows();
    Eigen::MatrixXd F(N, 3);
    F.setZero();
    F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd Hx = Eigen::MatrixXd::Identity(N, N);
    Hx.block(0, 0, 3, 3) = H_xi;
    Eigen::Matrix2d sigma_u;
    sigma_u << Q_k * fabs(enl), 0.0, 0.0, Q_k * fabs(enr);
    Eigen::MatrixXd Qk = wkh * sigma_u * wkh.transpose();
    sigma_ = Hx * sigma_ * Hx.transpose() + F * Qk * F.transpose();
}

void ArUcoEKFSLAM::addImage(const Mat &img)
{
    // ROS_INFO_STREAM("Image data reveived");
    ROS_INFO_STREAM(" \n\n\n\n  Image data reveived " << std::endl
                    // "z_hat:" << z_hat << std::endl
                    // << "mu:" << mu_ << std::endl
                    // << "sigma:" << sigma_ << std::endl
    );
    if (is_init_ == false)
        return;
    std::vector<ArucoMarker> obs;
    getObservations(img, obs);
    ROS_INFO_STREAM("obs_.size:" << obs.size() << std::endl);
    std::vector<ArucoMarker> observed_marker;
    for (ArucoMarker ob : obs)
    {
        /* 计算观测方差 */
        Eigen::Matrix3d Rk = ob.observe_covariance_;
        if (ob.aruco_index_ >= 0) // if the lamdmark is exixt
        {
            // ROS_INFO_STREAM("\n\n\n\n\n\\n\n  Image data reveived 1 \n\n\n\n\n");
            int N = mu_.rows();
            Eigen::MatrixXd F(6, N);
            F.setZero();
            //F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            //F.block<3, 3>(3, 3 + 3 * ob.aruco_index_) = Eigen::Matrix3d::Identity();
            F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
            F(3, 3 + 3 * ob.aruco_index_) = 1;
            F(4, 4 + 3 * ob.aruco_index_) = 1;
            F(5, 5 + 3 * ob.aruco_index_) = 1;
            /* calculate estimation based on estimated state */
            double &mx = mu_(3 + 3 * ob.aruco_index_);
            double &my = mu_(4 + 3 * ob.aruco_index_);
            double &mtheta = mu_(5 + 3 * ob.aruco_index_);
            double &x = mu_(0);
            double &y = mu_(1);
            double &theta = mu_(2);
            double sintheta = sin(theta);
            double costheta = cos(theta);
            double global_delta_x = mx - x;
            double global_delta_y = my - y;
            double global_delta_theta = mtheta - theta;
            normAngle(global_delta_theta);
https://im.csdn.net/chat/weixin_50578602
            Eigen::Vector3d z_hat(global_delta_x * costheta + global_delta_y * sintheta,
                                  -global_delta_x * sintheta + global_delta_y * costheta,
                                  global_delta_theta);

            Eigen::Vector3d z(ob.x_, ob.y_, ob.theta_);
            Eigen::Vector3d ze = z - z_hat;
            ROS_INFO_STREAM("the error between observation and prediction" << ze << std::endl);
            normAngle(ze[2]);

            Eigen::MatrixXd Gxm(3, 6);
            Gxm << -costheta, -sintheta, -global_delta_x * sintheta + global_delta_y * costheta, costheta, sintheta, 0,
                sintheta, -costheta, -global_delta_x * costheta - global_delta_y * sintheta, -sintheta, costheta, 0,
                0, 0, -1, 0, 0, 1;
            Eigen::MatrixXd Gx = Gxm * F;

            Eigen::MatrixXd K = sigma_ * Gx.transpose() * (Gx * sigma_ * Gx.transpose() + Rk).inverse();
            if (ze.norm() >= 1 || K.norm() >= 10)
            {
                ROS_INFO_STREAM("\n\n\n error \n\n\n"
                                //  << "ob.aruco_index_:"<<ob.aruco_index_ << std::endl
                                << "z_hat:" << z_hat << std::endl
                                << "z:" << z << std::endl
                                << "ze:" << ze.norm() << std::endl
                                << "K:" << K.norm() << std::endl
                                << "Rk:" << Rk << std::endl
                                << "Gxm:" << Gxm << std::endl
                                //  << "i:" << ob.aruco_index_ << std::endl
                                << "mu:" << mu_ << std::endl
                                << "sigma:" << sigma_ << std::endl);
                // TODO: Remove map point?
                // mu_.topLeftCorner(3, 0) += (K * ze).topLeftCorner(3, 0);
                // continue;
            }
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(N, N);

            bool up_date_map = false;
            double robot_pose_convariance = sigma_.topLeftCorner(3, 3).norm();
            double map_pose_convariance = sigma_.block(3 + 3 * ob.aruco_index_, 3 + 3 * ob.aruco_index_, 3, 3).norm();

            std::vector<ArucoMarker>::iterator last_observe_ptr = std::find(last_observed_marker_.begin(), last_observed_marker_.end(), ob);
            if (last_observe_ptr != last_observed_marker_.end() && (last_observe_ptr->last_observation_ - z).norm() < 0.01)
            {
                ROS_INFO_STREAM("\n lastobservation_delt:" << (last_observe_ptr->last_observation_ - z).norm() << std::endl);
                mu_.topLeftCorner(3, 0) += (K * ze).topLeftCorner(3, 0);
            }
            else
            {
                // only do correction after the ArucoMarker changes a distance
                ROS_INFO_STREAM("\n the landmark changes a distance" << std::endl);
                ob.last_observation_ = z;
                mu_ += (K * ze);
                sigma_ = ((I - K * Gx) * sigma_);
            }
        }
        else // new markers are added to the map
        {
            float sinth = sin(mu_(2));
            float costh = cos(mu_(2));

            /**** add to the map ****/
            /* updata the mean value */
            int N = mu_.rows();
            Eigen::VectorXd tmp_mu(N + 3);
            tmp_mu.setZero();

            double map_x = mu_(0) + costh * ob.x_ - sinth * ob.y_;
            double map_y = mu_(1) + sinth * ob.x_ + costh * ob.y_;
            double map_theta = mu_(2) + ob.theta_;
            normAngle(map_theta);
            tmp_mu << mu_, map_x, map_y, map_theta;
            mu_.resize(N + 3, 1);
            mu_ = tmp_mu;

            /* update the covariance of the map */
            double deltax = map_x - mu_(0);
            double deltay = map_y - mu_(1);
            Eigen::Matrix3d sigma_s = sigma_.block(0, 0, 3, 3);
            Eigen::Matrix<double, 3, 3> Gsk;
            Gsk << -costh, -sinth, -sinth * deltax + costh * deltay,
                sinth, -costh, -deltax * costh - deltay * sinth,
                0, 0, -1;
            Eigen::Matrix<double, 3, 3> Gmi;
            Gmi << costh, sinth, 0,
                -sinth, costh, 0,
                0, 0, 1;

            /* calculate variance for new marker*/
            Eigen::Matrix3d sigma_mm = Gmi * (Gsk * sigma_s * Gsk.transpose() + Rk).transpose() * Gmi.transpose();

            /* calculate covariance for new marker and exited markers*/
            Eigen::MatrixXd Gfx = sigma_.topRows(3);
            Eigen::MatrixXd sigma_mx = -Gmi * Gsk * Gfx;
            Eigen::MatrixXd tmp_sigma(N + 3, N + 3);
            tmp_sigma.setZero();
            tmp_sigma.topLeftCorner(N, N) = sigma_;
            tmp_sigma.topRightCorner(N, 3) = sigma_mx.transpose();
            tmp_sigma.bottomLeftCorner(3, N) = sigma_mx;
            tmp_sigma.bottomRightCorner(3, 3) = sigma_mm;
            sigma_.resize(N + 3, N + 3);
            sigma_ = tmp_sigma;

            /***** add new marker's id to the dictionary *****/
            aruco_id_map.insert(std::pair<int, int>{ob.aruco_id_, (mu_.rows() - 3) / 3 - 1});
        } // add new landmark
        observed_marker.push_back(ob);
    }
    last_observed_marker_ = observed_marker;
    /* visualise the new map marker */
    // TODO can just add instead of moving all marks?
}

int ArUcoEKFSLAM::getObservations(const cv::Mat &img, std::vector<ArucoMarker> &obs)
{
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> IDs;
    std::vector<cv::Vec3d> rvs, tvs;
    detected_markers_.markers.clear();
    cv::aruco::detectMarkers(img, dictionary_, marker_corners, IDs);
    cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length_, K_, dist_, rvs, tvs);

    /* draw all marks */
    marker_img_ = img.clone();
    cv::aruco::drawDetectedMarkers(marker_img_, marker_corners, IDs);
    for (size_t i = 0; i < IDs.size(); i++)
        cv::aruco::drawAxis(marker_img_, K_, dist_, rvs[i], tvs[i], 0.07);

    /*  筛选距离较近的使用 */
    // USEFUL_DISTANCE_THRESHOLD_ = 3; //3 m
    ROS_INFO_STREAM("IDs_.size:" << IDs.size() << std::endl);
    for (size_t i = 0; i < IDs.size(); i++)
    {
        float dist = cv::norm<double>(tvs[i]); //计算距离
        //ROS_ERROR_STREAM("the distance of the landmark is:"<<dist);
        if (dist > USEFUL_DISTANCE_THRESHOLD_)
        {
            //ROS_INFO_STREAM("USEFUL_DISTANCE_THRESHOLD_:"<<USEFUL_DISTANCE_THRESHOLD_)
            // ROS_ERROR_STREAM("USEFUL_DISTANCE_THRESHOLD_:"<<USEFUL_DISTANCE_THRESHOLD_ << "the distance of the landmark is :" << dist);
            continue;
        }
        /*visualise used marks */
        visualization_msgs::Marker _marker;
        tf2::Transform _transform;
        fillTransform(_transform, rvs[i], tvs[i]);
        std_msgs::ColorRGBA color;
        color.a = 1;
        color.b = 0;
        color.g = 0;
        color.r = 1;
        GenerateMarker(IDs[i], marker_length_, tvs[i][0], tvs[i][1], tvs[i][2], _transform.getRotation(), _marker, color, ros::Duration(0.1));
        tf2::doTransform(_marker.pose, _marker.pose, transformStamped_r2c);
        _marker.header.frame_id = "base_link";
        detected_markers_.markers.push_back(_marker);

        /* calculate observation mx, my, mtheta */
        cv::Vec3d tvec = tvs[i];
        cv::Vec3d rvec = rvs[i];
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        double x = tvec[2] + transformStamped_r2c.transform.translation.x;
        double y = -tvec[0] + transformStamped_r2c.transform.translation.y;
        double theta = atan2(-R.at<double>(0, 2), R.at<double>(2, 2));
        normAngle(theta);
        int aruco_id = IDs[i];
        Eigen::Matrix3d covariance;
        CalculateCovariance(tvec, rvec, marker_corners[i], covariance);
        /* add to observation vector */
        if (covariance.norm() > 1)
        {
            ROS_ERROR_STREAM("covariance norm is too big  " << covariance.norm());
            continue;
        }
        ArucoMarker ob(aruco_id, x, y, theta, covariance);
        int aruco_index;
        checkLandmark(aruco_id, aruco_index);
        ob.aruco_index_ = aruco_index;
        obs.push_back(ob);
    } //for all detected markers
    return obs.size();
}

void ArUcoEKFSLAM::CalculateCovariance(const cv::Vec3d &tvec, const cv::Vec3d &rvec, const std::vector<cv::Point2f> &marker_corners, Eigen::Matrix3d &covariance)
{

    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(objectPoints_, rvec, tvec, K_,
                      dist_, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;

    auto dist = [](const cv::Point2f &p1, const cv::Point2f &p2)
    {
        double x1 = p1.x;
        double y1 = p1.y;
        double x2 = p2.x;
        double y2 = p2.y;
        double dx = x1 - x2;
        double dy = y1 - y2;

        return sqrt(dx * dx + dy * dy);
    };

    for (unsigned int i = 0; i < objectPoints_.size(); i++)
    {
        double error = dist(marker_corners[i], projectedPoints[i]);
        //ROS_INFO_STREAM(" \n  error is " << error << std::endl);
        totalError += error * error;
    }
    double rmserror = (sqrt(totalError) / (double)objectPoints_.size())/20;
    double object_error = (rmserror / dist(marker_corners[0], marker_corners[2])) *
                       (norm(tvec) / marker_length_);
    ROS_INFO_STREAM(" \n  variace is " << object_error << std::endl);
    covariance << object_error * 2, 0, 0, 0, object_error * 2, 0, 0, 0, object_error + 1e-3;                   
//     double rmserror = sqrt(totalError / (double)objectPoints_.size());
//     double variace = rmserror / 50;
//     ROS_INFO_STREAM(" \n  variace is " << variace << std::endl);
//     covariance << variace, 0, 0, 0, variace, 0, 0, 0, variace/10;
}

//void ArUcoEKFSLAM::ImageCallback(const sensor_msgs::ImageConstPtr &img_ptr, const sensor_msgs::CameraInfoConstPtr &cinfo)
void ArUcoEKFSLAM::ImageCallback(const sensor_msgs::ImageConstPtr &img_ptr)
{
    ROS_INFO_STREAM("image_callback:");
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_ptr);

    /***** add image *****/
    addImage(cv_ptr->image);

    /***** pubish marked image*****/
    cv::Mat img = markedImg();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    g_img_pub.publish(msg);
}

void ArUcoEKFSLAM::EncoderCallback(const geometry_msgs::QuaternionStamped::ConstPtr &encoder_ptr)
{
    ROS_INFO_STREAM("encoder_callback:");
    double enr = encoder_ptr->quaternion.x;
    double enl = encoder_ptr->quaternion.y;

    addEncoder(enl, enr);
    visualization_msgs::MarkerArray marker = toRosMarker(1);
    g_landmark_pub.publish(marker);

    geometry_msgs::PoseWithCovarianceStamped pose = toRosPose();
    g_robot_pose_pub.publish(pose);
    g_true_pose_pub.publish(True_pose_);
}

bool ArUcoEKFSLAM::checkLandmark(const int &aruco_id, int &landmark_idx)
{

    if (!aruco_id_map.empty() && aruco_id_map.end() != aruco_id_map.find(aruco_id))
    {
        landmark_idx = aruco_id_map.at(aruco_id);
        //ROS_INFO_STREAM("aruco_id:" << aruco_id << "index:" << landmark_idx);
        return true;
    }
    else
        landmark_idx = -1;
        //ROS_INFO_STREAM("it is a new landmark" );
    return false;
}

bool ArUcoEKFSLAM::GenerateMarker(int id, double length, double x, double y, double z, tf2::Quaternion q, visualization_msgs::Marker &marker_, std_msgs::ColorRGBA color, ros::Duration lifetime)
{
    marker_.id = id;
    marker_.header.frame_id = "world";
    marker_.type = visualization_msgs::Marker::CUBE;
    marker_.scale.x = length;
    marker_.scale.y = length;
    marker_.scale.z = 0.01;

    marker_.color = color;
    marker_.pose.position.x = x;
    marker_.pose.position.y = y;
    marker_.pose.position.z = z;
    marker_.pose.orientation = tf2::toMsg(q);
    marker_.lifetime = lifetime;
    return true;
}

void ArUcoEKFSLAM::fillTransform(tf2::Transform &transform_, const cv::Vec3d &rvec, const cv::Vec3d &tvec)
{
    cv::Mat rot(3, 3, CV_64FC1);
    // cv::Mat Rvec64;
    // rvec.convertTo(Rvec64, CV_64FC1);
    cv::Rodrigues(rvec, rot);
    // cv::Mat tran64;
    // tvec.convertTo(tran64, CV_64FC1);

    tf2::Matrix3x3 tf_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                          rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                          rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));

    tf2::Vector3 tf_orig(tvec[0], tvec[1], tvec[2]);

    tf2::Transform transform(tf_rot, tf_orig);
    transform_ = transform;
}

visualization_msgs::MarkerArray ArUcoEKFSLAM::toRosMarker(double scale)
{
    visualization_msgs::MarkerArray markers;
    for (int i = 4; i < mu_.rows(); i += 3)
    {
        double &mx = mu_(i - 1, 0);
        double &my = mu_(i, 0);
        double &mtheta = mu_(i+1 , 0);

        /* 计算地图点的协方差椭圆角度以及轴长 */
        Eigen::Matrix2d sigma_m = sigma_.block(i - 1, i - 1, 2, 2); //协方差
        cv::Mat cvsigma_m = (cv::Mat_<double>(2, 2) << sigma_m(0, 0), sigma_m(0, 1), sigma_m(1, 0), sigma_m(1, 1));
        cv::Mat eigen_value, eigen_vector;
        cv::eigen(cvsigma_m, eigen_value, eigen_vector);
        double angle = atan2(eigen_vector.at<double>(0, 1), eigen_vector.at<double>(0, 0));
        double x_len = 2 * sqrt(eigen_value.at<double>(0, 0) * 5.991);
        double y_len = 2 * sqrt(eigen_value.at<double>(1, 0) * 5.991);

        /* 构造marker */
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "ekf_slam";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mx;
        marker.pose.position.y = my;
        marker.pose.position.z = 0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
        marker.scale.x = scale * x_len;
        marker.scale.y = scale * y_len;
        marker.scale.z = 0.1 * scale * (x_len + y_len);
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markers.markers.push_back(marker);
    } // for all mpts

    return markers;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "reslam_odom");
    ArUcoEKFSLAM slam;
    ros::spin();

    ros::shutdown();
    return 0;
}