#include <pf_slam/particle_filter_slam.h>

ParticleFilter::ParticleFilter():private_node("~")
{
    ROS_INFO_STREAM("\033[1;32m----> PFSLAM started.\033[0m");
    image_transport::ImageTransport it(node_handle);
    tf2_ros::TransformListener tfListener(g_tfBuffer_);
    GetTopicName();
    GetParameter();
    is_init_ = false;
    GenerateParticles();
    last_observed_marker_.clear();
    mu_.resize(3, 1);
    mu_.setZero();
    sigma_.resize(3, 3);
    sigma_.setZero();
    dictionary_ = cv::aruco::getPredefinedDictionary(
        static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(markers_dictionary));
   
    // dictionary_ = cv::aruco::getPredefinedDictionary(
    //     static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(cv::aruco::DICT_ARUCO_ORIGINAL));
        
    //ROS_INFO_STREAM("\033[1;32m----> EKFSLAM started.\033[0m");
    //image_sub = it.subscribeCamera(image_topic, 1, &ArUcoEKFSLAM::ImageCallback, this);
    encoder_sub = node_handle.subscribe(encoder_topic, 5, &ParticleFilter::EncoderCallback, this);
    image_sub = node_handle.subscribe(image_topic, 1, &ParticleFilter::ImageCallback, this);
    true_sub = node_handle.subscribe("true_pose", 1, &ParticleFilter::TrueCallback, this);

    g_landmark_pub = node_handle.advertise<visualization_msgs::MarkerArray>("pf_slam/landmark", 1, this);
    initial_state_pub = node_handle.advertise<std_msgs::Int8>("initial_state", 1, this);
    g_true_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("pf_slam/true_pose",1,this);
    g_robot_pose_pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("pf_slam/pose", 1, this);
    g_img_pub = it.advertise("pf_slam/image", 1, this);
};

void ParticleFilter::GetTopicName()
{
    node_handle.getParam("/aruco_pf_slam/topic/image", image_topic);
    node_handle.getParam("/aruco_pf_slam/topic/encoder", encoder_topic);
}

void ParticleFilter::GetParameter()
{
    node_handle.getParam("/aruco_pf_slam/particle/number", N);
    double fx, fy, cx, cy, k1, k2, p1, p2, k3;
    node_handle.getParam("/aruco_pf_slam/camera/fx", fx);
    node_handle.getParam("/aruco_pf_slam/camera/fy", fy);
    node_handle.getParam("/aruco_pf_slam/camera/cx", cx);
    node_handle.getParam("/aruco_pf_slam/camera/cy", cy);
    node_handle.getParam("/aruco_pf_slam/camera/k1", k1);
    node_handle.getParam("/aruco_pf_slam/camera/k2", k2);
    node_handle.getParam("/aruco_pf_slam/camera/p1", p1);
    node_handle.getParam("/aruco_pf_slam/camera/p2", p2);
    node_handle.getParam("/aruco_pf_slam/camera/k3", k3);

    K_ = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    dist_ = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);

    node_handle.getParam("/aruco_pf_slam/odom/kl", kl);
    node_handle.getParam("/aruco_pf_slam/odom/kr", kr);
    node_handle.getParam("/aruco_pf_slam/odom/b", b);
    //node_handle.getParam("/aruco_pf_slam/covariance/Q_k", Q_k); // encoder error coefficient
    //node_handle.getParam("/aruco_pf_slam/covariance/R_x", R_x); // encoder error coefficient
    //node_handle.getParam("/aruco_pf_slam/covariance/R_y", R_y); // encoder error coefficient
    //node_handle.getParam("/aruco_pf_slam/covariance/R_theta", R_theta); // encoder error coefficient
    node_handle.getParam("/aruco_pf_slam/aruco/markers_dictionary", markers_dictionary);
    node_handle.getParam("/aruco_pf_slam/aruco/marker_length", marker_length);

    std::string robot_frame_base, camera_frame_optical;
    node_handle.getParam("/aruco_pf_slam/frame/robot_frame_base", robot_frame_base);
    node_handle.getParam("/aruco_pf_slam/frame/camera_frame_optical", camera_frame_optical);
    node_handle.getParam("/aruco_pf_slam/const/USEFUL_DISTANCE_THRESHOLD_", USEFUL_DISTANCE_THRESHOLD_);
    //ROS_INFO_STREAM("\033 USEFUL_DISTANCE_THRESHOLD_" << USEFUL_DISTANCE_THRESHOLD_ << std::endl);
    
    node_handle.getParam("/aruco_pf_slam/robot/r2c_translation", r2c_translation);
    node_handle.getParam("/aruco_pf_slam/robot/r2c_rotation", r2c_rotation);
    //TransferVectorToMatrix(r2c_translation, r2c_rotation, T_r_c_);
    
    getTransformStamped(robot_frame_base, camera_frame_optical, transformStamped_r2c);
    ROS_INFO_STREAM("\033[1;32m----> Parameter Loaded.\033[0m");
}

void ParticleFilter::getTransformStamped(const std::string &target_frame, const std::string &source_frame, geometry_msgs::TransformStamped &transform_stamped)
{
    transform_stamped.transform.translation.x = r2c_translation[0];
    transform_stamped.transform.translation.y = r2c_translation[1];
    transform_stamped.transform.translation.z = r2c_translation[2];
    transform_stamped.transform.rotation.x = r2c_rotation[0];
    transform_stamped.transform.rotation.y = r2c_rotation[1];
    transform_stamped.transform.rotation.z = r2c_rotation[2];
    transform_stamped.transform.rotation.w = r2c_rotation[3];
}

void ParticleFilter::GenerateParticles()
{
    Particles.clear();
    Eigen::Vector3d pose;
    pose.resize(3, 1);
    pose.setZero();
    Eigen::Matrix3d cov;
    cov.resize(3, 3);
    cov.setZero();
    //struct Particle InitialParticle = Particle (pose , cov, 1.0);
    for (int i = 0; i < N ; i++)
    {
        struct Particle InitialParticle = Particle (pose , cov, 1.0);
        Particles.push_back(InitialParticle);
    }
    aruco_id_map.clear();
    ROS_INFO_STREAM("initialed !!!");
}

void ParticleFilter::EncoderCallback(const geometry_msgs::QuaternionStamped::ConstPtr &encoder_ptr)
{
    ROS_INFO_STREAM("encoder_callback:");
    double enr = encoder_ptr->quaternion.x;
    double enl = encoder_ptr->quaternion.y;

    addEncoder(enl, enr);
    MaxWeightPaticle();
    visualization_msgs::MarkerArray marker = toRosMarker(1);
    g_landmark_pub.publish(marker);


    geometry_msgs::PoseWithCovarianceStamped pose = toRosPose();
    g_robot_pose_pub.publish(pose);
    g_true_pose_pub.publish(True_pose_);
}

void ParticleFilter::ImageCallback(const sensor_msgs::ImageConstPtr &img_ptr)
{
    // if (!camera_inited_)
    // {
    //     setCameraParameters(parseCameraInfo(cinfo));
    //     ROS_INFO_STREAM("camera parameter" << K_ << std::endl << dist_ << std::endl);
    //     camera_inited_ = true;
    // }
    ROS_INFO_STREAM("image_callback:");
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_ptr);

    /***** add image *****/
    addImage(cv_ptr->image);

    /***** pubish marked image*****/
    cv::Mat img = markedImg();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    g_img_pub.publish(msg);
}

void ParticleFilter::TrueCallback(const geometry_msgs::PoseStamped &true_ptr)
{
    True_pose_ = true_ptr;
    // ROS_INFO_STREAM("\033[1;32m----> transformStamped_r2c is.\033[0m" << transformStamped_r2c);
    // ROS_INFO_STREAM(" The True pose is " << True_pose_.pose.position.x);
}

void ParticleFilter::addEncoder(const double &enl, const double &enr)
{
    // ROS_INFO_STREAM("Encoder data reveived");
    if (is_init_ == false)
    {
        last_time_ = ros::Time::now();
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
    double delta_x, delta_y;
    delta_x = delta_s * cos(temp_angle);
    delta_y = delta_s * sin(temp_angle);


    Eigen::Matrix<double, 3, 2> wkh;
    wkh << cos_tmp_th, cos_tmp_th, sin_tmp_th, sin_tmp_th, 1 / b, -1 / b;
    wkh = (0.5 * kl * dt) * wkh;

    Eigen::Matrix2d sigma_u;
    double Q_k = 0.1;
    sigma_u << Q_k * fabs(enl), 0.0, 0.0, Q_k * fabs(enr);
    Eigen::MatrixXd encoder_noise = wkh * sigma_u * wkh.transpose();
    //ROS_INFO_STREAM("encoder information:" << delta_x << std::endl << delta_y << std::endl);
    for (int i = 0; i < N ; i++)
    {
        Particles[i].addencoder(delta_x, delta_y, delta_theta, encoder_noise);
    }
}

void ParticleFilter::addImage(const Mat &img)
{
    // ROS_INFO_STREAM("Image data reveived");
    ROS_INFO_STREAM(" \n\n\n  Image data reveived " << std::endl
                    // "z_hat:" << z_hat << std::endl
                    // << "mu:" << mu_ << std::endl
                    // << "sigma:" << sigma_ << std::endl
    );
    if (is_init_ == false)
        return;
    
    std::vector<ArucoMarker> obs_;
    getObservations(img, obs_);
    ROS_INFO_STREAM("obs_.size:" << obs_.size() << std::endl);
    if (obs_.size() > 0)
    {
        ROS_INFO_STREAM("obs processing:" <<  std::endl);
        for (int j = 0; j < obs_.size() ; j++)
        {
            if (obs_[j].aruco_index_ == -1)
            {
                //ROS_ERROR_STREAM("insert new makers:" <<  std::endl);
                aruco_id_map.insert(std::pair<int, int>{obs_[j].aruco_id_, (aruco_id_map.size() )});
                for (int i = 0; i < N ; i++)
                {
                    Particles[i].insert(obs_[j]);
                }
            }
            else
            {
                for (int i = 0; i< N; i++)
                {
                    //ROS_INFO_STREAM("particle updates:" <<  std::endl);
                    Particles[i].update(obs_[j]);
                }
            }
        }
        ResampleParticles();
    }
    // ROS_INFO_STREAM("Image data precess finished "
    //                 // "z_hat:" << z_hat << std::endl
    //                 << std::endl << "mu:" << mu_ << std::endl
    //                 << "sigma:" << sigma_ << std::endl);
}

//get the observation
int ParticleFilter::getObservations(const cv::Mat &img, std::vector<ArucoMarker> &obs)
{
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> IDs;
    std::vector<cv::Vec3d> rvs, tvs;
    detected_markers_.markers.clear();
    cv::aruco::detectMarkers(img, dictionary_, marker_corners, IDs);
    cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length, K_, dist_, rvs, tvs);

    /* draw all marks */
    marker_img_ = img.clone();
    //cv::aruco::drawDetectedMarkers(marker_img_, marker_corners, IDs);
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
            //ROS_ERROR_STREAM("USEFUL_DISTANCE_THRESHOLD_:"<<USEFUL_DISTANCE_THRESHOLD_ << "the distance of the landmark is :" << dist);
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
        GenerateMarker(IDs[i], marker_length, tvs[i][0], tvs[i][1], tvs[i][2], _transform.getRotation(), _marker, color, ros::Duration(0.1));
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
        Eigen::Vector3d mean;
        mean << x, y ,theta;
        ArucoMarker ob(aruco_id, x, y, theta, mean, covariance);
        int aruco_index;
        checkLandmark(aruco_id, aruco_index);
        ob.aruco_index_ = aruco_index;
        obs.push_back(ob);
    } //for all detected markers
    return obs.size();
}

//resample the particles
void ParticleFilter::ResampleParticles()
{
    std::vector<double> weights;
    std::vector<struct Particle> NewParticles;
    double MaxWeight = 0.0;
    for (int i = 0; i < N; i++)
    {
        weights.push_back(Particles[i].weight);
        if (Particles[i].weight > MaxWeight)
        {
            MaxWeight = Particles[i].weight;
        }
    }

    int index = int((rand() / (RAND_MAX)) * N);
    ROS_INFO_STREAM(" ResampleParticles " <<  MaxWeight << std::endl);
    double beta = 0.0;
    for (int i  = 0; i < N; i++)
    {
        //ROS_INFO_STREAM("Resample" <<  std::endl);
        beta += (rand() / (RAND_MAX)) *2.0 * MaxWeight;
        while(beta > weights[index])
        {
            beta -= weights[index];
            index = (index + 1)% N;
        } 
        struct Particle NewParticle;
        NewParticle = Particles[index];
        NewParticle.weight = 1;
        NewParticles.push_back(NewParticle);
    }
    Particles.clear();
    //ROS_INFO_STREAM("ResampleParticles 3.0" <<  std::endl);
    Particles = NewParticles;
    NewParticles.clear();
}

bool ParticleFilter::checkLandmark(const int &aruco_id, int &landmark_idx)
{

    if (!aruco_id_map.empty() && aruco_id_map.end() != aruco_id_map.find(aruco_id))
    {
        landmark_idx = aruco_id_map.at(aruco_id);
        //ROS_INFO_STREAM("aruco_id:" << aruco_id << "index:" << landmark_idx);
        return true;
    }
    else
        landmark_idx = -1;
        ROS_INFO_STREAM("it is a new landmark" );
    return false;
}

void ParticleFilter::CalculateCovariance(const cv::Vec3d &tvec, const cv::Vec3d &rvec, const std::vector<cv::Point2f> &marker_corners, Eigen::Matrix3d &covariance)
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
    // double rmserror = (sqrt(totalError) / (double)objectPoints_.size())/20;
    // double object_error = (rmserror / dist(marker_corners[0], marker_corners[2])) *
    //                    (norm(tvec) / marker_length);
    // ROS_INFO_STREAM(" \n  variace is " << object_error << std::endl);
    // covariance << object_error * 2, 0, 0, 0, object_error * 2, 0, 0, 0, object_error + 1e-3;

    double rmserror = sqrt(totalError / (double)objectPoints_.size());
    double variace = rmserror / 80;
    ROS_INFO_STREAM(" \n  variace is " << variace << std::endl);
    covariance << variace, 0, 0, 0, variace, 0, 0, 0, variace/10;
}

visualization_msgs::MarkerArray ParticleFilter::toRosMarker(double scale)
{
    visualization_msgs::MarkerArray markers;
    for (int i = 0; i < Particles[BestParticle_ID].particle_map_.size(); i++)
    {
        double &mx = Particles[BestParticle_ID].particle_map_[i].x_;
        double &my = Particles[BestParticle_ID].particle_map_[i].y_;
        double &mtheta = Particles[BestParticle_ID].particle_map_[i].theta_;

        /* 计算地图点的协方差椭圆角度以及轴长 */
        Eigen::Matrix2d sigma_m = Particles[BestParticle_ID].particle_map_[i].observe_covariance_.block(0, 0, 2, 2);
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
        marker.ns = "pf_slam";
        marker.id = Particles[BestParticle_ID].particle_map_[i].aruco_id_;
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
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        markers.markers.push_back(marker);
    } // for all mpts
    // ROS_INFO_STREAM("" << markers.markers.size() << " the numbe of maker:"<<Particles[BestParticle_ID].particle_map_.size() <<" the size of aruco map is  " << aruco_id_map.size() << std::endl);
    return markers;
}

void ParticleFilter::MaxWeightPaticle()
{
    std::vector<double> weights;
    int MaxWeightID = 0;
    double MaxWeight = 0;
    for (int i = 0; i < N; i++)
    {
        if (Particles[i].weight > MaxWeight)
        {
            MaxWeightID = i;
            MaxWeight = Particles[i].weight;
        }
    }
    BestParticle_ID = MaxWeightID;
    BestParticle = Particles[MaxWeightID];
    mu_ = BestParticle.pose;
    sigma_ = BestParticle.cov;
}

bool ParticleFilter::GenerateMarker(int id, double length, double x, double y, double z, tf2::Quaternion q, visualization_msgs::Marker &marker_, std_msgs::ColorRGBA color, ros::Duration lifetime)
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

void ParticleFilter::fillTransform(tf2::Transform &transform_, const cv::Vec3d &rvec, const cv::Vec3d &tvec)
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