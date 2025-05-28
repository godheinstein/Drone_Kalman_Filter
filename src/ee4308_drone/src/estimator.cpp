#include "ee4308_drone/estimator.hpp"
// By Hein & Jaeuk & LG
namespace ee4308::drone
{
    // ================================ IMU sub callback / EKF Prediction ========================================
    void Estimator::cbIMU(const sensor_msgs::msg::Imu msg)
    {
        rclcpp::Time tnow = msg.header.stamp;
        double dt = tnow.seconds() - last_predict_time_;
        last_predict_time_ = tnow.seconds();

        if (dt < ee4308::THRES)
            return;

        // !!! NOT ALLOWED TO USE ORIENTATION FROM IMU as ORIENTATION IS DERIVED FROM ANGULAR VELOCTIY !!!

        // !!! Store the states in Xx_, Xy_, Xz_, and Xa_.
        //      Store the covariances in Px_, Py_, Pz_, and Pa_.
        //      Required for terminal printing during demonstration.

        // ==== make use of ====
        // msg.linear_acceleration
        // msg.angular_velocity
        // GRAVITY
        // var_imu_x_, var_imu_y_, var_imu_z_, var_imu_a_
        // Xx_, Xy_, Xz_, Xa_
        // Px_, Py_, Pz_, Pa_
        // dt
        // std::cos(), std::sin()
        // ====  ====

        // Yaw orientation regarding world for xy plane rotation
        double psi = Xa_(0);
        Eigen::Matrix2d R; R << cos(psi), -sin(psi), sin(psi), cos(psi);
        Eigen::Vector2d v; v << msg.linear_acceleration.x, msg.linear_acceleration.y;
        Eigen::Matrix2d F; F << 1, dt,
                                 0, 1;
        Eigen::Matrix2d Q; Q << var_imu_x_, 0,
                                 0, var_imu_y_; // covariance matrix

        // ==== Xx_ ====
        // Control input model
        Eigen::Matrix2d Wx; Wx << dt*dt/2, 0,
                                       dt, 0;
        // 1. Internal state prediction from kinematic
        Wx = Wx * R; // rotate the control input model // newly added
        Xx_ = F * Xx_ + Wx * v;
        // 2. Internal state covariance update
        Px_ = F * Px_ * F.transpose() + Wx * Q * Wx.transpose();
        // ==== Xx_ ====    


        // ==== Xy_ ====
        // Control input model
        Eigen::Matrix2d Wy; Wy << 0, dt*dt/2,
                                  0,      dt;
        // 1. Internal state prediction from kinematics 
        Wy = Wy * R; // rotate the control input model // newly added
        Xy_ = F * Xy_ + Wy * v;
        // 2. Internal state covariance update
        Py_ = F * Py_ * F.transpose() + Wy * Q * Wy.transpose();
        // ==== Xy_ ====

        // ==== Xz_ ====
        // Input and input variance
        double Uz = msg.linear_acceleration.z - GRAVITY;
        double Qz = var_imu_z_; // tune
        // State transition matrix, Control input model
        Eigen::Matrix3d Fz; Fz <<  1,dt, 0,
                                   0, 1, 0,
                                   0, 0, 1; // bias term!!!
        Eigen::Vector3d Wz; Wz << dt*dt/2, dt, 0;
        // 1. Internal state prediction from kinematics 
        Xz_ = Fz * Xz_ + Wz * Uz;
        // 2. Internal state covariance update
        Pz_ = Fz * Pz_ * Fz.transpose() + Wz * Qz * Wz.transpose();
        // ==== Xz_ ====

        // ==== Xa_ ====
        // Input and input variance
        double Ua = msg.angular_velocity.z;
        double Qa = var_imu_a_; // tune
        // State transition matrix, Control input model
        Eigen::Matrix2d Fa; Fa << 1, 0, 0, 0;
        Eigen::Vector2d Wa; Wa << dt, 1;
        // 1. Internal state prediction from kinematics 
        Xa_ = Fa * Xa_ + Wa * Ua;
        // 2. Internal state covariance update
        Pa_ = Fa * Pa_ * Fa.transpose() + Wa * Qa * Wa.transpose();
        // ==== Xa_ ====

        // Custom logger
        if (log_imu_x_.is_open())
            log_imu_x_ << last_predict_time_ << "," << v(0) << "\n";
        if (log_imu_y_.is_open())
            log_imu_y_ << last_predict_time_ << "," << v(1) << "\n";
        if (log_imu_z_.is_open())
            log_imu_z_ << last_predict_time_ << "," << Uz << "\n";
        if (log_imu_a_.is_open())
            log_imu_a_ << last_predict_time_ << "," << Ua << "\n";
        
        return;
    }

    // ================================ Sonar sub callback / EKF Correction ========================================
    void Estimator::cbSonar(const sensor_msgs::msg::Range msg)
    {
        (void)msg;

        if (msg.range > msg.max_range)
        { // skip erroneous measurements
            return;
        }

        // ==== make use of ====
        // msg.range
        // Ysonar_
        // var_sonar_
        // Xz_
        // Pz_
        // .transpose()
        // ====  ====

        // ==== Xz_ ====
        // Measure and measure variance
        Ysonar_ = msg.range;
        // Observation matrix
        Eigen::RowVector3d H; H << 1, 0, 0; // including zero influence on barometer bias
        // Innovation and innovation covariance
        double innov = Ysonar_ - H * Xz_;
        double innov_cov = double(H * Pz_ * H.transpose()) + var_sonar_;
        // 0. Kalman gain
        Eigen::Vector3d K = Pz_ * H.transpose() / innov_cov;
        // 1. Internal state correction based on innovation
        Xz_ = Xz_ + K * innov;
        // 2. Internal state covariance update
        Pz_ = Pz_ - K * H * Pz_;
        // ==== Xz_ ====

        // Custom logger
        if (log_sonar_.is_open())
            log_sonar_ << last_predict_time_ << "," << Ysonar_ << "\n";

        return;
    }

    // ================================ GPS sub callback / EKF Correction ========================================
    Eigen::Vector3d Estimator::getECEF(
        const double &sin_lat, const double &cos_lat,
        const double &sin_lon, const double &cos_lon,
        const double &alt)
    {
        Eigen::Vector3d ECEF;
        // ==== make use of ====
        // RAD_POLAR, RAD_EQUATOR
        // std::sqrt()
        // all the arguments above.
        // ====  ====

        // calculate the prime vertical radius of curvature 
        double e2 = 1 - (RAD_POLAR * RAD_POLAR) / (RAD_EQUATOR * RAD_EQUATOR);
        double N = RAD_EQUATOR / sqrt(1- e2 * sin_lat * sin_lat); 

        // calculate the ECEF coords 
        ECEF(0) = (N + alt) * cos_lat * cos_lon; 
        ECEF(1) = (N + alt) * cos_lat * sin_lon;
        ECEF(2) = ((RAD_POLAR * RAD_POLAR) / (RAD_EQUATOR * RAD_EQUATOR) * N + alt) * sin_lat; 

        return ECEF;
    }


    void Estimator::cbGPS(const sensor_msgs::msg::NavSatFix msg)
    {
        (void)msg;

        constexpr double DEG2RAD = M_PI / 180;
        double lat = -msg.latitude * DEG2RAD;  // !!! Gazebo spherical coordinates have a bug. Need to negate.
        double lon = -msg.longitude * DEG2RAD; // !!! Gazebo spherical coordinates have a bug. Need to negate.
        double alt = msg.altitude;

        double sin_lat = sin(lat);
        double cos_lat = cos(lat);
        double sin_lon = sin(lon);
        double cos_lon = cos(lon);

        if (initialized_ecef_ == false)
        {
            initial_ECEF_ = getECEF(sin_lat, cos_lat, sin_lon, cos_lon, alt);
            initialized_ecef_ = true;
            return;
        }

        Eigen::Vector3d ECEF = getECEF(sin_lat, cos_lat, sin_lon, cos_lon, alt);

        // ==== make use of ====
        // Ygps_
        // initial_position_
        // initial_ECEF_
        // sin_lat, cost_lat, sin_lon, cos_lon, alt
        // var_gps_x_, var_gps_y_, var_gps_z_
        // Px_, Py_, Pz_
        // Xx_, Xy_, Xz_
        //
        // - other Eigen methods like .transpose().
        // - Possible to divide a VectorXd element-wise by a double by using the divide operator '/'.
        // - Matrix multiplication using the times operator '*'.
        // ====  ====
        
        // convert to NED frame
        Eigen::Matrix3d Re_n; 
        Re_n << -sin_lat*cos_lon, -sin_lon, -cos_lat*cos_lon,
                -sin_lat*sin_lon, cos_lon,  -cos_lat*sin_lon,
                cos_lat,          0,        -sin_lat;

        Eigen::Vector3d NED = Re_n.transpose() * (ECEF - initial_ECEF_); 

        // convert to world frame 
        Eigen::Matrix3d Rm_n;
        Rm_n << 0, 1, 0,
                1, 0 ,0,
                0, 0, -1; 

        Ygps_ = Rm_n * NED + initial_position_; 

        // correction for each axis
        // X-axis correction 
        Eigen::RowVector2d Hx; Hx << 1, 0;
        double innov_x = Ygps_(0) - Hx * Xx_;
        double innov_cov_x = Hx * Px_ * Hx.transpose() + var_gps_x_; // tune
        Eigen::Vector2d Kx = Px_ * Hx.transpose() / innov_cov_x;
        Xx_ = Xx_ + Kx * innov_x;
        Px_ = Px_ - Kx * Hx * Px_;

        // Y-axis correction
        Eigen::RowVector2d Hy; Hy << 1, 0;
        double innov_y = Ygps_(1) - Hy * Xy_;
        double innov_cov_y = Hy * Py_ * Hy.transpose() + var_gps_y_; // tune
        Eigen::Vector2d Ky = Py_ * Hy.transpose() / innov_cov_y;
        Xy_ = Xy_ + Ky * innov_y;
        Py_ = Py_ - Ky * Hy * Py_;

        // Z-axis correction
        Eigen::RowVector3d Hz; Hz << 1, 0, 0;
        double innov_z = Ygps_(2) - Hz * Xz_;
        double innov_cov_z = Hz * Pz_ * Hz.transpose() + var_gps_z_; // tune
        Eigen::Vector3d Kz = Pz_ * Hz.transpose() / innov_cov_z;
        Xz_ = Xz_ + Kz * innov_z;
        Pz_ = Pz_ - Kz * Hz * Pz_;

        // Custom logger
        if (log_gps_x_.is_open())
            log_gps_x_ << last_predict_time_ << "," << Ygps_(0) << "\n";
        if (log_gps_y_.is_open())
            log_gps_y_ << last_predict_time_ << "," << Ygps_(1) << "\n";
        if (log_gps_z_.is_open())
            log_gps_z_ << last_predict_time_ << "," << Ygps_(2) << "\n";

        return;
    }

    // ================================ Magnetic sub callback / EKF Correction ========================================
    void Estimator::cbMagnetic(const geometry_msgs::msg::Vector3Stamped msg)
    {
        (void)msg;
        // Along the horizontal plane, the magnetic north in Gazebo points towards +x, when it should point to +y. It is a bug.
        // As the drone always starts pointing towards +x, there is no need to offset the calculation with an initial heading.
        
        // magnetic force direction in drone's z-axis can be ignored.

        // ==== make use of ====
        // Ymagnet_
        // msg.vector.x // the magnetic force direction along drone's x-axis.
        // msg.vector.y // the magnetic force direction along drone's y-axis.
        // std::atan2()
        // Xa_
        // Pa_
        // var_magnet_
        // .transpose()
        // limitAngle()
        // ====  ====

        // ==== Xa_ ====
        // Measure and measure variance
        Ymagnet_ = -std::atan2(msg.vector.y, msg.vector.x);
        // Observation matrix
        Eigen::RowVector2d H; H << 1, 0;
        // Innovation and innovation covariance
        double innov = limitAngle(Ymagnet_ - Xa_(0));  // difference in yaw, angle-limited
        double innov_cov = double(H * Pa_ * H.transpose()) + var_magnet_;
        // 0. Kalman gain
        Eigen::Vector2d K = Pa_ * H.transpose() / innov_cov;
        // 1. Internal state correction based on innovation
        Xa_ = Xa_ + K * innov;
        Xa_(0) = limitAngle(Xa_(0));  // Ensure yaw remains bounded
        // 2. Internal state covariance update
        Pa_ = Pa_ - K * H * Pa_;
        // ==== Xa_ ====

        // Custom logger
        if (log_magnet_.is_open())
            log_magnet_ << last_predict_time_ << "," << Ymagnet_ << "\n";

        return;
    }

    // ================================ Baro sub callback / EKF Correction ========================================
    void Estimator::cbBaro(const geometry_msgs::msg::PointStamped msg)
    {
        // if this section is done, Pz_ has to be augmented with the barometer bias to become a 3-state vector.

        (void)msg;

        // ==== make use of ====
        // Ybaro_ 
        // msg.point.z
        // var_baro_
        // Pz_
        // Xz_
        // .transpose()
        // ====  ====

        // Extending dimension for Xz_, Pz_: Modify F, W, H, and soon K

        // ==== Xz_ ====
        // Measure and measure variance
        Ybaro_ = msg.point.z;
        // Observation matrix
        Eigen::RowVector3d H; H << 1, 0, baro_w_; // Y = (wz + b) + e
        // Innovation and innovation covariance
        double innov = Ybaro_ - H * Xz_;  // difference in z
        double innov_cov = double(H * Pz_ * H.transpose()) + var_baro_;
        // 0. Kalman gain
        Eigen::Vector3d K = Pz_ * H.transpose() / innov_cov;
        // 1. Internal state correction based on innovation
        Xz_= Xz_ + K * innov;
        // 2. Internal state covariance update
        Pz_ = Pz_ - K * H * Pz_;
        // ==== Xz_ ====

        // Custom logger
        if (log_baro_.is_open())
            log_baro_ << last_predict_time_ << "," << Ybaro_ << "\n";

        return;
    }


    Estimator::Estimator(
        const double &initial_x, const double &initial_y, const double &initial_z,
        const std::string &name = "estimator")
        : Node(name)
    {
        // parameters
        initParam(this, "frequency", frequency_, 10.0);
        initParam(this, "var_imu_x", var_imu_x_, 0.2);
        initParam(this, "var_imu_y", var_imu_y_, 0.2);
        initParam(this, "var_imu_z", var_imu_z_, 0.2);
        initParam(this, "var_imu_a", var_imu_a_, 0.2);
        initParam(this, "var_gps_x", var_gps_x_, 0.2);
        initParam(this, "var_gps_y", var_gps_y_, 0.2);
        initParam(this, "var_gps_z", var_gps_z_, 0.2);
        initParam(this, "var_baro", var_baro_, 0.2);
        initParam(this, "var_sonar", var_sonar_, 0.2);
        initParam(this, "var_magnet", var_magnet_, 0.2);
        initParam(this, "verbose", verbose_, true);
        
        initParam(this, "logvar", logvar_, false);       // Custom logger
        initParam(this, "logerr", logerr_, false);       // Custom logger
        initParam(this, "baro_w", baro_w_, 1.0);        // Barometer weight
        initParam(this, "baro_b", baro_b_, 0.0);        // Barometer bias

        // topics
        pub_est_odom_ = create_publisher<nav_msgs::msg::Odometry>("est_odom", rclcpp::ServicesQoS());
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", qos, std::bind(&Estimator::cbOdom, this, std::placeholders::_1)); // ground truth in sim.
        sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", qos, std::bind(&Estimator::cbGPS, this, std::placeholders::_1));
        sub_sonar_ = create_subscription<sensor_msgs::msg::Range>(
            "sonar", qos, std::bind(&Estimator::cbSonar, this, std::placeholders::_1));
        sub_magnetic_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "magnetic", qos, std::bind(&Estimator::cbMagnetic, this, std::placeholders::_1));
        sub_baro_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "altitude", qos, std::bind(&Estimator::cbBaro, this, std::placeholders::_1));
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", qos, std::bind(&Estimator::cbIMU, this, std::placeholders::_1));

        // states
        initial_position_ << initial_x, initial_y, initial_z;
        Xx_ << initial_x, 0;
        Xy_ << initial_y, 0;
        Xz_ << initial_z, 0, baro_b_; // Extension to 3d for barometer
        Xa_ << 0, 0;
        Px_ = Eigen::Matrix2d::Constant(1e3),
        Py_ = Eigen::Matrix2d::Constant(1e3),
        Pz_ = Eigen::Matrix3d::Constant(1e3); // Extension to 3d for barometer
        Pa_ = Eigen::Matrix2d::Constant(1e3);
        initial_ECEF_ << NAN, NAN, NAN;
        Ygps_ << NAN, NAN, NAN;
        Ymagnet_ = NAN;
        Ybaro_ = NAN;
        Ysonar_ = NAN;

        last_predict_time_ = this->now().seconds();
        initialized_ecef_ = false;
        initialized_magnetic_ = false;

        timer_ = this->create_wall_timer(
            1s / frequency_,
            std::bind(&Estimator::cbTimer, this));
        
        // Custom logger
        if (logvar_) {
            log_imu_x_.open("mylog/imu_x.csv", std::ios::out | std::ios::app);
            log_imu_y_.open("mylog/imu_y.csv", std::ios::out | std::ios::app);
            log_imu_z_.open("mylog/imu_z.csv", std::ios::out | std::ios::app);
            log_imu_a_.open("mylog/imu_a.csv", std::ios::out | std::ios::app);
            log_gps_x_.open("mylog/gps_x.csv", std::ios::out | std::ios::app);
            log_gps_y_.open("mylog/gps_y.csv", std::ios::out | std::ios::app);
            log_gps_z_.open("mylog/gps_z.csv", std::ios::out | std::ios::app);
            log_baro_.open("mylog/baro.csv", std::ios::out | std::ios::app);
            log_sonar_.open("mylog/sonar.csv", std::ios::out | std::ios::app);
            log_magnet_.open("mylog/magnet.csv", std::ios::out | std::ios::app);
            
            log_imu_x_ << "time,Ux\n";
            log_imu_y_ << "time,Uy\n";
            log_imu_z_ << "time,Uz\n";
            log_imu_a_ << "time,Ua\n";
            log_gps_x_ << "time,gps_x\n";
            log_gps_y_ << "time,gps_y\n";
            log_gps_z_ << "time,gps_z\n";
            log_baro_  << "time,point\n";
            log_sonar_ << "time,range\n";
            log_magnet_ << "time,bearing\n";
        }
        
        if (logerr_) {
            log_err_pose_.open("mylog/err_pose.csv", std::ios::out | std::ios::app);
            log_err_twist_.open("mylog/err_twist.csv", std::ios::out | std::ios::app);
            
            log_err_pose_ << "time,x,y,z,yaw\n";
            log_err_twist_ << "time,v_x,v_y,v_z,v_yaw\n";
        }
    }

    void Estimator::cbTimer()
    {
        nav_msgs::msg::Odometry est_odom;

        est_odom.header.stamp = this->now();
        est_odom.child_frame_id = "";     //; std::string(this->get_namespace()) + "/base_footprint";
        est_odom.header.frame_id = "map"; //; std::string(this->get_namespace()) + "/odom";

        est_odom.pose.pose.position.x = Xx_[0];
        est_odom.pose.pose.position.y = Xy_[0];
        est_odom.pose.pose.position.z = Xz_[0];
        getQuaternionFromYaw(Xa_[0], est_odom.pose.pose.orientation);
        est_odom.pose.covariance[0] = Px_(0, 0);
        est_odom.pose.covariance[7] = Py_(0, 0);
        est_odom.pose.covariance[14] = Pz_(0, 0);
        est_odom.pose.covariance[35] = Pa_(0, 0);

        est_odom.twist.twist.linear.x = Xx_[1];
        est_odom.twist.twist.linear.y = Xy_[1];
        est_odom.twist.twist.linear.z = Xz_[1];
        est_odom.twist.twist.angular.z = Xa_[1];
        est_odom.twist.covariance[0] = Px_(1, 1);
        est_odom.twist.covariance[7] = Py_(1, 1);
        est_odom.twist.covariance[14] = Pz_(1, 1);
        est_odom.twist.covariance[35] = Pa_(1, 1);

        pub_est_odom_->publish(est_odom);

        // Custom logger
        if (log_err_pose_.is_open())
            log_err_pose_ << last_predict_time_ << ","
                          << odom_.pose.pose.position.x - Xx_(0) << ","
                          << odom_.pose.pose.position.y - Xy_(0) << ","
                          << odom_.pose.pose.position.z - Xz_(0) << ","
                          << limitAngle(getYawFromQuaternion(odom_.pose.pose.orientation) - Xa_(0))
                          << "\n";
        if (log_err_twist_.is_open())
            log_err_twist_ << last_predict_time_ << ","
                           << odom_.twist.twist.linear.x - Xx_(1) << ","
                           << odom_.twist.twist.linear.y - Xy_(1) << ","
                           << odom_.twist.twist.linear.z - Xz_(1) << ","
                           << odom_.twist.twist.angular.z - Xa_(1)
                           << "\n";

        if (verbose_)
        {

            RCLCPP_INFO_STREAM(this->get_logger(), "===");
            std::cout << std::fixed;
            std::cout << "    Pose("
                      << std::setw(7) << std::setprecision(3) << Xx_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Xy_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Xz_(0) << ","
                      << std::setw(7) << std::setprecision(3) << limitAngle(Xa_(0)) << ")"
                      << std::endl;
            std::cout << "   Twist("
                      << std::setw(7) << std::setprecision(3) << Xx_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xy_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xz_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xa_(1) << ")"
                      << std::endl;
            std::cout << " ErrPose("
                      << std::setw(7) << std::setprecision(3) << odom_.pose.pose.position.x - Xx_(0) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.pose.pose.position.y - Xy_(0) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.pose.pose.position.z - Xz_(0) << ","
                      << std::setw(7) << std::setprecision(3) << ee4308::limitAngle(ee4308::getYawFromQuaternion(odom_.pose.pose.orientation) - Xa_(0)) << ")"
                      << std::endl;
            std::cout << "ErrTwist("
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.linear.x - Xx_(1) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.linear.y - Xy_(1) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.linear.z - Xz_(1) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.angular.z - Xa_(1) << ")"
                      << std::endl;
            std::cout << "     GPS("
                      << std::setw(7) << std::setprecision(3) << Ygps_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Ygps_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Ygps_(2) << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            std::cout << "    Baro("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Ybaro_ << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            std::cout << "   BBias("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Xz_(2) << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            std::cout << "   Sonar("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Ysonar_ << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            std::cout << "   Magnt("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Ymagnet_ << ")"
                      << std::endl;
        }
    }

    void Estimator::cbOdom(const nav_msgs::msg::Odometry msg)
    {
        odom_ = msg;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    double initial_x = std::stod(argv[1]);
    double initial_y = std::stod(argv[2]);
    double initial_z = std::stod(argv[3]);

    rclcpp::spin(std::make_shared<ee4308::drone::Estimator>(initial_x, initial_y, initial_z));
    rclcpp::shutdown();
    return 0;
}