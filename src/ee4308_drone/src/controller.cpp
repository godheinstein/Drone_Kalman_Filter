#include "ee4308_drone/controller.hpp"
// By NZ
namespace ee4308::drone
{
    Controller::Controller(const std::string &name = "controller_ee4308") : Node(name)
    {
        initParam(this, "frequency", frequency_, 10.0);
        initParam(this, "use_ground_truth", use_ground_truth_, false);
        initParam(this, "enable", enable_, true);
        initParam(this, "lookahead_distance", lookahead_distance_, 1.0);
        initParam(this, "max_xy_vel", max_xy_vel_, 1.0);
        initParam(this, "max_z_vel", max_z_vel_, 0.5);
        initParam(this, "yaw_vel", yaw_vel_, -0.3);
        initParam(this, "kp_xy", kp_xy_, 1.0);
        initParam(this, "kp_z", kp_z_, 1.0);

        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::ServicesQoS());
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            (use_ground_truth_ ? "odom" : "est_odom"), rclcpp::SensorDataQoS(),
            std::bind(&Controller::cbOdom, this, std::placeholders::_1));
        sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::cbPlan, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(1s / frequency_, std::bind(&Controller::cbTimer, this));
    }

    void Controller::cbOdom(const nav_msgs::msg::Odometry msg)
    {
        odom_ = msg;
    }

    void Controller::cbPlan(const nav_msgs::msg::Path msg)
    {
        plan_ = msg;
    }

    void Controller::cbTimer()
    {
        // 1. If controller is disabled, do nothing
        if (!enable_) {
            publishCmdVel(0, 0, 0, 0);
            return;
        }

        // 2. If path is empty, hover in place
        if (plan_.poses.empty())
        {
            // RCLCPP_WARN_STREAM(this->get_logger(), "No path published");
            publishCmdVel(0, 0, 0, 0);
            return;
        }

        // ==== make use of ====
        // plan_.poses
        // odom_
        // ee4308::getYawFromQuaternion()
        // std::hypot()
        // std::clamp()
        // std::cos(), std::sin() 
        // lookahead_distance_
        // kp_xy_
        // kp_z_
        // max_xy_vel_
        // max_z_vel_
        // yaw_vel_
        // publishCmdVel()
        // ==== ====

        // Extract drone's current position and yaw
        const auto &pose = odom_.pose.pose;
        double drone_x = pose.position.x;
        double drone_y = pose.position.y;
        double drone_z = pose.position.z;
        double drone_yaw = ee4308::getYawFromQuaternion(pose.orientation);

        // 3. Find the closest point in the plan
        std::vector<double> distances;
        distances.reserve(plan_.poses.size());

        size_t i = 0;
        int closest_idx = 0;
        int lookahead_idx = plan_.poses.size() - 1;
        double min_dist = std::numeric_limits<double>::max();

        for (; i < plan_.poses.size(); i++)
        {
            double px = plan_.poses[i].pose.position.x;
            double py = plan_.poses[i].pose.position.y;
            double pz = plan_.poses[i].pose.position.z;
            double dist = std::hypot(px - drone_x, py - drone_y, pz - drone_z);
            distances.push_back(dist);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // 4. Determine the lookahead point
        //    i) If final goal is within lookahead distance, use the goal
        //    ii) Otherwise, pick the first point >= lookahead_distance_ from the closest point

        for (i = closest_idx; i < plan_.poses.size(); i++)
        {
            if (distances[i] > lookahead_distance_)
            {
                lookahead_idx = i;
                break;
            }
        }

        // The lookahead point
        const auto &lookahead_pose = plan_.poses[lookahead_idx].pose.position;
        double lx = lookahead_pose.x;
        double ly = lookahead_pose.y;
        double lz = lookahead_pose.z;

        // 5. Compute the offset in the world frame
        double dx_world = lx - drone_x;
        double dy_world = ly - drone_y;
        double dz = lz - drone_z;

        // Convert offset into drone's local frame
        // (assuming drone's x-axis is yaw forward, y-axis is yaw left, in 2D plane)
        double dx_local =  dx_world * std::cos(drone_yaw) + dy_world * std::sin(drone_yaw);
        double dy_local = -dx_world * std::sin(drone_yaw) + dy_world * std::cos(drone_yaw);

        // 6. Apply proportional gains to get raw velocities
        double x_vel = kp_xy_ * dx_local;
        double y_vel = kp_xy_ * dy_local;
        double z_vel = kp_z_  * dz;

        // 7. Clamp horizontal velocity
        double xy_speed = std::hypot(x_vel, y_vel);
        if (xy_speed > max_xy_vel_)
        {
            double scale = max_xy_vel_ / xy_speed;
            x_vel *= scale;
            y_vel *= scale;
        }

        // Also clamp vertical velocity
        z_vel = std::clamp(z_vel, -max_z_vel_, max_z_vel_);

        // 8. Always rotate at yaw_vel_
        double z_ang = yaw_vel_;

        // 9. Publish command
        publishCmdVel(x_vel, y_vel, z_vel, z_ang);
    }

    // ================================  PUBLISHING ========================================
    void Controller::publishCmdVel(double x_vel, double y_vel, double z_vel, double yaw_vel)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = x_vel;
        cmd_vel.linear.y = y_vel;
        cmd_vel.linear.z = z_vel;
        cmd_vel.angular.z = yaw_vel;
        pub_cmd_vel_->publish(cmd_vel);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ee4308::drone::Controller>());
    rclcpp::shutdown();
    return 0;
}