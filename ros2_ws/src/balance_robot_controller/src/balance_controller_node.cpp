/**
 * balance_controller_node.cpp
 *
 * Cascade PID controller for Wheeled Inverted Pendulum.
 *
 * Architecture:
 *   IMU pitch (θ) ──► [Balance PID] ──► linear velocity v
 *   /cmd_vel      ──► (velocity offset + turn rate)
 *   combined ──► /cmd_vel_wheels ──► diff_drive plugin
 *
 * Subscriptions:
 *   /imu/data   [sensor_msgs/Imu]     - pitch angle & rate
 *   /cmd_vel    [geometry_msgs/Twist] - user velocity command
 *   /odom       [nav_msgs/Odometry]   - current velocity feedback
 *
 * Publication:
 *   /cmd_vel_wheels [geometry_msgs/Twist] - wheel velocity command
 */

#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "balance_robot_controller/pid.hpp"

static double quat_to_pitch(double qx, double qy, double qz, double qw)
{
  double sinp = 2.0 * (qw * qy - qz * qx);
  sinp = std::clamp(sinp, -1.0, 1.0);
  return std::asin(sinp);
}

class BalanceControllerNode : public rclcpp::Node
{
public:
  BalanceControllerNode()
  : Node("balance_controller")
  {
    // Parameters
    this->declare_parameter("balance_kp",    2.0);
    this->declare_parameter("balance_ki",    0.05);
    this->declare_parameter("balance_kd",    0.3);
    this->declare_parameter("vel_kp",        0.5);
    this->declare_parameter("vel_ki",        0.05);
    this->declare_parameter("vel_kd",        0.01);
    this->declare_parameter("theta_max",     0.35);
    this->declare_parameter("max_vel",       1.5);
    this->declare_parameter("vel_theta_max", 0.12);
    this->declare_parameter("wheel_radius",  0.05);

    double bkp = this->get_parameter("balance_kp").as_double();
    double bki = this->get_parameter("balance_ki").as_double();
    double bkd = this->get_parameter("balance_kd").as_double();
    double vkp = this->get_parameter("vel_kp").as_double();
    double vki = this->get_parameter("vel_ki").as_double();
    double vkd = this->get_parameter("vel_kd").as_double();

    theta_max_     = this->get_parameter("theta_max").as_double();
    max_vel_       = this->get_parameter("max_vel").as_double();
    vel_theta_max_ = this->get_parameter("vel_theta_max").as_double();

    pid_balance_.set_gains(bkp, bki, bkd);
    pid_vel_.set_gains(vkp, vki, vkd);

    // Subscribers
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        auto & q = msg->orientation;
        theta_       = quat_to_pitch(q.x, q.y, q.z, q.w);
        theta_dot_   = msg->angular_velocity.y;
        imu_received_ = true;
      });

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vx_ = msg->linear.x;
        cmd_wz_ = msg->angular.z;
      });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        current_vx_ = msg->twist.twist.linear.x;
      });

    // Publisher → diff_drive plugin
    wheel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_wheels", 10);

    // Control loop 200 Hz
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
      5ms, std::bind(&BalanceControllerNode::control_loop, this));

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "Balance controller started. Kp=%.2f Ki=%.3f Kd=%.2f",
      pid_balance_.kp(), pid_balance_.ki(), pid_balance_.kd());
  }

private:
  void control_loop()
  {
    if (!imu_received_) return;

    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0 || dt > 0.5) return;

    // Safety: cut motors if fallen
    if (std::abs(theta_) > theta_max_) {
      publish_cmd(0.0, 0.0);
      if (!fallen_) {
        RCLCPP_WARN(this->get_logger(),
          "Robot fallen! theta=%.3f rad. Stopping.", theta_);
        pid_balance_.reset();
        pid_vel_.reset();
        fallen_ = true;
      }
      return;
    }
    fallen_ = false;

    // Outer velocity loop: user cmd_vel + current velocity → tilt setpoint
    double vel_error      = cmd_vx_ - current_vx_;
    double theta_setpoint = pid_vel_.compute(vel_error, dt);
    theta_setpoint = std::clamp(theta_setpoint, -vel_theta_max_, vel_theta_max_);

    // Inner balance loop: tilt error → linear velocity
    double theta_error = theta_setpoint - theta_;
    double v_out       = pid_balance_.compute(theta_error, dt);
    v_out = std::clamp(v_out, -max_vel_, max_vel_);

    // Turn rate passes through from user cmd_vel
    double wz = cmd_wz_;

    publish_cmd(v_out, wz);

    if (log_counter_++ % 40 == 0) {
      RCLCPP_DEBUG(this->get_logger(),
        "θ=%.3f sp=%.3f v_out=%.3f vx=%.3f",
        theta_, theta_setpoint, v_out, current_vx_);
    }
  }

  void publish_cmd(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = linear_x;
    msg.angular.z = angular_z;
    wheel_pub_->publish(msg);
  }

  // Subs/Pubs
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   wheel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  PID pid_balance_, pid_vel_;

  double theta_      = 0.0;
  double theta_dot_  = 0.0;
  double current_vx_ = 0.0;
  double cmd_vx_     = 0.0;
  double cmd_wz_     = 0.0;

  bool imu_received_ = false;
  bool fallen_       = false;
  int  log_counter_  = 0;

  rclcpp::Time last_time_;

  double theta_max_     = 0.35;
  double max_vel_       = 1.5;
  double vel_theta_max_ = 0.12;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BalanceControllerNode>());
  rclcpp::shutdown();
  return 0;
}
