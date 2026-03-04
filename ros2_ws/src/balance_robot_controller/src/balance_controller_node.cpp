/**
 * balance_controller_node.cpp
 *
 * ROS2 node for Wheeled Inverted Pendulum balance control.
 *
 * Subscriptions:
 *   /imu/data          [sensor_msgs/Imu]       - IMU orientation & angular velocity
 *   /cmd_vel           [geometry_msgs/Twist]   - velocity command (vx, wz)
 *   /joint_states      [sensor_msgs/JointState]- wheel velocities
 *
 * Publications:
 *   /left_wheel_effort_controller/commands  [std_msgs/Float64MultiArray]
 *   /right_wheel_effort_controller/commands [std_msgs/Float64MultiArray]
 *
 * Control loops:
 *   1. Balance loop (high priority): keeps theta = 0
 *      u_balance = PID_balance(theta_error)
 *   2. Velocity loop (outer loop): adjusts theta setpoint to achieve cmd_vel
 *      theta_setpoint = PID_vel(vel_error)
 *
 * The two loops are combined: u_total = u_balance + u_vel
 */

#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "balance_robot_controller/pid.hpp"

// Quaternion → Euler (pitch only, assuming small roll)
static double quat_to_pitch(double qx, double qy, double qz, double qw)
{
  // Roll-Pitch-Yaw from quaternion (RPY = XYZ)
  // pitch = asin(2*(qw*qy - qz*qx))
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
    // --- Declare & get parameters ---
    this->declare_parameter("balance_kp", 80.0);
    this->declare_parameter("balance_ki",  2.0);
    this->declare_parameter("balance_kd", 10.0);

    this->declare_parameter("vel_kp", 0.5);
    this->declare_parameter("vel_ki", 0.05);
    this->declare_parameter("vel_kd", 0.01);

    this->declare_parameter("theta_max",      0.35);  // rad (~20 deg): safety cutoff
    this->declare_parameter("max_effort",    10.0);   // Nm
    this->declare_parameter("vel_theta_max",  0.15);  // rad: max tilt setpoint from vel loop
    this->declare_parameter("wheel_radius",   0.05);  // m

    load_params();

    // --- PID controllers ---
    pid_balance_.set_gains(
      this->get_parameter("balance_kp").as_double(),
      this->get_parameter("balance_ki").as_double(),
      this->get_parameter("balance_kd").as_double()
    );
    pid_vel_.set_gains(
      this->get_parameter("vel_kp").as_double(),
      this->get_parameter("vel_ki").as_double(),
      this->get_parameter("vel_kd").as_double()
    );

    // --- Subscribers ---
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&BalanceControllerNode::imu_callback, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&BalanceControllerNode::cmd_vel_callback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&BalanceControllerNode::joint_state_callback, this, std::placeholders::_1));

    // --- Publishers ---
    left_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/left_wheel_effort_controller/commands", 10);
    right_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/right_wheel_effort_controller/commands", 10);

    // --- Control timer: 500 Hz ---
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
      2ms, std::bind(&BalanceControllerNode::control_loop, this));

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "Balance controller started. Gains: Kp=%.1f Ki=%.2f Kd=%.1f",
      pid_balance_.kp(), pid_balance_.ki(), pid_balance_.kd());
  }

private:
  // ---------------------------------------------------------------
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto & q = msg->orientation;
    theta_     = quat_to_pitch(q.x, q.y, q.z, q.w);
    theta_dot_ = msg->angular_velocity.y;  // pitch rate
    imu_received_ = true;
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_vx_  = msg->linear.x;
    cmd_wz_  = msg->angular.z;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Compute average wheel linear velocity
    double v_left = 0.0, v_right = 0.0;
    bool found_left = false, found_right = false;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "left_wheel_joint") {
        v_left = msg->velocity[i] * wheel_radius_;
        found_left = true;
      }
      if (msg->name[i] == "right_wheel_joint") {
        v_right = msg->velocity[i] * wheel_radius_;
        found_right = true;
      }
    }
    if (found_left && found_right) {
      current_vx_ = (v_left + v_right) / 2.0;
    }
  }

  // ---------------------------------------------------------------
  void control_loop()
  {
    if (!imu_received_) return;

    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0 || dt > 0.5) return;  // skip bad dt

    // Safety: stop if robot has fallen over
    if (std::abs(theta_) > theta_max_) {
      publish_effort(0.0, 0.0);
      if (!fallen_) {
        RCLCPP_WARN(this->get_logger(),
          "Robot fallen! theta=%.3f rad. Motors disabled.", theta_);
        pid_balance_.reset();
        pid_vel_.reset();
        fallen_ = true;
      }
      return;
    }
    fallen_ = false;

    // --- Outer velocity loop: adjusts desired tilt angle ---
    double vel_error = cmd_vx_ - current_vx_;
    double theta_setpoint = pid_vel_.compute(vel_error, dt);
    theta_setpoint = std::clamp(theta_setpoint, -vel_theta_max_, vel_theta_max_);

    // --- Inner balance loop ---
    double theta_error = theta_setpoint - theta_;
    double u_balance   = pid_balance_.compute(theta_error, dt);

    // --- Turning: differential effort ---
    // Simple: add/subtract a fraction of cmd_wz to each wheel
    double u_turn = cmd_wz_ * 1.0;  // tuning factor

    double effort_left  = u_balance - u_turn;
    double effort_right = u_balance + u_turn;

    publish_effort(effort_left, effort_right);

    // Debug log at 10 Hz
    if (log_counter_++ % 50 == 0) {
      RCLCPP_DEBUG(this->get_logger(),
        "theta=%.4f sp=%.4f err=%.4f u=%.3f vx=%.3f",
        theta_, theta_setpoint, theta_error, u_balance, current_vx_);
    }
  }

  // ---------------------------------------------------------------
  void publish_effort(double left, double right)
  {
    std_msgs::msg::Float64MultiArray msg_l, msg_r;
    msg_l.data = {left};
    msg_r.data = {right};
    left_pub_->publish(msg_l);
    right_pub_->publish(msg_r);
  }

  void load_params()
  {
    theta_max_     = this->get_parameter("theta_max").as_double();
    vel_theta_max_ = this->get_parameter("vel_theta_max").as_double();
    wheel_radius_  = this->get_parameter("wheel_radius").as_double();
  }

  // ---------------------------------------------------------------
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Publications
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // PID controllers
  PID pid_balance_;
  PID pid_vel_;

  // State
  double theta_       = 0.0;   // pitch angle [rad]
  double theta_dot_   = 0.0;   // pitch rate  [rad/s]
  double current_vx_  = 0.0;   // measured linear velocity [m/s]
  double cmd_vx_      = 0.0;   // commanded linear velocity [m/s]
  double cmd_wz_      = 0.0;   // commanded angular velocity [rad/s]

  bool imu_received_  = false;
  bool fallen_        = false;
  int  log_counter_   = 0;

  rclcpp::Time last_time_;

  // Parameters
  double theta_max_     = 0.35;
  double vel_theta_max_ = 0.15;
  double wheel_radius_  = 0.05;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BalanceControllerNode>());
  rclcpp::shutdown();
  return 0;
}
