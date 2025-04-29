// src/balance_controller.cpp
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

using std::placeholders::_1;

class BalanceController : public rclcpp::Node {
public:
  BalanceController()
  : Node("balance_controller") {
    // PID gains 
    p_gain_ = 50.0;
    i_gain_ = 1.0;
    d_gain_ = 5.0;

    // Kalman filter initialization
    // state: [angle; bias]
    x_.fill(0.0);

    P_[0] = {1.0, 0.0};
    P_[1] = {0.0, 1.0};

    Q_angle_ = 0.001;
    Q_bias_ = 0.003;
    R_measure_ = 0.03;

    // Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&BalanceController::imu_callback, this, _1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&BalanceController::joint_callback, this, _1));

    // Publishers (effort commands)
    left_cmd_pub_ = create_publisher<std_msgs::msg::Float64>("/left_wheel_controller/command", 10);
    right_cmd_pub_ = create_publisher<std_msgs::msg::Float64>("/right_wheel_controller/command", 10);

    last_time_ = this->now();
    integral_ = 0.0;
    prev_error_ = 0.0;
  }

private:
  // Subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_cmd_pub_, right_cmd_pub_;

  // PID state
  double p_gain_, i_gain_, d_gain_;
  double integral_, prev_error_;

  // Kalman filter state
  std::array<double,2> x_;          // angle, bias
  std::array<std::array<double,2>,2> P_; // error covariance
  double Q_angle_, Q_bias_, R_measure_;

  rclcpp::Time last_time_;

  // Kalman filter update
  double kalman_update(double newRate, double newAngle, double dt) {
    // Predict
    x_[0] += dt * (newRate - x_[1]);
    P_[0][0] += dt * (dt*P_[1][1] - P_[0][1] - P_[1][0] + Q_angle_);
    P_[0][1] -= dt * P_[1][1];
    P_[1][0] -= dt * P_[1][1];
    P_[1][1] += Q_bias_ * dt;

    // Update
    double y = newAngle - x_[0];
    double S = P_[0][0] + R_measure_;
    double K0 = P_[0][0] / S;
    double K1 = P_[1][0] / S;

    x_[0] += K0 * y;
    x_[1] += K1 * y;

    double P00 = P_[0][0];
    double P01 = P_[0][1];
    double P10 = P_[1][0];
    double P11 = P_[1][1];

    P_[0][0] = P00 - K0 * P00;
    P_[0][1] = P01 - K0 * P01;
    P_[1][0] = P10 - K1 * P00;
    P_[1][1] = P11 - K1 * P01;

    return x_[0];
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // extract pitch angle and rate (assuming orientation quaternion -> Euler conversion done externally)
    double rate = msg->angular_velocity.y;
    // approximate angle from accel: pitch = atan2(ax, az)
    double angle = atan2(msg->linear_acceleration.x, msg->linear_acceleration.z);

    auto now = this->now();
    double dt = (now - last_time_).seconds();
   // RCLCPP_INFO(get_logger(), "dt = %.6f", dt);
    last_time_ = now;

    double filtered_angle = kalman_update(rate, angle, dt);

    // PID on angle
    double error = 0.0 - filtered_angle;
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    double control = p_gain_ * error + i_gain_ * integral_ + d_gain_ * derivative;
    prev_error_ = error;

    std_msgs::msg::Float64 cmd;
    cmd.data = control;

    // send same command to both wheels for balancing
    left_cmd_pub_->publish(cmd);
    right_cmd_pub_->publish(cmd);

    //RCLCPP_INFO(get_logger(), "Got IMU: angle=%.3f control=%.3f", filtered_angle, control);

  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // wheel position will be used later
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BalanceController>());
  rclcpp::shutdown();
  return 0;
}
