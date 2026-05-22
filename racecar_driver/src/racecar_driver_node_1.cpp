//
// racecar_driver_node.cpp
//

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "../include/racecar_driver.h"
#include <cstring>

class RacecarDriver : public rclcpp::Node
{
public:
RacecarDriver()
: Node("racecar_driver")
{
this->declare_parameter<std::string>("serial_port", "/dev/car");
this->declare_parameter<int>("baud_rate", 38400);
this->declare_parameter<double>("speed_coefficient", 10.0);

std::string serial_port;
int baud_rate;

this->get_parameter("serial_port", serial_port);
this->get_parameter("baud_rate", baud_rate);
this->get_parameter("speed_coefficient", speed_coefficient_);

// Convert std::string to char*
char serial_port_char[serial_port.size() + 1];
std::strcpy(serial_port_char, serial_port.c_str());

art_racecar_init(baud_rate, serial_port_char);

RCLCPP_INFO(this->get_logger(), "qidong");

subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
"/cmd_vel_nav", 1, std::bind(&RacecarDriver::TwistCallback, this, std::placeholders::_1));
}

private:
void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  // ─── 速度换算: m/s → PWM ───
  //   实测拟合: PWM = 1550 + speed(m/s) × 10
  //   实测数据: 2.0m/s→PWM1570  3.0→1580  4.0→1590  4.5→1600
  double speed_pwm;
  double speed = twist->linear.x;

  if (fabs(speed) < 0.01) {
    speed_pwm = 1500.0;                     // 停止
  } else if (speed > 0) {
    speed_pwm = 1550.0 + speed * 10.0;      // 前进
    if (speed_pwm < 1535.0)
      speed_pwm = 1535.0;                   // 越过死区
  } else {
    speed_pwm = 1500.0 + speed * 100.0;     // 后退（speed为负）
    if (speed_pwm > 1335.0)
      speed_pwm = 1335.0;                   // 越过死区
  }

  // ─── 转向换算: rad/s → 舵机角度(90居中) → 舵机PWM ───
  double angle_bias = -8.0;    // 机械左偏补偿
  double angle_indicator = 90.0 + twist->angular.z * 60.0 + angle_bias;
  double angle_pwm = 2500.0 - angle_indicator * 2000.0 / 180.0;

  // ─── PWM限幅 ───
  if (speed_pwm > 2500.0) speed_pwm = 2500.0;
  if (speed_pwm < 500.0)  speed_pwm = 500.0;
  if (angle_pwm > 2500.0) angle_pwm = 2500.0;
  if (angle_pwm < 500.0)  angle_pwm = 500.0;

  // ─── 日志输出 ───
  RCLCPP_INFO(this->get_logger(),
  "→ STM32: linear=%.2f(m/s) angular=%.2f(rad/s) | motor_pwm=%d servo_pwm=%d",
  twist->linear.x, twist->angular.z,
  static_cast<uint16_t>(speed_pwm), static_cast<uint16_t>(angle_pwm));

  // ─── 发送给STM32 ───
  send_cmd(static_cast<uint16_t>(speed_pwm),
           static_cast<uint16_t>(angle_pwm));
}

rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
double speed_coefficient_;
};

int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<RacecarDriver>());
rclcpp::shutdown();
return 0;
}
