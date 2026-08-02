#ifndef HT_CAN_CONTROL_HPP
#define HT_CAN_CONTROL_HPP

#include <cstdio>
#include <memory>
#include <chrono>
#include <vector>
#include <deque>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

#include "ht_can_ros2/binary.h"
#include "ht_can_ros2/controlcan.h"

#include "ht_can_ros2/msg/drive_sdfz_error.hpp"
#include "ht_can_ros2/msg/frame.hpp"

#include "robot_ros2_msgs/msg/auto_charge.hpp"
#include "robot_ros2_msgs/msg/battery_state.hpp"
#include "robot_ros2_msgs/msg/chassis_state.hpp"
#include "robot_ros2_msgs/msg/chassis_velocity.hpp"
#include "robot_ros2_msgs/msg/four_wheel_steer_angle.hpp"
#include "robot_ros2_msgs/msg/four_wheel_steer_current.hpp"
#include "robot_ros2_msgs/msg/four_wheel_steer_encoder.hpp"
#include "robot_ros2_msgs/msg/four_wheel_steer_motion.hpp"
#include "robot_ros2_msgs/msg/four_wheel_steer_speed.hpp"
#include "robot_ros2_msgs/msg/remote_control.hpp"

namespace ht_can_control
{

using DriveSdfzError = ht_can_ros2::msg::DriveSdfzError;
using Frame          = ht_can_ros2::msg::Frame;

using AutoCharge            = robot_ros2_msgs::msg::AutoCharge;
using BatteryState          = robot_ros2_msgs::msg::BatteryState;
using ChassisState          = robot_ros2_msgs::msg::ChassisState;
using FourWheelSteerAngle   = robot_ros2_msgs::msg::FourWheelSteerAngle;
using FourWheelSteerCurrent = robot_ros2_msgs::msg::FourWheelSteerCurrent;
using FourWheelSteerEncoder = robot_ros2_msgs::msg::FourWheelSteerEncoder;
using FourWheelSteerMotion  = robot_ros2_msgs::msg::FourWheelSteerMotion;
using FourWheelSteerSpeed   = robot_ros2_msgs::msg::FourWheelSteerSpeed;
using RemoteControl         = robot_ros2_msgs::msg::RemoteControl;

#if 0
#define logs_debug(...)     do{if(param_.log){printf("\033[33m[HT] [DEBUG] "); printf(__VA_ARGS__); printf("\033[0m\n"); fflush(stdout);}}while(0)
#define logs_info(...)      do{if(param_.log){printf(        "[HT] [INFO] ");  printf(__VA_ARGS__); printf("\n");        fflush(stdout);}}while(0)
#define logs_error(...)     do{if(param_.log){printf("\033[31m[HT] [ERROR] "); printf(__VA_ARGS__); printf("\033[0m\n"); fflush(stdout);}}while(0)

#define logs_debug_stream(args)    do{if(param_.log){std::cout << "\033[33m" << "[HT]" << " " << "[DEBUG]" << " " << args << "\033[0m" << std::endl; std::cout.flush();}}while(0)
#define logs_info_stream(args)     do{if(param_.log){std::cout <<               "[HT]" << " " << "[INFO]"  << " " << args              << std::endl; std::cout.flush();}}while(0)
#define logs_error_stream(args)    do{if(param_.log){std::cout << "\033[31m" << "[HT]" << " " << "[ERROR]" << " " << args << "\033[0m" << std::endl; std::cout.flush();}}while(0)
#endif

#if 1
#define logs_debug(...)     do{if(param_.log){RCLCPP_DEBUG(this->get_logger(), "[HT] " __VA_ARGS__);}}while(0)
#define logs_info(...)      do{if(param_.log){RCLCPP_INFO (this->get_logger(), "[HT] " __VA_ARGS__);}}while(0)
#define logs_error(...)     do{if(true      ){RCLCPP_ERROR(this->get_logger(), "[HT] " __VA_ARGS__);}}while(0)

#define logs_debug_stream(args)    do{if(param_.log){RCLCPP_DEBUG_STREAM(this->get_logger(), "[HT] " << args);}}while(0)
#define logs_info_stream(args)     do{if(param_.log){RCLCPP_INFO_STREAM (this->get_logger(), "[HT] " << args);}}while(0)
#define logs_error_stream(args)    do{if(true      ){RCLCPP_ERROR_STREAM(this->get_logger(), "[HT] " << args);}}while(0)
#endif

#define pkgInfoDisplay(name, max_cnt, str) \
do                                         \
{                                          \
  static uint8_t name##_cnt = 0;           \
                                           \
  name##_cnt++;                            \
                                           \
  if(name##_cnt == max_cnt)                \
  {                                        \
    name##_cnt = 0;                        \
    logs_info(str);                        \
  }                                        \
}                                          \
while(0)

#define CAN_DEVICE_TYPE       VCI_USBCAN2
#define CAN_DEVICE_INDEX      0
#define CAN_DEVICE_CHANNEL    0

#define angle_to_rad(angle)    ((float)(((angle) * 3.14159265f) / 180.0f))
#define rad_to_angle(rad)      ((float)(((rad) * 180.0f) / 3.14159265f))

enum class PkgGetCode
{
  send,
  get,
  wait,
  skip,
  timeout,
};

struct Param
{
  bool odom = false;
  bool log = true;
  bool original = false;
  bool skip_find = true;
};

struct Pub
{
  rclcpp::Publisher<Frame>::SharedPtr frame;

  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr drive_steer_front_left_error;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr drive_steer_front_right_error;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr drive_steer_rear_left_error;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr drive_steer_rear_right_error;

  rclcpp::Publisher<DriveSdfzError>::SharedPtr drive_front_left_error;
  rclcpp::Publisher<DriveSdfzError>::SharedPtr drive_front_right_error;
  rclcpp::Publisher<DriveSdfzError>::SharedPtr drive_rear_left_error;
  rclcpp::Publisher<DriveSdfzError>::SharedPtr drive_rear_right_error;

  rclcpp::Publisher<AutoCharge>::SharedPtr auto_charge;
  rclcpp::Publisher<BatteryState>::SharedPtr battery;
  rclcpp::Publisher<ChassisState>::SharedPtr state;
  rclcpp::Publisher<RemoteControl>::SharedPtr remote_ctrl;
  rclcpp::Publisher<FourWheelSteerAngle>::SharedPtr angle;
  rclcpp::Publisher<FourWheelSteerCurrent>::SharedPtr current;
  rclcpp::Publisher<FourWheelSteerEncoder>::SharedPtr encoder;
  rclcpp::Publisher<FourWheelSteerMotion>::SharedPtr motion;
  rclcpp::Publisher<FourWheelSteerSpeed>::SharedPtr speed;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
};

struct Sub
{
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ftfd;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ackermann;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rotate;

  rclcpp::Subscription<FourWheelSteerMotion>::SharedPtr motion;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr auto_charge_ctrl;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr collision_clean;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lamp_ctrl;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr odom_clean;
};

struct Tim
{
  rclcpp::TimerBase::SharedPtr can_init;
  rclcpp::TimerBase::SharedPtr pkg_loop;
  rclcpp::TimerBase::SharedPtr pkg_receive;
};

struct Msg
{
  Frame frame;

  std_msgs::msg::UInt16 drive_steer_front_left_error;
  std_msgs::msg::UInt16 drive_steer_front_right_error;
  std_msgs::msg::UInt16 drive_steer_rear_left_error;
  std_msgs::msg::UInt16 drive_steer_rear_right_error;

  DriveSdfzError drive_front_left_error;
  DriveSdfzError drive_front_right_error;
  DriveSdfzError drive_rear_left_error;
  DriveSdfzError drive_rear_right_error;

  AutoCharge auto_charge;
  BatteryState battery_state;
  ChassisState chassis_state;
  FourWheelSteerAngle four_wheel_steer_angle;
  FourWheelSteerCurrent four_wheel_steer_current;
  FourWheelSteerEncoder four_wheel_steer_encoder;
  FourWheelSteerMotion four_wheel_steer_motion;
  FourWheelSteerSpeed four_wheel_steer_speed;
  RemoteControl remote_control;
};

struct CanStep
{
  bool find = false;
  bool open = false;
  bool init = false;
  bool start = false;

  void clean(void)
  {
    find = false; open = false;
    init = false; start = false;
  }

  bool finish(void)
  {
    return ((find && open && init && start) == true) ? true : false;
  }
};

struct Odom
{
  double x = 0;
  double y = 0;
  double yaw = 0;
  rclcpp::Time last_time;
  nav_msgs::msg::Odometry value;

  void clean(void)
  {
    x = 0; y = 0; yaw = 0;
    value.pose.pose.position.x = x;
    value.pose.pose.position.y = y;
    last_time = rclcpp::Clock().now();
  }
};

struct PkgFlag
{
  bool once = false;

  bool request = false;
  bool response = false;

  uint32_t timeout_cnt = 0;
};

struct PkgGet
{
  bool do_once = false;
  uint32_t timeout_ms = 0;

  PkgFlag flag;

  void setOnce(bool set) {do_once = set;}
  void setTimeout(uint32_t ms) {timeout_ms = ms;}
  void flagClean(void) {flag = PkgFlag();}

  void setResponse(void) {if(flag.request == true) {flag.response = true;}}
};

struct Pkg
{
  uint32_t cycle = 100;
  uint32_t step = 0;

  const size_t max_num = 1;

  /* get once */

  PkgGet idle;

  /* upload */

  //

  /* loop get */

  //

  /* not use */

  //

};

class HtCanControl : public rclcpp::Node
{

public:

  HtCanControl(void);
  ~HtCanControl(void);

private:

  Param param_;
  Pub pub_;
  Sub sub_;
  Tim tim_;
  Msg msg_;
  CanStep step_;
  Odom odom_;
  Pkg pkg_;

  void canStep(void);
  bool canFindDevice(void);
  bool canOpenDevice(void);
  bool canCloseDevice(void);
  bool canInitDevice(void);
  bool canStartDevice(void);
  bool canResetDevice(void);

  void canTransmit(VCI_CAN_OBJ *tx_buff, uint32_t tx_len);
  void canReceive(void);
  void canProcess(uint32_t id, uint8_t (&data)[8], uint8_t data_len);
  void odomCalculation(double linear_x, double linear_y, double angular_z);

  void pkgSetVelocity(double linear_x, double linear_y, double angular_z);
  void pkgSetMotion(FourWheelSteerMotion motion);
  void pkgSetAutoCharge(uint8_t charge_type);
  void pkgCollisionClean(void);
  void pkgSetLamp(bool enable);

  void pkgLoopInit(void);
  void pkgLoopReset(void);
  void pkgGetLoop(void);
  PkgGetCode pkgGetCheck(PkgGet &pkg, uint32_t &step);

};

}

#endif