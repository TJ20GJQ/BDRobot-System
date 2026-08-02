#ifndef DALI_BMS_CONTROL_HPP
#define DALI_BMS_CONTROL_HPP

#include <cstdio>
#include <memory>
#include <chrono>
#include <vector>
#include <deque>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "serial_driver/serial_driver.hpp"

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

#include "dali_bms_ros2/binary.h"

#include "dali_bms_ros2/msg/frame.hpp"

#include "robot_ros2_msgs/msg/dali_bms.hpp"

namespace dali_bms_control
{

using Frame = dali_bms_ros2::msg::Frame;

using DaliBms = robot_ros2_msgs::msg::DaliBms;

#define logs_debug(...)     do{if(param_.log){printf("\033[33m[BMS] [DEBUG] "); printf(__VA_ARGS__); printf("\033[0m\n"); fflush(stdout);}}while(0)
#define logs_info(...)      do{if(param_.log){printf(        "[BMS] [INFO] ");  printf(__VA_ARGS__); printf("\n");        fflush(stdout);}}while(0)
#define logs_error(...)     do{if(param_.log){printf("\033[31m[BMS] [ERROR] "); printf(__VA_ARGS__); printf("\033[0m\n"); fflush(stdout);}}while(0)

#define logs_debug_stream(args)    do{if(param_.log){std::cout << "\033[33m" << "[BMS]" << " " << "[DEBUG]" << " " << args << "\033[0m" << std::endl; std::cout.flush();}}while(0)
#define logs_info_stream(args)     do{if(param_.log){std::cout <<               "[BMS]" << " " << "[INFO]"  << " " << args              << std::endl; std::cout.flush();}}while(0)
#define logs_error_stream(args)    do{if(param_.log){std::cout << "\033[31m" << "[BMS]" << " " << "[ERROR]" << " " << args << "\033[0m" << std::endl; std::cout.flush();}}while(0)

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

enum class TransmitCmd
{
  soc_of_total_voltage_current = 0x90,
  maximum_minimum_voltage      = 0x91,
  maximum_minimum_temperature  = 0x92,
  charge_discharge_mos_status  = 0x93,
  status_information           = 0x94,
  cell_voltage                 = 0x95,
  cell_temperature             = 0x96,
  cell_balance_state           = 0x97,
  battery_failure_status       = 0x98,
};

enum class ReceiveCmd
{
  soc_of_total_voltage_current = 0x90,
  maximum_minimum_voltage      = 0x91,
  maximum_minimum_temperature  = 0x92,
  charge_discharge_mos_status  = 0x93,
  status_information           = 0x94,
  cell_voltage                 = 0x95,
  cell_temperature             = 0x96,
  cell_balance_state           = 0x97,
  battery_failure_status       = 0x98,
};

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
  std::string port;
  int baudrate = 0;

  bool log = true;
  bool original = false;

  double sub_interval;
  rclcpp::Time sub_last_time;
};

struct Pub
{
  rclcpp::Publisher<Frame>::SharedPtr frame;

  rclcpp::Publisher<DaliBms>::SharedPtr dali_bms;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state;
};

struct Sub
{
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr no_sub;
};

struct Tim
{
  rclcpp::TimerBase::SharedPtr serial_init;
  rclcpp::TimerBase::SharedPtr cell_timeout;
  rclcpp::TimerBase::SharedPtr pkg_loop;
  rclcpp::TimerBase::SharedPtr pkg_receive;
};

struct Msg
{
  Frame frame;

  DaliBms dali_bms;
  sensor_msgs::msg::BatteryState battery_state;
};

struct SerialStep
{
  bool create = false;
  bool close = false;
  bool open = false;

  void clean(void)
  {
    create = false;
    close = false;
    open = false;
  }

  bool finish(void)
  {
    return ((create && close && open) == true) ? true : false;
  }
};

struct SerialPort
{
  std::unique_ptr<drivers::common::IoContext> ctx{};
  std::unique_ptr<drivers::serial_driver::SerialDriver> driver;

  std::deque<uint8_t> rx;
  std::deque<std::vector<uint8_t>> tx;
};

struct Cell
{
  bool first = true;

  uint8_t cell_num = 0;
  uint8_t frame_max = 0;
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
  uint32_t cycle = 10;
  uint32_t step = 0;

  const size_t max_num = 9;

  /* get once */

  //

  /* upload */

  //

  /* loop get */

  PkgGet soc_of_total_voltage_current;
  PkgGet maximum_minimum_voltage;
  PkgGet maximum_minimum_temperature;
  PkgGet charge_discharge_mos_status;
  PkgGet status_information;
  PkgGet cell_voltage;
  PkgGet cell_temperature;
  PkgGet cell_balance_state;
  PkgGet battery_failure_status;

  /* not use */

  //

};

class DaliBmsControl : public rclcpp::Node
{

public:

  DaliBmsControl(void);
  ~DaliBmsControl(void);

private:

  Param param_;
  Pub pub_;
  Sub sub_;
  Tim tim_;
  Msg msg_;
  SerialStep step_;
  SerialPort port_;
  Cell cell_;
  Pkg pkg_;

  bool subIntervalCheck(void);

  void serialInit(void);
  void serialReset(void);
  bool serialCreate(void);
  bool serialOpen(void);
  bool serialClose(void);
  void serialRead(const std::vector<uint8_t> &array, const size_t &size);
  void serialWrite(std::vector<uint8_t> &array);

  void pkgReceive(void);
  void pkgProcess(void);

  void pkgFormatFrame(TransmitCmd cmd, std::vector<uint8_t> &tx_buffer);

  void pkgGetSocOfTotalVoltageCurrent(void);
  void pkgGetMaximumMinimumVoltage(void);
  void pkgGetMaximumMinimumTemperature(void);
  void pkgGetChargeDischargeMosStatus(void);
  void pkgGeStatusInformation(void);
  void pkgGetCellVoltage(void);
  void pkgGetCellTemperature(void);
  void pkgGetCellBalanceState(void);
  void pkgGetBatteryFailureStatus(void);

  void pkgGetParameter(void);
  void pkgGetSoftwareVersion(void);
  void pkgGetHardwareVersion(void);
  void pkgGetDate(void);

  void pkgLoopInit(void);
  void pkgLoopReset(void);
  void pkgGetLoop(void);
  PkgGetCode pkgGetCheck(PkgGet &pkg, uint32_t &step);

};

}

#endif