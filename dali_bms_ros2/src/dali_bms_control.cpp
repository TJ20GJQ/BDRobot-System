#include "dali_bms_ros2/dali_bms_control.hpp"

using namespace dali_bms_control;

using namespace std::chrono_literals;

DaliBmsControl::DaliBmsControl(void) : Node("dali_bms_ros2_node")
{
  declare_parameter("dali_bms_port", "/dev/ttyUSB0");
  declare_parameter("dali_bms_baudrate", 9600);
  declare_parameter("dali_bms_log_display", true);
  declare_parameter("dali_bms_original_display", false);

  param_.port     = get_parameter("dali_bms_port").as_string();
  param_.baudrate = get_parameter("dali_bms_baudrate").as_int();
  param_.log      = get_parameter("dali_bms_log_display").as_bool();
  param_.original = get_parameter("dali_bms_original_display").as_bool();

  msg_.dali_bms.no_of_battery_string = 0;
  msg_.dali_bms.no_of_temperature = 0;
  msg_.battery_state.charge = NAN;
  msg_.battery_state.capacity = NAN;
  msg_.battery_state.design_capacity = NAN;
  msg_.battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg_.battery_state.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

  param_.sub_interval = (1 / param_.baudrate) * 10.0f;
  param_.sub_last_time = rclcpp::Clock().now();

  auto qos = rclcpp::QoS(100).transient_local();

  (void)qos;

  pub_.frame = create_publisher<Frame>("/dali_bms/frame_info", 100);

  pub_.dali_bms = create_publisher<DaliBms>("/dali_bms/battery_info", 100);
  pub_.battery_state = create_publisher<sensor_msgs::msg::BatteryState>("/dali_bms/battery_state", 100);


  #if 0
  sub_.no_sub = create_subscription<std_msgs::msg::Empty>("/dali_bms/do_nothing", 100,
  [this](const std_msgs::msg::Empty::ConstSharedPtr)
  {
    if(subIntervalCheck() == false) {return;}
    // do nothing
  });
  #endif


  pkgLoopInit();


  tim_.serial_init = create_wall_timer(1s, [this](void)
  {
    serialInit();
  });
  tim_.pkg_loop = create_wall_timer(std::chrono::milliseconds(pkg_.cycle), [this](void)
  {
    pkgGetLoop();
  });
  tim_.pkg_receive = create_wall_timer(1ms, [this](void)
  {
    pkgReceive();
  });

}

DaliBmsControl::~DaliBmsControl(void)
{
  if(port_.ctx)
    port_.ctx->waitForExit();
}

bool DaliBmsControl::subIntervalCheck(void)
{
  if(((rclcpp::Clock().now() - param_.sub_last_time).seconds()) < param_.sub_interval)
  {
    logs_error("Sending failed, the sending interval is too short.");
    logs_error("The data sending cycle should be greater than %.3f seconds, or less than %.3f Hz.", param_.sub_interval, 1 / param_.sub_interval);

    return false;
  }
  else
  {
    param_.sub_last_time = rclcpp::Clock().now();

    return true;
  }

  return false;
}

void DaliBmsControl::serialInit(void)
{
  if(step_.finish() == true) {return;}

  step_.create = serialCreate();
  if(step_.create == false) {serialReset(); return;}

  step_.close = serialClose();
  if(step_.close == false) {serialReset(); return;}

  step_.open = serialOpen();
  if(step_.open == false) {serialReset(); return;}
}

void DaliBmsControl::serialReset(void)
{
  if(port_.ctx) {port_.ctx->waitForExit();}
  step_.clean();
  port_.rx.clear();
  for(auto &item : port_.tx) {item.clear();}
  port_.tx.clear();
  pkgLoopReset();
}

bool DaliBmsControl::serialCreate(void)
{
  try
  {
    logs_info("serial port try create.");

    drivers::serial_driver::SerialPortConfig config(
      static_cast<uint32_t>(param_.baudrate),
      drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE
    );

    port_.ctx = std::make_unique<drivers::common::IoContext>(1);
    port_.driver = std::make_unique<drivers::serial_driver::SerialDriver>(*port_.ctx);

    port_.driver->init_port(param_.port, config);
  }
  catch(const std::exception& e)
  {
    logs_error("serial port create fail.");

    logs_error_stream(e.what());

    return false;
  }

  logs_info("serial port create succeed.");

  return true;
}

bool DaliBmsControl::serialOpen(void)
{
  try
  {
    logs_info("serial port try open.");

    if(port_.driver->port()->is_open() == false)
      port_.driver->port()->open();

    port_.driver->port()->async_receive(std::bind(&dali_bms_control::DaliBmsControl::serialRead, this, std::placeholders::_1, std::placeholders::_2));

  }
  catch(const std::exception& e)
  {
    logs_error("serial port open fail.");

    logs_error_stream(e.what());

    return false;
  }

  logs_info("serial port open succeed.");

  return true;
}

bool DaliBmsControl::serialClose(void)
{
  try
  {
    logs_info("serial port try close.");

    port_.driver->port()->close();
  }
  catch(const std::exception& e)
  {
    logs_error("serial port close fail.");

    logs_error_stream(e.what());

    return false;
  }

  logs_info("serial port close succeed.");

  return true;
}

void DaliBmsControl::serialRead(const std::vector<uint8_t> &array, const size_t &size)
{
  if(step_.finish() != true) {return;}

  if(size == 0) {return;}

  for(size_t loop = 0; loop < size; loop++)
    port_.rx.push_back(array.at(loop));
}

void DaliBmsControl::serialWrite(std::vector<uint8_t> &array)
{
  if(step_.finish() != true) {return;}

  if(array.size() == 0) {return;}

  try
  {
    /* logs_info("serial port try write."); */

    port_.driver->port()->send(array);
  }
  catch(const std::exception& e)
  {
    logs_error("serial port write fail.");

    logs_error_stream(e.what());

    serialReset();

    return;
  }
}

void DaliBmsControl::pkgReceive(void)
{
  if(step_.finish() != true) {return;}

  std::deque<uint8_t> &rx_buffer = port_.rx;

  while(rx_buffer.size() >= 13)
  {
    if(rx_buffer.at(0) != 0xA5) {rx_buffer.pop_front(); continue;}

    if(rx_buffer.at(1) != 0x01) {rx_buffer.pop_front(); continue;}

    if(rx_buffer.at(3) != 0x08) {rx_buffer.pop_front(); continue;}

    uint8_t sum = 0;

    for(int32_t loop = 0; loop < 12; loop++) {sum = sum + rx_buffer.at(loop);}

    if(rx_buffer.at(12) != sum) {rx_buffer.pop_front(); continue;}

    if((param_.log == true) && (param_.original == true))
    {
      printf("rx frame: ");
      for(int32_t loop = 0; loop < 12; loop++)
      {
        printf("%02X ", rx_buffer.at(loop));
      }
      printf("\n");
    }

    pkgProcess();

    for(int32_t loop = 0; loop < 13; loop++)
    {
      rx_buffer.pop_front();
    }

    /* std::cout << "rx buffe size: " << rx_buffer.size() << std::endl; */
  }
}

void DaliBmsControl::pkgProcess(void)
{
  std::deque<uint8_t> &rx_buffer = port_.rx;

  switch((ReceiveCmd)rx_buffer.at(2))
  {
    case ReceiveCmd::soc_of_total_voltage_current:
    {
      for(int32_t loop = 0; loop < 13; loop++)
      {msg_.frame.cmd_90_soc_of_total_voltage_current.at(loop) = rx_buffer.at(loop);}
      pub_.frame->publish(msg_.frame);

      uint16_t cumulative_total_voltage = (uint16_t)((rx_buffer.at(5) << 0) | (rx_buffer.at(4) << 8));
      uint16_t gather_total_voltage     = (uint16_t)((rx_buffer.at(7) << 0) | (rx_buffer.at(6) << 8));
      uint16_t current                  = (uint16_t)((rx_buffer.at(9) << 0) | (rx_buffer.at(8) << 8));
      uint16_t soc                      = (uint16_t)((rx_buffer.at(11) << 0) | (rx_buffer.at(10) << 8));

      msg_.dali_bms.cumulative_total_voltage = ((float)cumulative_total_voltage) * 0.1f;
      msg_.dali_bms.gather_total_voltage     = ((float)gather_total_voltage) * 0.1f;
      msg_.dali_bms.current                  = (((float)current) * 0.1f) - 3000.0f;
      msg_.dali_bms.soc                      = ((float)soc) * 0.1f;

      msg_.battery_state.header.stamp = this->now();
      msg_.battery_state.voltage      = msg_.dali_bms.cumulative_total_voltage;
      msg_.battery_state.current      = msg_.dali_bms.current;
      msg_.battery_state.percentage   = msg_.dali_bms.soc * 0.01;

      pub_.dali_bms->publish(msg_.dali_bms);
      pub_.battery_state->publish(msg_.battery_state);

      pkg_.soc_of_total_voltage_current.setResponse();

      pkgInfoDisplay(soc_of_total_voltage_current, 2, "get soc of total voltage current success.");

      break;
    }
    case ReceiveCmd::maximum_minimum_voltage:
    {
      for(int32_t loop = 0; loop < 13; loop++)
      {msg_.frame.cmd_91_maximum_minimum_voltage.at(loop) = rx_buffer.at(loop);}
      pub_.frame->publish(msg_.frame);

      uint16_t maximum_cell_voltage_value     = (uint16_t)((rx_buffer.at(5) << 0) | (rx_buffer.at(4) << 8));
      uint8_t no_of_cell_with_maximum_voltage = (uint8_t)rx_buffer.at(6);
      uint16_t minimum_cell_voltage_value     = (uint16_t)((rx_buffer.at(8) << 0) | (rx_buffer.at(7) << 8));
      uint8_t no_of_cell_with_minimum_voltage = (uint8_t)rx_buffer.at(9);

      msg_.dali_bms.maximum_cell_voltage_value      = ((float)maximum_cell_voltage_value) * 0.001f;
      msg_.dali_bms.no_of_cell_with_maximum_voltage = no_of_cell_with_maximum_voltage;
      msg_.dali_bms.minimum_cell_voltage_value      = ((float)minimum_cell_voltage_value) * 0.001f;
      msg_.dali_bms.no_of_cell_with_minimum_voltage = no_of_cell_with_minimum_voltage;

      pub_.dali_bms->publish(msg_.dali_bms);

      pkg_.maximum_minimum_voltage.setResponse();

      pkgInfoDisplay(maximum_minimum_voltage, 2, "get maximum minimum voltage success.");

      break;
    }
    case ReceiveCmd::maximum_minimum_temperature:
    {
      for(int32_t loop = 0; loop < 13; loop++)
      {msg_.frame.cmd_92_maximum_minimum_temperature.at(loop) = rx_buffer.at(loop);}
      pub_.frame->publish(msg_.frame);

      uint8_t maximum_temperature_value   = (uint8_t)rx_buffer.at(4);
      uint8_t maximum_temperature_cell_no = (uint8_t)rx_buffer.at(5);
      uint8_t minimum_temperature_value   = (uint8_t)rx_buffer.at(6);
      uint8_t minimum_temperature_cell_no = (uint8_t)rx_buffer.at(7);

      msg_.dali_bms.maximum_temperature_value   = maximum_temperature_value - 40;
      msg_.dali_bms.maximum_temperature_cell_no = maximum_temperature_cell_no;
      msg_.dali_bms.minimum_temperature_value   = minimum_temperature_value - 40;
      msg_.dali_bms.minimum_temperature_cell_no = minimum_temperature_cell_no;

      msg_.battery_state.header.stamp = this->now();
      msg_.battery_state.temperature  = msg_.dali_bms.maximum_temperature_value;

      pub_.dali_bms->publish(msg_.dali_bms);
      pub_.battery_state->publish(msg_.battery_state);

      pkg_.maximum_minimum_temperature.setResponse();

      pkgInfoDisplay(maximum_minimum_temperature, 2, "get minimum temperature cell no success.");

      break;
    }
    case ReceiveCmd::charge_discharge_mos_status:
    {
      for(int32_t loop = 0; loop < 13; loop++)
      {msg_.frame.cmd_93_charge_discharge_mos_status.at(loop) = rx_buffer.at(loop);}
      pub_.frame->publish(msg_.frame);

      uint8_t state                = (uint8_t)rx_buffer.at(4);
      uint8_t charge_mos_state     = (uint8_t)rx_buffer.at(5);
      uint8_t discharge_mos_status = (uint8_t)rx_buffer.at(6);
      uint8_t bms_life             = (uint8_t)rx_buffer.at(7);
      uint32_t remain_capacity     = (uint32_t)((rx_buffer.at(11) << 0) | (rx_buffer.at(10) << 8) | (rx_buffer.at(9) << 16) | (rx_buffer.at(8) << 24));

      msg_.dali_bms.state                = state;
      msg_.dali_bms.charge_mos_state     = charge_mos_state;
      msg_.dali_bms.discharge_mos_status = discharge_mos_status;
      msg_.dali_bms.bms_life             = bms_life;
      msg_.dali_bms.remain_capacity      = ((float)remain_capacity) * 0.001f;

      msg_.battery_state.header.stamp = this->now();
      if(msg_.dali_bms.state == 0) {msg_.battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;}
      else if(msg_.dali_bms.state == 1) {msg_.battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;}
      else if(msg_.dali_bms.state == 2) {msg_.battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;}
      else {msg_.battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;}

      pub_.dali_bms->publish(msg_.dali_bms);
      pub_.battery_state->publish(msg_.battery_state);

      pkg_.charge_discharge_mos_status.setResponse();

      pkgInfoDisplay(charge_discharge_mos_status, 2, "get charge discharge mos status success.");

      break;
    }
    case ReceiveCmd::status_information:
    {
      for(int32_t loop = 0; loop < 13; loop++)
      {msg_.frame.cmd_94_status_information.at(loop) = rx_buffer.at(loop);}
      pub_.frame->publish(msg_.frame);

      uint8_t no_of_battery_string = (uint8_t)rx_buffer.at(4);
      uint8_t no_of_temperature    = (uint8_t)rx_buffer.at(5);
      uint8_t charger_status       = (uint8_t)rx_buffer.at(6);
      uint8_t load_status          = (uint8_t)rx_buffer.at(7);
      uint8_t di_do_state          = (uint8_t)rx_buffer.at(8);

      msg_.dali_bms.no_of_battery_string = no_of_battery_string;
      msg_.dali_bms.no_of_temperature    = no_of_temperature;
      msg_.dali_bms.charger_status       = charger_status;
      msg_.dali_bms.load_status          = load_status;

      for(int32_t loop = 0; loop < 4; loop++)
      {
        msg_.dali_bms.di_state.at(loop) = (di_do_state >> loop) & 0x01;
      }

      for(int32_t loop = 4; loop < 8; loop++)
      {
        msg_.dali_bms.do_state.at(loop - 4) = (di_do_state >> loop) & 0x01;
      }

      pub_.dali_bms->publish(msg_.dali_bms);

      pkg_.status_information.setResponse();

      pkgInfoDisplay(status_information, 2, "get status information success.");

      break;
    }
    case ReceiveCmd::cell_voltage:
    {
      for(int32_t loop = 0; loop < 13; loop++)
      {msg_.frame.cmd_95_cell_voltage.at(loop) = rx_buffer.at(loop);}
      pub_.frame->publish(msg_.frame);

      uint8_t frame_id        = (uint8_t)rx_buffer.at(4);
      uint16_t cell_0_voltage = (uint16_t)((rx_buffer.at(6) << 0) | (rx_buffer.at(5) << 8));
      uint16_t cell_1_voltage = (uint16_t)((rx_buffer.at(8) << 0) | (rx_buffer.at(7) << 8));
      uint16_t cell_2_voltage = (uint16_t)((rx_buffer.at(10) << 0) | (rx_buffer.at(9) << 8));

      if(frame_id == 1)
      {
        msg_.dali_bms.cell_voltage.clear();
        msg_.battery_state.cell_voltage.clear();
      }

      if(cell_0_voltage != 0xFFFF) {msg_.dali_bms.cell_voltage.push_back(((float)cell_0_voltage) * 0.001f);}
      if(cell_1_voltage != 0xFFFF) {msg_.dali_bms.cell_voltage.push_back(((float)cell_1_voltage) * 0.001f);}
      if(cell_2_voltage != 0xFFFF) {msg_.dali_bms.cell_voltage.push_back(((float)cell_2_voltage) * 0.001f);}

      if(cell_0_voltage != 0xFFFF) {msg_.battery_state.cell_voltage.push_back(((float)cell_0_voltage) * 0.001f);}
      if(cell_1_voltage != 0xFFFF) {msg_.battery_state.cell_voltage.push_back(((float)cell_1_voltage) * 0.001f);}
      if(cell_2_voltage != 0xFFFF) {msg_.battery_state.cell_voltage.push_back(((float)cell_2_voltage) * 0.001f);}

      if(cell_.first == true)
      {
        if(frame_id > cell_.frame_max) {cell_.frame_max = frame_id;}

        if(tim_.cell_timeout != nullptr)
          tim_.cell_timeout->cancel();

        tim_.cell_timeout = create_wall_timer(500ms, [this, cell_0_voltage, cell_1_voltage, cell_2_voltage](void)
        {
          tim_.cell_timeout->cancel();

          cell_.first = false;

          cell_.cell_num = msg_.dali_bms.cell_voltage.size();
          if(cell_0_voltage == 0xFFFF) {cell_.cell_num--;}
          if(cell_1_voltage == 0xFFFF) {cell_.cell_num--;}
          if(cell_2_voltage == 0xFFFF) {cell_.cell_num--;}

          pub_.dali_bms->publish(msg_.dali_bms);
          pub_.battery_state->publish(msg_.battery_state);

          pkg_.cell_voltage.setResponse();

          pkgInfoDisplay(cell_voltage, 2, "get cell voltage success.");
        });
      }
      else
      {
        if(frame_id == cell_.frame_max)
        {
          pub_.dali_bms->publish(msg_.dali_bms);
          pub_.battery_state->publish(msg_.battery_state);

          pkg_.cell_voltage.setResponse();

          pkgInfoDisplay(cell_voltage, 2, "get cell voltage success.");
        }
      }

      break;
    }
    case ReceiveCmd::cell_temperature:
    {
      for(int32_t loop = 0; loop < 13; loop++)
      {msg_.frame.cmd_96_cell_temperature.at(loop) = rx_buffer.at(loop);}
      pub_.frame->publish(msg_.frame);

      uint8_t frame_id          = (uint8_t)rx_buffer.at(4);
      int8_t cell_0_temperature = (int8_t)(rx_buffer.at(5) - 40);
      int8_t cell_1_temperature = (int8_t)(rx_buffer.at(6) - 40);
      int8_t cell_2_temperature = (int8_t)(rx_buffer.at(7) - 40);
      int8_t cell_3_temperature = (int8_t)(rx_buffer.at(8) - 40);
      int8_t cell_4_temperature = (int8_t)(rx_buffer.at(9) - 40);
      int8_t cell_5_temperature = (int8_t)(rx_buffer.at(10) - 40);
      int8_t cell_6_temperature = (int8_t)(rx_buffer.at(11) - 40);

      if(frame_id == 1)
      {
        msg_.dali_bms.cell_temperature.clear();
        msg_.battery_state.cell_temperature.clear();
      }

      if(msg_.dali_bms.cell_temperature.size() < msg_.dali_bms.no_of_temperature)
      {msg_.dali_bms.cell_temperature.push_back(cell_0_temperature); msg_.battery_state.cell_temperature.push_back(cell_0_temperature);}
      if(msg_.dali_bms.cell_temperature.size() < msg_.dali_bms.no_of_temperature)
      {msg_.dali_bms.cell_temperature.push_back(cell_1_temperature); msg_.battery_state.cell_temperature.push_back(cell_1_temperature);}
      if(msg_.dali_bms.cell_temperature.size() < msg_.dali_bms.no_of_temperature)
      {msg_.dali_bms.cell_temperature.push_back(cell_2_temperature); msg_.battery_state.cell_temperature.push_back(cell_2_temperature);}
      if(msg_.dali_bms.cell_temperature.size() < msg_.dali_bms.no_of_temperature)
      {msg_.dali_bms.cell_temperature.push_back(cell_3_temperature); msg_.battery_state.cell_temperature.push_back(cell_3_temperature);}
      if(msg_.dali_bms.cell_temperature.size() < msg_.dali_bms.no_of_temperature)
      {msg_.dali_bms.cell_temperature.push_back(cell_4_temperature); msg_.battery_state.cell_temperature.push_back(cell_4_temperature);}
      if(msg_.dali_bms.cell_temperature.size() < msg_.dali_bms.no_of_temperature)
      {msg_.dali_bms.cell_temperature.push_back(cell_5_temperature); msg_.battery_state.cell_temperature.push_back(cell_5_temperature);}
      if(msg_.dali_bms.cell_temperature.size() < msg_.dali_bms.no_of_temperature)
      {msg_.dali_bms.cell_temperature.push_back(cell_6_temperature); msg_.battery_state.cell_temperature.push_back(cell_6_temperature);}

      uint8_t frame_max = ((msg_.dali_bms.no_of_temperature % 7) == 0) ? (msg_.dali_bms.no_of_temperature / 7) : ((msg_.dali_bms.no_of_temperature / 7) + 1);

      if(frame_id == frame_max)
      {
        pub_.dali_bms->publish(msg_.dali_bms);
        pub_.battery_state->publish(msg_.battery_state);

        pkg_.cell_temperature.setResponse();

        pkgInfoDisplay(cell_temperature, 2, "get cell temperature success.");
      }

      break;
    }
    case ReceiveCmd::cell_balance_state:
    {
      for(int32_t loop = 0; loop < 13; loop++)
      {msg_.frame.cmd_97_cell_balance_state.at(loop) = rx_buffer.at(loop);}
      pub_.frame->publish(msg_.frame);

      for(size_t loop = 0; loop < 48; loop++)
      {
        msg_.dali_bms.cell_balance_state.at(loop) = (bool)((rx_buffer.at(4 + (loop / 8)) >> (loop % 8)) & 0x01);
      }

      pub_.dali_bms->publish(msg_.dali_bms);

      pkg_.cell_balance_state.setResponse();

      pkgInfoDisplay(cell_balance_state, 2, "get cell balance state success.");

      break;
    }
    case ReceiveCmd::battery_failure_status:
    {
      for(int32_t loop = 0; loop < 13; loop++)
      {msg_.frame.cmd_98_battery_failure_status.at(loop) = rx_buffer.at(loop);}
      pub_.frame->publish(msg_.frame);

      msg_.dali_bms.cell_volt_high_level_1 = (rx_buffer.at(4 + 0) >> 0) & 0x01;
      msg_.dali_bms.cell_volt_high_level_2 = (rx_buffer.at(4 + 0) >> 1) & 0x01;
      msg_.dali_bms.cell_volt_low_level_1  = (rx_buffer.at(4 + 0) >> 2) & 0x01;
      msg_.dali_bms.cell_volt_low_level_2  = (rx_buffer.at(4 + 0) >> 3) & 0x01;
      msg_.dali_bms.sum_volt_high_level_1  = (rx_buffer.at(4 + 0) >> 4) & 0x01;
      msg_.dali_bms.sum_volt_high_level_2  = (rx_buffer.at(4 + 0) >> 5) & 0x01;
      msg_.dali_bms.sum_volt_low_level_1   = (rx_buffer.at(4 + 0) >> 6) & 0x01;
      msg_.dali_bms.sum_volt_low_level_2   = (rx_buffer.at(4 + 0) >> 7) & 0x01;

      msg_.dali_bms.chg_temp_high_level_1    = (rx_buffer.at(4 + 1) >> 0) & 0x01;
      msg_.dali_bms.chg_temp_high_level_2    = (rx_buffer.at(4 + 1) >> 1) & 0x01;
      msg_.dali_bms.chg_temp_low_level_1     = (rx_buffer.at(4 + 1) >> 2) & 0x01;
      msg_.dali_bms.chg_temp_low_level_2     = (rx_buffer.at(4 + 1) >> 3) & 0x01;
      msg_.dali_bms.dischg_temp_high_level_1 = (rx_buffer.at(4 + 1) >> 4) & 0x01;
      msg_.dali_bms.dischg_temp_high_level_2 = (rx_buffer.at(4 + 1) >> 5) & 0x01;
      msg_.dali_bms.dischg_temp_low_level_1  = (rx_buffer.at(4 + 1) >> 6) & 0x01;
      msg_.dali_bms.dischg_temp_low_level_2  = (rx_buffer.at(4 + 1) >> 7) & 0x01;

      msg_.dali_bms.chg_overcurrent_level_1    = (rx_buffer.at(4 + 2) >> 0) & 0x01;
      msg_.dali_bms.chg_overcurrent_level_2    = (rx_buffer.at(4 + 2) >> 1) & 0x01;
      msg_.dali_bms.dischg_overcurrent_level_1 = (rx_buffer.at(4 + 2) >> 2) & 0x01;
      msg_.dali_bms.dischg_overcurrent_level_2 = (rx_buffer.at(4 + 2) >> 3) & 0x01;
      msg_.dali_bms.soc_high_level_1           = (rx_buffer.at(4 + 2) >> 4) & 0x01;
      msg_.dali_bms.soc_high_level_2           = (rx_buffer.at(4 + 2) >> 5) & 0x01;
      msg_.dali_bms.soc_low_level_1            = (rx_buffer.at(4 + 2) >> 6) & 0x01;
      msg_.dali_bms.soc_low_level_2            = (rx_buffer.at(4 + 2) >> 7) & 0x01;

      msg_.dali_bms.diff_volt_level_1 = (rx_buffer.at(4 + 3) >> 0) & 0x01;
      msg_.dali_bms.diff_volt_level_2 = (rx_buffer.at(4 + 3) >> 1) & 0x01;
      msg_.dali_bms.diff_temp_level_1 = (rx_buffer.at(4 + 3) >> 2) & 0x01;
      msg_.dali_bms.diff_temp_level_2 = (rx_buffer.at(4 + 3) >> 3) & 0x01;

      msg_.dali_bms.chg_mos_temp_high_alarm     = (rx_buffer.at(4 + 4) >> 0) & 0x01;
      msg_.dali_bms.dischg_mos_temp_high_alarm  = (rx_buffer.at(4 + 4) >> 1) & 0x01;
      msg_.dali_bms.chg_mos_temp_sensor_err     = (rx_buffer.at(4 + 4) >> 2) & 0x01;
      msg_.dali_bms.dischg_mos_temp_sensor_err  = (rx_buffer.at(4 + 4) >> 3) & 0x01;
      msg_.dali_bms.chg_mos_adhesion_err        = (rx_buffer.at(4 + 4) >> 4) & 0x01;
      msg_.dali_bms.dischg_mos_adhesion_err     = (rx_buffer.at(4 + 4) >> 5) & 0x01;
      msg_.dali_bms.chg_mos_open_circuit_err    = (rx_buffer.at(4 + 4) >> 6) & 0x01;
      msg_.dali_bms.dischg_mos_open_circuit_err = (rx_buffer.at(4 + 4) >> 7) & 0x01;

      msg_.dali_bms.afe_collect_chip_err           = (rx_buffer.at(4 + 5) >> 0) & 0x01;
      msg_.dali_bms.voltage_collect_dropped        = (rx_buffer.at(4 + 5) >> 1) & 0x01;
      msg_.dali_bms.cell_temp_sensor_err           = (rx_buffer.at(4 + 5) >> 2) & 0x01;
      msg_.dali_bms.eeprom_err                     = (rx_buffer.at(4 + 5) >> 3) & 0x01;
      msg_.dali_bms.rtc_err                        = (rx_buffer.at(4 + 5) >> 4) & 0x01;
      msg_.dali_bms.precharge_failure              = (rx_buffer.at(4 + 5) >> 5) & 0x01;
      msg_.dali_bms.communication_failure          = (rx_buffer.at(4 + 5) >> 6) & 0x01;
      msg_.dali_bms.internal_communication_failure = (rx_buffer.at(4 + 5) >> 7) & 0x01;

      msg_.dali_bms.current_module_fault         = (rx_buffer.at(4 + 6) >> 0) & 0x01;
      msg_.dali_bms.sum_voltage_detect_fault     = (rx_buffer.at(4 + 6) >> 1) & 0x01;
      msg_.dali_bms.short_circuit_protect_fault  = (rx_buffer.at(4 + 6) >> 2) & 0x01;
      msg_.dali_bms.low_volt_forbidden_chg_fault = (rx_buffer.at(4 + 6) >> 3) & 0x01;

      msg_.dali_bms.fault_code = rx_buffer.at(4 + 7);

      pub_.dali_bms->publish(msg_.dali_bms);

      pkg_.battery_failure_status.setResponse();

      pkgInfoDisplay(battery_failure_status, 2, "get battery failure status success.");

      break;
    }
    default:
    {
      break;
    }
  }

}

void DaliBmsControl::pkgFormatFrame(TransmitCmd cmd, std::vector<uint8_t> &tx_buffer)
{
  size_t loop = 0;
  uint8_t sum_value = 0;

  tx_buffer.clear();

  tx_buffer.push_back(0xA5);
  tx_buffer.push_back(0x40);
  tx_buffer.push_back((uint8_t)cmd);
  tx_buffer.push_back(0x08);

  tx_buffer.push_back(0x00);
  tx_buffer.push_back(0x00);
  tx_buffer.push_back(0x00);
  tx_buffer.push_back(0x00);
  tx_buffer.push_back(0x00);
  tx_buffer.push_back(0x00);
  tx_buffer.push_back(0x00);
  tx_buffer.push_back(0x00);

  for(loop = 0; loop < tx_buffer.size(); loop++)
  {
    sum_value = sum_value + tx_buffer.at(loop);
  }

  tx_buffer.push_back(sum_value);

  #if 0
  printf("tx frame: ");
  for(loop = 0; loop < tx_buffer.size(); loop++)
  {printf("%02X ", tx_buffer.at(loop));}
  printf("\n");
  #endif
}

void DaliBmsControl::pkgGetSocOfTotalVoltageCurrent(void)
{
  std::vector<uint8_t> tx_buffer;

  pkgFormatFrame(TransmitCmd::soc_of_total_voltage_current, tx_buffer);
  serialWrite(tx_buffer);
}

void DaliBmsControl::pkgGetMaximumMinimumVoltage(void)
{
  std::vector<uint8_t> tx_buffer;

  pkgFormatFrame(TransmitCmd::maximum_minimum_voltage, tx_buffer);
  serialWrite(tx_buffer);
}

void DaliBmsControl::pkgGetMaximumMinimumTemperature(void)
{
  std::vector<uint8_t> tx_buffer;

  pkgFormatFrame(TransmitCmd::maximum_minimum_temperature, tx_buffer);
  serialWrite(tx_buffer);
}

void DaliBmsControl::pkgGetChargeDischargeMosStatus(void)
{
  std::vector<uint8_t> tx_buffer;

  pkgFormatFrame(TransmitCmd::charge_discharge_mos_status, tx_buffer);
  serialWrite(tx_buffer);
}

void DaliBmsControl::pkgGeStatusInformation(void)
{
  std::vector<uint8_t> tx_buffer;

  pkgFormatFrame(TransmitCmd::status_information, tx_buffer);
  serialWrite(tx_buffer);
}

void DaliBmsControl::pkgGetCellVoltage(void)
{
  std::vector<uint8_t> tx_buffer;

  pkgFormatFrame(TransmitCmd::cell_voltage, tx_buffer);
  serialWrite(tx_buffer);
}

void DaliBmsControl::pkgGetCellTemperature(void)
{
  std::vector<uint8_t> tx_buffer;

  pkgFormatFrame(TransmitCmd::cell_temperature, tx_buffer);
  serialWrite(tx_buffer);
}

void DaliBmsControl::pkgGetCellBalanceState(void)
{
  std::vector<uint8_t> tx_buffer;

  pkgFormatFrame(TransmitCmd::cell_balance_state, tx_buffer);
  serialWrite(tx_buffer);
}

void DaliBmsControl::pkgGetBatteryFailureStatus(void)
{
  std::vector<uint8_t> tx_buffer;

  pkgFormatFrame(TransmitCmd::battery_failure_status, tx_buffer);
  serialWrite(tx_buffer);
}

void DaliBmsControl::pkgLoopInit(void)
{
  pkg_.step = 0;

  /* get once */

  //

  /* upload */

  //

  /* loop get */

  pkg_.soc_of_total_voltage_current.setOnce(false);
  pkg_.soc_of_total_voltage_current.setTimeout(200);
  pkg_.soc_of_total_voltage_current.flagClean();

  pkg_.maximum_minimum_voltage.setOnce(false);
  pkg_.maximum_minimum_voltage.setTimeout(200);
  pkg_.maximum_minimum_voltage.flagClean();

  pkg_.maximum_minimum_temperature.setOnce(false);
  pkg_.maximum_minimum_temperature.setTimeout(200);
  pkg_.maximum_minimum_temperature.flagClean();

  pkg_.charge_discharge_mos_status.setOnce(false);
  pkg_.charge_discharge_mos_status.setTimeout(200);
  pkg_.charge_discharge_mos_status.flagClean();

  pkg_.status_information.setOnce(false);
  pkg_.status_information.setTimeout(200);
  pkg_.status_information.flagClean();

  pkg_.cell_voltage.setOnce(false);
  pkg_.cell_voltage.setTimeout(2000);
  pkg_.cell_voltage.flagClean();

  pkg_.cell_temperature.setOnce(false);
  pkg_.cell_temperature.setTimeout(1000);
  pkg_.cell_temperature.flagClean();

  pkg_.cell_balance_state.setOnce(false);
  pkg_.cell_balance_state.setTimeout(200);
  pkg_.cell_balance_state.flagClean();

  pkg_.battery_failure_status.setOnce(false);
  pkg_.battery_failure_status.setTimeout(200);
  pkg_.battery_failure_status.flagClean();

  /* not use */

  //

}

void DaliBmsControl::pkgLoopReset(void)
{
  pkg_.step = 0;

  pkg_.soc_of_total_voltage_current.flagClean();
  pkg_.maximum_minimum_voltage.flagClean();
  pkg_.maximum_minimum_temperature.flagClean();
  pkg_.charge_discharge_mos_status.flagClean();
  pkg_.status_information.flagClean();
  pkg_.cell_voltage.flagClean();
  pkg_.cell_temperature.flagClean();
  pkg_.cell_balance_state.flagClean();
  pkg_.battery_failure_status.flagClean();

}

void DaliBmsControl::pkgGetLoop(void)
{
  if(step_.finish() != true) {return;}

  PkgGetCode code = PkgGetCode::wait;

  for(size_t step_loop = 0; step_loop < pkg_.max_num; step_loop++)
  {
    /* get once */

    //

    /* upload */

    //

    /* loop get */

    if(pkg_.step == 0)
    {
      code = pkgGetCheck(pkg_.soc_of_total_voltage_current, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetSocOfTotalVoltageCurrent(); break;}
      if(code == PkgGetCode::get)     { /* do sth */ break;}
      if(code == PkgGetCode::timeout) {logs_error("get soc of total voltage current timeout."); break;}
    }
    else if(pkg_.step == 1)
    {
      code = pkgGetCheck(pkg_.maximum_minimum_voltage, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetMaximumMinimumVoltage(); break;}
      if(code == PkgGetCode::get)     { /* do sth */ break;}
      if(code == PkgGetCode::timeout) {logs_error("get maximum minimum voltage timeout."); break;}
    }
    else if(pkg_.step == 2)
    {
      code = pkgGetCheck(pkg_.maximum_minimum_temperature, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetMaximumMinimumTemperature(); break;}
      if(code == PkgGetCode::get)     { /* do sth */ break;}
      if(code == PkgGetCode::timeout) {logs_error("get maximum minimum temperature timeout."); break;}
    }
    else if(pkg_.step == 3)
    {
      code = pkgGetCheck(pkg_.charge_discharge_mos_status, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetChargeDischargeMosStatus(); break;}
      if(code == PkgGetCode::get)     { /* do sth */ break;}
      if(code == PkgGetCode::timeout) {logs_error("get charge discharge mos status timeout."); break;}
    }
    else if(pkg_.step == 4)
    {
      code = pkgGetCheck(pkg_.status_information, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGeStatusInformation(); break;}
      if(code == PkgGetCode::get)     { /* do sth */ break;}
      if(code == PkgGetCode::timeout) {logs_error("get status information timeout."); break;}
    }
    else if(pkg_.step == 5)
    {
      code = pkgGetCheck(pkg_.cell_voltage, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetCellVoltage(); break;}
      if(code == PkgGetCode::get)     { /* do sth */ break;}
      if(code == PkgGetCode::timeout) {logs_error("get cell voltage timeout."); break;}
    }
    else if(pkg_.step == 6)
    {
      code = pkgGetCheck(pkg_.cell_temperature, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetCellTemperature(); break;}
      if(code == PkgGetCode::get)     { /* do sth */ break;}
      if(code == PkgGetCode::timeout) {logs_error("get cell temperature timeout."); break;}
    }
    else if(pkg_.step == 7)
    {
      code = pkgGetCheck(pkg_.cell_balance_state, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetCellBalanceState(); break;}
      if(code == PkgGetCode::get)     { /* do sth */ break;}
      if(code == PkgGetCode::timeout) {logs_error("get cell balance state timeout."); break;}
    }
    else if(pkg_.step == 8)
    {
      code = pkgGetCheck(pkg_.battery_failure_status, pkg_.step);
      if(code == PkgGetCode::send)    {pkgGetBatteryFailureStatus(); break;}
      if(code == PkgGetCode::get)     { /* do sth */ break;}
      if(code == PkgGetCode::timeout) {logs_error("get battery failure status timeout."); break;}
    }

    /* not use */

    //

    if(code == PkgGetCode::get) {break;}
    if(code == PkgGetCode::wait) {break;}
    if(code == PkgGetCode::skip) {continue;}
  }

  if(pkg_.step == pkg_.max_num)
  {
    pkg_.step = 0;
  }
}

PkgGetCode DaliBmsControl::pkgGetCheck(PkgGet &pkg, uint32_t &step)
{
  if((pkg.do_once == true) && (pkg.flag.once == true))
  {
    step++;
    return PkgGetCode::skip;
  }

  if(pkg.flag.request == false)
  {
    pkg.flag.request = true;
    pkg.flag.response = false;
    pkg.flag.timeout_cnt = 0;
    return PkgGetCode::send;
  }

  if(pkg.flag.request == true)
  {
    pkg.flag.timeout_cnt++;

    if(pkg.flag.response == true)
    {
      step++;
      pkg.flag.once = true;
      pkg.flag.request = false;
      pkg.flag.response = false;
      pkg.flag.timeout_cnt = 0;
      return PkgGetCode::get;
    }

    if(pkg.flag.timeout_cnt > ((pkg.timeout_ms < pkg_.cycle) ? 1 : (pkg.timeout_ms / pkg_.cycle)))
    {
      step++;
      pkg.flag.request = false;
      pkg.flag.response = false;
      pkg.flag.timeout_cnt = 0;
      return PkgGetCode::timeout;
    }

    if(pkg.flag.response == false)
    {
      return PkgGetCode::wait;
    }
  }

  return PkgGetCode::wait;
}
