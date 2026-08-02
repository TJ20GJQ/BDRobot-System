#include "ht_can_ros2/ht_can_control.hpp"

#include <boost/array.hpp>

using namespace ht_can_control;

using namespace std::chrono_literals;

std::array<double, 36> cov_array = \
{1e-3, 0,    0,   0,   0,   0,
 0,    1e-3, 0,   0,   0,   0,
 0,    0,    1e6, 0,   0,   0,
 0,    0,    0,   1e6, 0,   0,
 0,    0,    0,   0,   1e6, 0,
 0,    0,    0,   0,   0,   1e3};

HtCanControl::HtCanControl(void) : Node("ht_can_ros2_node")
{
  declare_parameter("ht_odom_enable", true);
  declare_parameter("ht_log_display", true);
  declare_parameter("ht_original_display", false);
  declare_parameter("ht_skip_find", true);

  param_.odom      = get_parameter("ht_odom_enable").as_bool();
  param_.log       = get_parameter("ht_log_display").as_bool();
  param_.original  = get_parameter("ht_original_display").as_bool();
  param_.skip_find = get_parameter("ht_skip_find").as_bool();

  auto qos = rclcpp::QoS(100).transient_local();

  (void)qos;

  pub_.frame = create_publisher<Frame>("/ht/frame_info", 100);

  pub_.drive_steer_front_left_error  = create_publisher<std_msgs::msg::UInt16>("/ht/drive_steer_front_left_error_info", 100);
  pub_.drive_steer_front_right_error = create_publisher<std_msgs::msg::UInt16>("/ht/drive_steer_front_right_error_info", 100);
  pub_.drive_steer_rear_left_error   = create_publisher<std_msgs::msg::UInt16>("/ht/drive_steer_rear_left_error_info", 100);
  pub_.drive_steer_rear_right_error  = create_publisher<std_msgs::msg::UInt16>("/ht/drive_steer_rear_right_error_info", 100);

  pub_.drive_front_left_error  = create_publisher<DriveSdfzError>("/ht/drive_front_left_error_info", 100);
  pub_.drive_front_right_error = create_publisher<DriveSdfzError>("/ht/drive_front_right_error_info", 100);
  pub_.drive_rear_left_error   = create_publisher<DriveSdfzError>("/ht/drive_rear_left_error_info", 100);
  pub_.drive_rear_right_error  = create_publisher<DriveSdfzError>("/ht/drive_rear_right_error_info", 100);

  pub_.auto_charge = create_publisher<AutoCharge>             ("/ht/auto_charge_info", 100);
  pub_.battery     = create_publisher<BatteryState>           ("/ht/battery_info", 100);
  pub_.state       = create_publisher<ChassisState>           ("/ht/state_info", 100);
  pub_.remote_ctrl = create_publisher<RemoteControl>          ("/ht/remote_control_info", 100);
  pub_.angle       = create_publisher<FourWheelSteerAngle>    ("/ht/angle_info", 100);
  pub_.current     = create_publisher<FourWheelSteerCurrent>  ("/ht/current_info", 100);
  pub_.encoder     = create_publisher<FourWheelSteerEncoder>  ("/ht/encoder_count_info", 100);
  pub_.motion      = create_publisher<FourWheelSteerMotion>   ("/ht/motion_info", 100);
  pub_.speed       = create_publisher<FourWheelSteerSpeed>    ("/ht/speed_info", 100);
  pub_.odom        = create_publisher<nav_msgs::msg::Odometry>("/ht/odom_info", 100);


  sub_.velocity = create_subscription<geometry_msgs::msg::Twist>("/ht/velocity_ctrl", 100,
  [this](const geometry_msgs::msg::Twist::ConstSharedPtr twist)
  {
    pkgSetVelocity(twist->linear.x, twist->linear.y, twist->angular.z);
  });
  sub_.ftfd = create_subscription<geometry_msgs::msg::Twist>("/ht/ftfd_ctrl", 100,
  [this](const geometry_msgs::msg::Twist::ConstSharedPtr twist)
  {
    FourWheelSteerMotion motion;
    motion.motion_mode = 0x01;
    motion.linear_x = twist->linear.x;
    motion.linear_y = twist->linear.y;
    motion.angular_z = twist->angular.z;
    pkgSetMotion(motion);
  });
  sub_.ackermann = create_subscription<geometry_msgs::msg::Twist>("/ht/ackermann_ctrl", 100,
  [this](const geometry_msgs::msg::Twist::ConstSharedPtr twist)
  {
    FourWheelSteerMotion motion;
    motion.motion_mode = 0x00;
    motion.linear = twist->linear.x;
    motion.steering = twist->angular.z;
    motion.angular = 0;
    pkgSetMotion(motion);
  });
  sub_.rotate = create_subscription<geometry_msgs::msg::Twist>("/ht/rotate_ctrl", 100,
  [this](const geometry_msgs::msg::Twist::ConstSharedPtr twist)
  {
    FourWheelSteerMotion motion;
    motion.motion_mode = 0x00;
    motion.linear = 0;
    motion.steering = 0;
    motion.angular = twist->angular.z;
    pkgSetMotion(motion);
  });
  sub_.motion = create_subscription<FourWheelSteerMotion>("/ht/motion_ctrl", 100,
  [this](const FourWheelSteerMotion::ConstSharedPtr motion)
  {
    pkgSetMotion(*motion);
  });
  sub_.auto_charge_ctrl = create_subscription<std_msgs::msg::UInt8>("/ht/auto_charge_ctrl", 100,
  [this](const std_msgs::msg::UInt8::ConstSharedPtr auto_charge_type)
  {
    pkgSetAutoCharge(auto_charge_type->data);
  });
  sub_.collision_clean = create_subscription<std_msgs::msg::Empty>("/ht/collision_clean", 100,
  [this](const std_msgs::msg::Empty::ConstSharedPtr)
  {
    pkgCollisionClean();
  });
  sub_.lamp_ctrl = create_subscription<std_msgs::msg::Bool>("/ht/lamp_ctrl", 100,
  [this](const std_msgs::msg::Bool::ConstSharedPtr enable)
  {
    pkgSetLamp(enable->data);
  });
  sub_.odom_clean = create_subscription<std_msgs::msg::Empty>("/ht/odom_clean", 100,
  [this](const std_msgs::msg::Empty::ConstSharedPtr)
  {
    odom_.clean();
  });


  pkgLoopInit();


  tim_.can_init = create_wall_timer(1s, [this](void)
  {
    canStep();
  });
  tim_.pkg_loop = create_wall_timer(std::chrono::milliseconds(pkg_.cycle), [this](void)
  {
    pkgGetLoop();
  });
  tim_.pkg_receive = create_wall_timer(1ms, [this](void)
  {
    canReceive();
  });

}

HtCanControl::~HtCanControl(void)
{
  canCloseDevice();
}

void HtCanControl::canStep(void)
{
  if(step_.finish() == true) {return;}

  step_.find = canFindDevice();
  if(step_.find == false) {step_.clean(); return;}

  step_.open = canOpenDevice();
  if(step_.open == false) {step_.clean(); return;}

  step_.init = canInitDevice();
  if(step_.init == false) {step_.clean(); return;}

  step_.start = canStartDevice();
  if(step_.start == false) {step_.clean(); return;}
}

bool HtCanControl::canFindDevice(void)
{
  VCI_BOARD_INFO pInfo[50];
  uint32_t device_num = VCI_FindUsbDevice2(pInfo);    /* struct[] */
  logs_info("found %d CAN device.", device_num);

  if(device_num == 0)
  {
    if(param_.skip_find == true)
    {
      logs_info("CAN device not find.");
      logs_info("CAN device 0 will try open.");
      return true;
    }

    if(param_.skip_find == false)
    {
      logs_error("CAN device not find.");
      return false;
    }
  }

  if(device_num != 0)
  {
    logs_info("CAN device hardware type: %s", pInfo[0].str_hw_Type);
    logs_info("CAN device serial number: %s", pInfo[0].str_Serial_Num);
    logs_info("CAN device %s will be used.", pInfo[0].str_Serial_Num);
    return true;
  }

  return false;
}

bool HtCanControl::canOpenDevice(void)
{
  size_t open_flag = VCI_OpenDevice(CAN_DEVICE_TYPE, CAN_DEVICE_INDEX, 0);    /* DevType, DevIndex, Reserved */

  if(open_flag == UINT32_MAX)
  {
    logs_error("CAN device open fail. device lost.");
    canCloseDevice();
    step_.clean();
    return false;
  }

  logs_info("CAN device open success.");
  return true;
}

bool HtCanControl::canCloseDevice(void)
{
  size_t close_flag = VCI_CloseDevice(CAN_DEVICE_TYPE, CAN_DEVICE_INDEX);    /* DevType, DevIndex */

  if(close_flag == UINT32_MAX)
  {
    logs_error("CAN device close fail. device lost.");
    return false;
  }

  logs_info("CAN device close success.");
  return true;
}

bool HtCanControl::canInitDevice(void)
{
  VCI_INIT_CONFIG config;

  config.AccCode = 0x00000000;
	config.AccMask = 0xFFFFFFFF;
	config.Filter  = 1;
	config.Timing0 = 0x00;
	config.Timing1 = 0x1C;
	config.Mode    = 0;

  size_t init_flag = VCI_InitCAN(CAN_DEVICE_TYPE, CAN_DEVICE_INDEX, CAN_DEVICE_CHANNEL, &config);    /* DevType, DevIndex, CANIndex, struct */

  if(init_flag == UINT32_MAX)
  {
    logs_error("CAN device init fail. device lost.");
    canCloseDevice();
    step_.clean();
    return false;
  }

  logs_info("CAN device init success.");
  return true;
}

bool HtCanControl::canStartDevice(void)
{
  size_t start_flag = VCI_StartCAN(CAN_DEVICE_TYPE, CAN_DEVICE_INDEX, CAN_DEVICE_CHANNEL);    /* DevType, DevIndex, CANIndex */

  if(start_flag == UINT32_MAX)
  {
    logs_error("CAN device start fail. device lost.");
    canCloseDevice();
    step_.clean();
    return false;
  }

  logs_info("CAN device start success.");
  return true;
}

bool HtCanControl::canResetDevice(void)
{
  size_t reset_flag = VCI_ResetCAN(CAN_DEVICE_TYPE, CAN_DEVICE_INDEX, CAN_DEVICE_CHANNEL);    /* DevType, DevIndex, CANIndex */

  if(reset_flag == UINT32_MAX)
  {
    logs_error("CAN device reset fail. device lost.");
    return false;
  }

  logs_info("CAN device reset success.");
  return true;
}

void HtCanControl::canTransmit(VCI_CAN_OBJ *tx_buff, uint32_t tx_len)
{
  size_t tx_flag = VCI_Transmit(CAN_DEVICE_TYPE, CAN_DEVICE_INDEX, CAN_DEVICE_CHANNEL, tx_buff, tx_len);    /* DevType, DevIndex, CANIndex, pSend, Length */

  if(tx_flag == UINT32_MAX)
  {
    logs_error("CAN device transmi fail. device lost.");
    canCloseDevice();
    step_.clean();
    return;
  }

  //logs_info("CAN transmi %d frame.", tx_flag);

  if((param_.log == true) && (param_.original == true))
  {
    VCI_CAN_OBJ *tx_addr = tx_buff;

    for(size_t loop = 0; loop < tx_len; loop++)
    {
      logs_info("transmi frame id: %03X    frame data: %02X %02X %02X %02X %02X %02X %02X %02X", tx_addr->ID, \
                                          tx_addr->Data[0], tx_addr->Data[1], tx_addr->Data[2], tx_addr->Data[3], \
                                          tx_addr->Data[4], tx_addr->Data[5], tx_addr->Data[6], tx_addr->Data[7]);

      tx_addr++;
    }
  }
}

void HtCanControl::canReceive(void)
{
  if(step_.finish() != true) {return;}

  static VCI_CAN_OBJ rx_buff[3000];

  size_t rx_cnt = VCI_Receive(CAN_DEVICE_TYPE, CAN_DEVICE_INDEX, CAN_DEVICE_CHANNEL, rx_buff, 3000, 100);    /* DevType, DevIndex, CANIndex, pReceive, Len, WaitTime */

  if(rx_cnt == UINT32_MAX)
  {
    logs_error("CAN device receive fail. device lost.");
    canCloseDevice();
    step_.clean();
    return;
  }

  //logs_info("CAN receive %d frame.", rx_cnt);

  for(size_t loop = 0; loop < rx_cnt; loop++)
  {
    if(rx_buff[loop].ExternFlag != 0x00) {return;}
    if(rx_buff[loop].RemoteFlag != 0x00) {return;}

    if((param_.log == true) && (param_.original == true))
    {
      logs_info("receive frame id: %03X    frame data: %02X %02X %02X %02X %02X %02X %02X %02X", \
                                            rx_buff[loop].ID, \
                                            rx_buff[loop].Data[0], rx_buff[loop].Data[1], \
                                            rx_buff[loop].Data[2], rx_buff[loop].Data[3], \
                                            rx_buff[loop].Data[4], rx_buff[loop].Data[5], \
                                            rx_buff[loop].Data[6], rx_buff[loop].Data[7]);
    }

    canProcess(rx_buff[loop].ID, rx_buff[loop].Data, rx_buff[loop].DataLen);
  }
}

void HtCanControl::canProcess(uint32_t id, uint8_t (&data)[8], uint8_t data_len)
{
  (void)data_len;

  switch(id)
  {
    case 0x10:
    {
      msg_.frame.id_10_motion_state.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_10_motion_state.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      msg_.four_wheel_steer_motion.motion_mode = data[1];

      if(data[1] == 0)  /* akm */
      {
        msg_.four_wheel_steer_motion.linear_x  = 0;
        msg_.four_wheel_steer_motion.linear_y  = 0;
        msg_.four_wheel_steer_motion.angular_z = 0;

        msg_.four_wheel_steer_motion.linear   = (((int16_t)((data[3] << 8) | (data[2] << 0))) * 0.001f);
        msg_.four_wheel_steer_motion.steering = angle_to_rad(((int16_t)((data[5] << 8) | (data[4] << 0))) * 0.01f);
        msg_.four_wheel_steer_motion.angular  = (((int16_t)((data[7] << 8) | (data[6] << 0))) * 0.001f);
      }

      if(data[1] == 1)  /* ftfd */
      {
        msg_.four_wheel_steer_motion.linear_x  = (((int16_t)((data[3] << 8) | (data[2] << 0))) * 0.001f);
        msg_.four_wheel_steer_motion.linear_y  = (((int16_t)((data[5] << 8) | (data[4] << 0))) * 0.001f);
        msg_.four_wheel_steer_motion.angular_z = (((int16_t)((data[7] << 8) | (data[6] << 0))) * 0.001f);

        msg_.four_wheel_steer_motion.linear   = 0;
        msg_.four_wheel_steer_motion.steering = 0;
        msg_.four_wheel_steer_motion.angular  = 0;
      }

      if(data[1] == 2)  /* double akm */
      {
        msg_.four_wheel_steer_motion.linear_x  = 0;
        msg_.four_wheel_steer_motion.linear_y  = 0;
        msg_.four_wheel_steer_motion.angular_z = 0;

        msg_.four_wheel_steer_motion.linear   = (((int16_t)((data[3] << 8) | (data[2] << 0))) * 0.001f);
        msg_.four_wheel_steer_motion.steering = angle_to_rad(((int16_t)((data[5] << 8) | (data[4] << 0))) * 0.01f);
        msg_.four_wheel_steer_motion.angular  = (((int16_t)((data[7] << 8) | (data[6] << 0))) * 0.001f);
      }

      if(msg_.four_wheel_steer_motion.motion_mode == 0)
      {
        odomCalculation(0, 0, msg_.four_wheel_steer_motion.angular);
      }

      if(msg_.four_wheel_steer_motion.motion_mode == 1)
      {
        odomCalculation(msg_.four_wheel_steer_motion.linear_x, msg_.four_wheel_steer_motion.linear_y, msg_.four_wheel_steer_motion.angular_z);
      }

      if(msg_.four_wheel_steer_motion.motion_mode == 2)
      {
        odomCalculation(0, 0, msg_.four_wheel_steer_motion.angular);
      }

      pub_.motion->publish(msg_.four_wheel_steer_motion);

      pkgInfoDisplay(motion, 50, "get motion state package.");

      break;
    }
    case 0x11:
    {
      msg_.frame.id_11_system_state_1.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_11_system_state_1.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      msg_.battery_state.voltage = ((uint16_t)((data[1] << 8) | (data[0] << 0))) * 0.01f;

      msg_.chassis_state.control_mode = data[2];

      msg_.chassis_state.remote_control_online = (data[3] >> 0) & 0x01;
      msg_.chassis_state.stop_button           = (data[3] >> 1) & 0x01;
      msg_.chassis_state.remote_control_stop   = (data[3] >> 2) & 0x01;
      msg_.chassis_state.software_stop         = (data[3] >> 3) & 0x01;

      msg_.chassis_state.motor_drive_error   = !!data[4];
      msg_.chassis_state.motor_encoder_error = data[5] || (data[6] & B_0000_1111);

      msg_.battery_state.percent = data[7];

      pub_.state->publish(msg_.chassis_state);
      pub_.battery->publish(msg_.battery_state);

      pkgInfoDisplay(state_1, 50, "get system state 1 package.");

      break;
    }
    case 0x12:
    {
      msg_.frame.id_12_drive_motor.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_12_drive_motor.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      int16_t speed = (int16_t)((data[3] << 8) | (data[2] << 0));
      uint32_t encoder = (uint32_t)((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | (data[4] << 0));

      switch(data[1])
      {
        case 0x01:
        {
          msg_.four_wheel_steer_speed.front_left = speed;
          msg_.four_wheel_steer_encoder.front_left = encoder;
          break;
        }
        case 0x02:
        {
          msg_.four_wheel_steer_speed.front_right = speed;
          msg_.four_wheel_steer_encoder.front_right = encoder;
          break;
        }
        case 0x03:
        {
          msg_.four_wheel_steer_speed.rear_left = speed;
          msg_.four_wheel_steer_encoder.rear_left = encoder;
          break;
        }
        case 0x04:
        {
          msg_.four_wheel_steer_speed.rear_right = speed;
          msg_.four_wheel_steer_encoder.rear_right = encoder;
          break;
        }
      }

      pub_.speed->publish(msg_.four_wheel_steer_speed);
      pub_.encoder->publish(msg_.four_wheel_steer_encoder);

      pkgInfoDisplay(drive_motor, 200, "get drive motor package.");

      break;
    }
    case 0x13:
    {
      msg_.frame.id_13_steer_motor.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_13_steer_motor.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      int16_t angle = (int16_t)((data[3] << 8) | (data[2] << 0));

      switch(data[1])
      {
        case 0x01:
        {
          msg_.four_wheel_steer_angle.front_left = angle_to_rad(angle * 0.01f);
          break;
        }
        case 0x02:
        {
          msg_.four_wheel_steer_angle.front_right = angle_to_rad(angle * 0.01f);
          break;
        }
        case 0x03:
        {
          msg_.four_wheel_steer_angle.rear_left = angle_to_rad(angle * 0.01f);
          break;
        }
        case 0x04:
        {
          msg_.four_wheel_steer_angle.rear_right = angle_to_rad(angle * 0.01f);
          break;
        }
      }

      pub_.angle->publish(msg_.four_wheel_steer_angle);

      pkgInfoDisplay(steer_motor, 200, "get steer motor package.");

      break;
    }
    case 0x14:
    {
      msg_.frame.id_14_auto_charge.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_14_auto_charge.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      msg_.auto_charge.online       = !data[0];
      msg_.auto_charge.state        = data[1];
      msg_.auto_charge.limit_switch = (data[2] >> 0) & 0x01;
      msg_.auto_charge.relays       = (data[2] >> 1) & 0x01;
      msg_.auto_charge.voltage      = (data[2] >> 2) & 0x01;
      msg_.auto_charge.infrared     = data[3];

      pub_.auto_charge->publish(msg_.auto_charge);

      pkgInfoDisplay(charge, 10, "get charge package.");

      break;
    }
    case 0x15:
    {
      msg_.frame.id_15_rc_rocker.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_15_rc_rocker.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      msg_.remote_control.rocker_right_x = (int16_t)((data[1] << 8) | (data[0] << 0));
      msg_.remote_control.rocker_right_y = (int16_t)((data[3] << 8) | (data[2] << 0));
      msg_.remote_control.rocker_left_y  = (int16_t)((data[5] << 8) | (data[4] << 0));
      msg_.remote_control.rocker_left_x  = (int16_t)((data[7] << 8) | (data[6] << 0));

      pub_.remote_ctrl->publish(msg_.remote_control);

      pkgInfoDisplay(remote_ctrl_rocker, 20, "get remote ctrl rocker package.");

      break;
    }
    case 0x16:
    {
      msg_.frame.id_16_rc_key.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_16_rc_key.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      msg_.remote_control.round_left_vra  = (int16_t)((data[1] << 8) | (data[0] << 0));
      msg_.remote_control.round_right_vrb = (int16_t)((data[3] << 8) | (data[2] << 0));

      msg_.remote_control.key_swa = (data[4] >> (0 * 2)) & B_0000_0011;
      msg_.remote_control.key_swb = (data[4] >> (1 * 2)) & B_0000_0011;
      msg_.remote_control.key_swc = (data[4] >> (2 * 2)) & B_0000_0011;
      msg_.remote_control.key_swd = (data[4] >> (3 * 2)) & B_0000_0011;

      msg_.remote_control.online = !((data[5] >> 0) & 0x01);

      pub_.remote_ctrl->publish(msg_.remote_control);

      pkgInfoDisplay(remote_ctrl_key, 20, "get remote ctrl key package.");

      break;
    }
    case 0x30:
    {
      msg_.frame.id_30_system_state_2.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_30_system_state_2.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      msg_.chassis_state.front_collision = (((uint16_t)((data[1] << 8) | (data[0] << 0))) >> 0) & 0x0001;
      msg_.chassis_state.rear_collision  = (((uint16_t)((data[1] << 8) | (data[0] << 0))) >> 1) & 0x0001;
      msg_.chassis_state.lamp_enable     = (((uint16_t)((data[1] << 8) | (data[0] << 0))) >> 2) & 0x0001;

      msg_.chassis_state.motor_drive_online = !(data[2] & B_0000_1111);

      pub_.state->publish(msg_.chassis_state);

      pkgInfoDisplay(state_2, 10, "get system state 2 package.");

      break;
    }
    case 0x40:
    {
      msg_.frame.id_40_steer_current.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_40_steer_current.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      msg_.four_wheel_steer_current.front_steering_left  = ((int16_t)((data[1] << 8) | (data[0] << 0))) * 0.1f;
      msg_.four_wheel_steer_current.front_steering_right = ((int16_t)((data[3] << 8) | (data[2] << 0))) * 0.1f;
      msg_.four_wheel_steer_current.rear_steering_left   = ((int16_t)((data[5] << 8) | (data[4] << 0))) * 0.1f;
      msg_.four_wheel_steer_current.rear_steering_right  = ((int16_t)((data[7] << 8) | (data[6] << 0))) * 0.1f;

      pub_.current->publish(msg_.four_wheel_steer_current);

      pkgInfoDisplay(steer_current, 50, "get steer current package.");

      break;
    }
    case 0x41:
    {
      msg_.frame.id_41_drive_current.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_41_drive_current.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      msg_.four_wheel_steer_current.front_left  = ((int16_t)((data[1] << 8) | (data[0] << 0))) * 0.1f;
      msg_.four_wheel_steer_current.front_right = ((int16_t)((data[3] << 8) | (data[2] << 0))) * 0.1f;
      msg_.four_wheel_steer_current.rear_left   = ((int16_t)((data[5] << 8) | (data[4] << 0))) * 0.1f;
      msg_.four_wheel_steer_current.rear_right  = ((int16_t)((data[7] << 8) | (data[6] << 0))) * 0.1f;

      pub_.current->publish(msg_.four_wheel_steer_current);

      pkgInfoDisplay(drive_current, 50, "get drive current package.");

      break;
    }
    case 0x42:
    {
      msg_.frame.id_42_steer_error.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_42_steer_error.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      msg_.drive_steer_front_left_error.data  = ((uint16_t)((data[1] << 8) | (data[0] << 0)));
      msg_.drive_steer_front_right_error.data = ((uint16_t)((data[3] << 8) | (data[2] << 0)));
      msg_.drive_steer_rear_left_error.data   = ((uint16_t)((data[5] << 8) | (data[4] << 0)));
      msg_.drive_steer_rear_right_error.data  = ((uint16_t)((data[7] << 8) | (data[6] << 0)));

      pub_.drive_steer_front_left_error->publish(msg_.drive_steer_front_left_error);
      pub_.drive_steer_front_right_error->publish(msg_.drive_steer_front_right_error);
      pub_.drive_steer_rear_left_error->publish(msg_.drive_steer_rear_left_error);
      pub_.drive_steer_rear_right_error->publish(msg_.drive_steer_rear_right_error);

      pkgInfoDisplay(steer_error, 10, "get steer error package.");

      break;
    }
    case 0x43:
    {
      msg_.frame.id_43_drive_error.clear();
      for(size_t loop = 0; loop < 8; loop++)
      {msg_.frame.id_43_drive_error.push_back(data[loop]);}
      pub_.frame->publish(msg_.frame);

      for(size_t loop = 0; loop < 4; loop++)
      {
        uint16_t drive_error = (uint16_t)((data[1 + (loop * 2)] << 8) | (data[0 + (loop * 2)] << 0));
        DriveSdfzError *sdfz_error = nullptr;

        if(loop == 0) {sdfz_error = &msg_.drive_front_left_error;}
        if(loop == 1) {sdfz_error = &msg_.drive_front_right_error;}
        if(loop == 2) {sdfz_error = &msg_.drive_rear_left_error;}
        if(loop == 3) {sdfz_error = &msg_.drive_rear_right_error;}

        *sdfz_error = DriveSdfzError();

        sdfz_error->internal               = (drive_error >> 0) & 0x01;
        sdfz_error->encoder_abz            = (drive_error >> 1) & 0x01;
        sdfz_error->encoder_uvw            = (drive_error >> 2) & 0x01;
        sdfz_error->encoder_count          = (drive_error >> 3) & 0x01;
        sdfz_error->drive_overtemp         = (drive_error >> 4) & 0x01;
        sdfz_error->drive_bus_overvoltage  = (drive_error >> 5) & 0x01;
        sdfz_error->drive_bus_undervoltage = (drive_error >> 6) & 0x01;
        sdfz_error->drive_short_circuit    = (drive_error >> 7) & 0x01;
        sdfz_error->brake_overtemp         = (drive_error >> 8) & 0x01;
        sdfz_error->actual_following       = (drive_error >> 9) & 0x01;
        sdfz_error->reserved               = (drive_error >> 10) & 0x01;
        sdfz_error->i2t                    = (drive_error >> 11) & 0x01;
        sdfz_error->speed_following        = (drive_error >> 12) & 0x01;
        sdfz_error->motor_overtemp         = (drive_error >> 13) & 0x01;
        sdfz_error->encoder_comm           = (drive_error >> 14) & 0x01;
        sdfz_error->comm_loss              = (drive_error >> 15) & 0x01;
      }

      pub_.drive_front_left_error->publish(msg_.drive_front_left_error);
      pub_.drive_front_right_error->publish(msg_.drive_front_right_error);
      pub_.drive_rear_left_error->publish(msg_.drive_rear_left_error);
      pub_.drive_rear_right_error->publish(msg_.drive_rear_right_error);

      pkgInfoDisplay(drive_error, 10, "get drive error package.");

      break;
    }
    default:
    {
      break;
    }
  }
}

void HtCanControl::odomCalculation(double linear_x, double linear_y, double angular_z)
{
  if(param_.odom == false) {return;}

  odom_.value.header.stamp = rclcpp::Clock().now();
  odom_.value.header.frame_id = "odom";
  odom_.value.child_frame_id = "base_link";

  double vx = linear_x;
  double vy = linear_y;
  double vyaw = angular_z;
  double dt = (rclcpp::Clock().now() - odom_.last_time).seconds();
  odom_.last_time = rclcpp::Clock().now();

  double dx = (vx * cos(odom_.yaw) - vy * sin(odom_.yaw)) * dt;
  double dy = (vx * sin(odom_.yaw) + vy * cos(odom_.yaw)) * dt;
  double dyaw = vyaw * dt;

  odom_.x += dx;
  odom_.y += dy;
  odom_.yaw += dyaw;

  tf2::Quaternion q;
  q.setRPY(0, 0, odom_.yaw);

  odom_.value.pose.pose.position.x = odom_.x;
  odom_.value.pose.pose.position.y = odom_.y;
  odom_.value.pose.pose.position.z = 0;
  odom_.value.pose.pose.orientation = tf2::toMsg(q);
  odom_.value.twist.twist.linear.x = vx;
  odom_.value.twist.twist.angular.z = vyaw;
  odom_.value.pose.covariance = cov_array;
  odom_.value.twist.covariance = cov_array;

  pub_.odom->publish(odom_.value);
}

void HtCanControl::pkgSetVelocity(double linear_x, double linear_y, double angular_z)
{
  VCI_CAN_OBJ tx_buff;

  bool use_akm_mode = (linear_x == 0) && (linear_y == 0) && (angular_z != 0);

  tx_buff.ID = 0x001;
  tx_buff.RemoteFlag = 0;
  tx_buff.ExternFlag = 0;
  tx_buff.SendType = 0;
  tx_buff.DataLen = 8;

  tx_buff.Data[0] = 0x01;
  tx_buff.Data[1] = !use_akm_mode;
  tx_buff.Data[2] = use_akm_mode ? 0x00 : ((((int16_t)(linear_x * 1000)) >> 0) & 0x00FF);
  tx_buff.Data[3] = use_akm_mode ? 0x00 : ((((int16_t)(linear_x * 1000)) >> 8) & 0x00FF);
  tx_buff.Data[4] = use_akm_mode ? 0x00 : ((((int16_t)(linear_y * 1000)) >> 8) & 0x00FF);
  tx_buff.Data[5] = use_akm_mode ? 0x00 : ((((int16_t)(linear_y * 1000)) >> 8) & 0x00FF);
  tx_buff.Data[6] = (((int16_t)(angular_z * 1000)) >> 0) & 0x00FF;
  tx_buff.Data[7] = (((int16_t)(angular_z * 1000)) >> 8) & 0x00FF;

  canTransmit(&tx_buff, 1);
}

void HtCanControl::pkgSetMotion(FourWheelSteerMotion motion)
{
  VCI_CAN_OBJ tx_buff;

  if(motion.motion_mode > 2) {return;}

  tx_buff.ID = 0x001;
  tx_buff.RemoteFlag = 0;
  tx_buff.ExternFlag = 0;
  tx_buff.SendType = 0;
  tx_buff.DataLen = 8;

  if(motion.motion_mode == 0)  /* akm */
  {
    tx_buff.Data[0] = 0x01;
    tx_buff.Data[1] = motion.motion_mode;
    tx_buff.Data[2] = ((((int16_t)(motion.linear * 1000)) >> 0) & 0x00FF);
    tx_buff.Data[3] = ((((int16_t)(motion.linear * 1000)) >> 8) & 0x00FF);
    tx_buff.Data[4] = ((((int16_t)(rad_to_angle(motion.steering) * 100.0)) >> 0) & 0x00FF);
    tx_buff.Data[5] = ((((int16_t)(rad_to_angle(motion.steering) * 100.0)) >> 8) & 0x00FF);
    tx_buff.Data[6] = ((((int16_t)(motion.angular * 1000)) >> 0) & 0x00FF);
    tx_buff.Data[7] = ((((int16_t)(motion.angular * 1000)) >> 8) & 0x00FF);
  }

  if(motion.motion_mode == 1)  /* ftfd */
  {
    tx_buff.Data[0] = 0x01;
    tx_buff.Data[1] = motion.motion_mode;
    tx_buff.Data[2] = ((((int16_t)(motion.linear_x * 1000)) >> 0) & 0x00FF);
    tx_buff.Data[3] = ((((int16_t)(motion.linear_x * 1000)) >> 8) & 0x00FF);
    tx_buff.Data[4] = ((((int16_t)(motion.linear_y * 1000)) >> 0) & 0x00FF);
    tx_buff.Data[5] = ((((int16_t)(motion.linear_y * 1000)) >> 8) & 0x00FF);
    tx_buff.Data[6] = ((((int16_t)(motion.angular_z * 1000)) >> 0) & 0x00FF);
    tx_buff.Data[7] = ((((int16_t)(motion.angular_z * 1000)) >> 8) & 0x00FF);
  }

  if(motion.motion_mode == 2)  /* double akm */
  {
    tx_buff.Data[0] = 0x01;
    tx_buff.Data[1] = motion.motion_mode;
    tx_buff.Data[2] = ((((int16_t)(motion.linear * 1000)) >> 0) & 0x00FF);
    tx_buff.Data[3] = ((((int16_t)(motion.linear * 1000)) >> 8) & 0x00FF);
    tx_buff.Data[4] = ((((int16_t)(rad_to_angle(motion.steering) * 100.0)) >> 0) & 0x00FF);
    tx_buff.Data[5] = ((((int16_t)(rad_to_angle(motion.steering) * 100.0)) >> 8) & 0x00FF);
    tx_buff.Data[6] = ((((int16_t)(motion.angular * 1000)) >> 0) & 0x00FF);
    tx_buff.Data[7] = ((((int16_t)(motion.angular * 1000)) >> 8) & 0x00FF);
  }

  canTransmit(&tx_buff, 1);
}

void HtCanControl::pkgSetAutoCharge(uint8_t charge_type)
{
  VCI_CAN_OBJ tx_buff;

  if((charge_type != 0x00) && (charge_type != 0x01) && (charge_type != 0x02)) {return;}

  tx_buff.ID = 0x001;
  tx_buff.RemoteFlag = 0;
  tx_buff.ExternFlag = 0;
  tx_buff.SendType = 0;
  tx_buff.DataLen = 8;

  tx_buff.Data[0] = 0x02;
  tx_buff.Data[1] = charge_type;
  tx_buff.Data[2] = 0x00;
  tx_buff.Data[3] = 0x00;
  tx_buff.Data[4] = 0x00;
  tx_buff.Data[5] = 0x00;
  tx_buff.Data[6] = 0x00;
  tx_buff.Data[7] = 0x00;

  canTransmit(&tx_buff, 1);
}

void HtCanControl::pkgCollisionClean(void)
{
  VCI_CAN_OBJ tx_buff;

  tx_buff.ID = 0x001;
  tx_buff.RemoteFlag = 0;
  tx_buff.ExternFlag = 0;
  tx_buff.SendType = 0;
  tx_buff.DataLen = 8;

  tx_buff.Data[0] = 0x20;
  tx_buff.Data[1] = 0x01;
  tx_buff.Data[2] = 0x00;
  tx_buff.Data[3] = 0x00;
  tx_buff.Data[4] = 0x00;
  tx_buff.Data[5] = 0x00;
  tx_buff.Data[6] = 0x00;
  tx_buff.Data[7] = 0x00;

  canTransmit(&tx_buff, 1);
}

void HtCanControl::pkgSetLamp(bool enable)
{
  VCI_CAN_OBJ tx_buff;

  tx_buff.ID = 0x001;
  tx_buff.RemoteFlag = 0;
  tx_buff.ExternFlag = 0;
  tx_buff.SendType = 0;
  tx_buff.DataLen = 8;

  tx_buff.Data[0] = 0x40;
  tx_buff.Data[1] = enable;
  tx_buff.Data[2] = 0x00;
  tx_buff.Data[3] = 0x00;
  tx_buff.Data[4] = 0x00;
  tx_buff.Data[5] = 0x00;
  tx_buff.Data[6] = 0x00;
  tx_buff.Data[7] = 0x00;

  canTransmit(&tx_buff, 1);
}

void HtCanControl::pkgLoopInit(void)
{
  pkg_.step = 0;

  /* get once */

  pkg_.idle.setOnce(true);
  pkg_.idle.setTimeout(3000);
  pkg_.idle.flagClean();

  /* upload */

  //

  /* loop get */

  //

  /* not use */

  //

}

void HtCanControl::pkgLoopReset(void)
{
  pkg_.step = 0;

  pkg_.idle.flagClean();
}

void HtCanControl::pkgGetLoop(void)
{
  if(step_.finish() != true) {return;}

  PkgGetCode code = PkgGetCode::wait;

  for(size_t step_loop = 0; step_loop < pkg_.max_num; step_loop++)
  {
    /* get once */

    if(pkg_.step == 0)
    {
      code = pkgGetCheck(pkg_.idle, pkg_.step);
      if(code == PkgGetCode::send)    {pkg_.idle.setResponse();}
      if(code == PkgGetCode::send)    {/* logs_info("try idle step."); */ break;}
      if(code == PkgGetCode::timeout) {/* logs_error("idle step timeout."); */ break;}
    }

    /* upload */

    //

    /* loop get */

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

PkgGetCode HtCanControl::pkgGetCheck(PkgGet &pkg, uint32_t &step)
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
