#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start SerialDriver!");

  // init parameters for serial port
  this->initSerialParams();

  // Messages sent out:
  // 1. TF broadcaster and publishers
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // 2. CVmode Publisher
  mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/serial_driver/mode", 10);
  // 3. Armor Detector parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
  // 4. Armor Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/armor_tracker/reset");
  // 5. Latency publisher for debugging
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/serial_driver/debug_latency", 10);

  // Messages received:
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/serial_driver/command", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendCommand, this, std::placeholders::_1));

  // Open serial port
  while (true) {
    try {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open()) {
        serial_driver_->port()->open();
        receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
      }
      if (serial_driver_->port()->is_open()) break;
    } catch (const std::exception & ex) {
      // throw exception
      RCLCPP_ERROR(
        get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    }
  }
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(3);
  std::vector<uint8_t> data;
  data.reserve(sizeof(BaseMsg));

  // rclcpp::Clock clock;
  // int64_t lastRxTime = clock.now().nanoseconds() / 1000000;

  // __attribute_used__ int dataLenth = 0;
  // int64_t microseconds = now. / 1000;

  while (rclcpp::ok()) {
    try {
    startRx:
      data.resize(1);
      while (true) {
        serial_driver_->port()->receive(data);
        if (data[0] == 0xAA) {
          header[0] = data[0];
          // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "header found: %d", data[0]);
          break;
        } else {
          // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "resyncing: %d", data[0]);
        }
      }
      serial_driver_->port()->receive(data);
      header[1] = data[0];
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "dataLen: %d", data[0]);
      // dataLenth = data[0];
      serial_driver_->port()->receive(data);
      header[2] = data[0];

      if (header[1] != 24 || header[2] != GIMBAL_MSG) {
        // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "protocolId: %d", data[0]);
        goto startRx;
      }

      // serial_driver_->port()->receive(header);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "start_byte: %d", header[0]);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "dataLen: %d", header[1]);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "protocolId: %d", header[2]);

      // if (data[0] == GIMBAL_MSG) {
      data.resize(header[1] + 2);
      serial_driver_->port()->receive(data);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "-2: %d", header[1]);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "-1: %d", data[35]);
      // for(int i = 0; i < 38; i++)
      // {
      //   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1, "data[%d]: %u", i, data[i]);
      //   std::cout << "data[" << i <<"]: "  << unsigned(data[i]) << std::endl;
      // }

      data.insert(data.begin(), header[2]);
      data.insert(data.begin(), header[1]);
      data.insert(data.begin(), header[0]);
      BaseMsg packet = fromVector(data);

      // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 20, "cur_cv_mode: %d", packet.cur_cv_mode);
      // RCLCPP_WARN_THROTTLE(
      //   get_logger(), *get_clock(), 20, "target_color: %d", packet.target_color);
      // RCLCPP_ERROR_THROTTLE(
      //   get_logger(), *get_clock(), 20, "bullet_speed: %.2f", packet.bullet_speed);

      bool crc_ok =
        crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));

      // }
      // else
      // {

      // RCLCPP_WARN_THROTTLE(
      // get_logger(), *get_clock(), 20, "eulerAngles: %.4f", packet.eulerAngles[0]);
      // RCLCPP_WARN_THROTTLE(
      // get_logger(), *get_clock(), 20, "eulerAngles: %.4f", packet.eulerAngles[1]);
      // RCLCPP_WARN_THROTTLE(
      // get_logger(), *get_clock(), 20, "eulerAngles: %.4f", packet.eulerAngles[2]);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "accel: %.4f", packet.accel[0]);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "accel: %.4f", packet.accel[1]);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "accel: %.4f", packet.accel[2]);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "gyroXYZ: %.4f", packet.gyroXYZ[0]);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "gyroXYZ: %.4f", packet.gyroXYZ[1]);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "gyroXYZ: %.4f", packet.gyroXYZ[2]);
      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "checksum: %d", packet.checksum);
      // }
      if (!crc_ok) {
        RCLCPP_ERROR(get_logger(), "CRC error!");
        continue;
      }
        // To save space, reuse the variable "target_color" to indicate whether to reset tracker
        // if (packet.target_color == 2) {
        //   resetArmorTracker();
        // } else {
        //   if (!initial_set_param_ || packet.target_color != previous_receive_color_) {
        //     setParam(rclcpp::Parameter("detect_color", packet.target_color));
        //     // setParam(rclcpp::Parameter("detect_color", 1));
        //     // setParam(rclcpp::Parameter("detect_color", 0));
        //     previous_receive_color_ = packet.target_color;
        //   }
        // }
        
        std_msgs::msg::UInt8 mode_msg;
        // cvmode_msg.bullet_speed = packet.bullet_speed;
        // cvmode_msg.cur_cv_mode = packet.cur_cv_mode;
        mode_pub_->publish(mode_msg);

        geometry_msgs::msg::TransformStamped t;
        timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
        t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
        t.header.frame_id = "world";
        t.child_frame_id = "serial_base";
        tf2::Quaternion q;
        q.setRPY(packet.eulerAngles[2], packet.eulerAngles[1], packet.eulerAngles[0]);  // roll, pitch, yaw
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendCommand(geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    BaseCommand base_command_msg;
    base_command_msg.dataLen = sizeof(BaseCommand) - 5;
    base_command_msg.protocolID = GIMBAL_CMD;
    // base_command_msg.target_pitch = -msg->linear.x;
    // base_command_msg.target_yaw = msg->linear.y;
    // base_command_msg.shoot_mode = msg->linear.z;
    base_command_msg.target_linear_vel_x = msg->linear.x;
    base_command_msg.target_linear_vel_y = msg->linear.y;
    base_command_msg.target_angular_vel_z = msg->angular.z;

    crc16::Append_CRC16_Check_Sum(
      reinterpret_cast<uint8_t *>(&base_command_msg), sizeof(BaseCommand));

    std::vector<uint8_t> data = toVector(base_command_msg);

    serial_driver_->port()->send(data);
    // std_msgs::msg::Float64 latency;
    // latency.data = (this->now() - msg->header.stamp).seconds();
    // RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "s");
    // latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::initSerialParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    // RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetArmorTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
