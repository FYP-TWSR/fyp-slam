#include "rm_serial_driver/sentry_serial_driver.hpp"

namespace rm_serial_driver
{
SentrySerialDriver::SentrySerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start SentrySerialDriver!");

  // init parameters for serial port
  this->initSerialParams();

  // Messages sent out:
  // 1. TF broadcaster and publishers
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // 2. CVmode Publisher
  cvmode_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::Cvmode>("/serial_driver/cv_mode", 10);
  // 3. Armor Detector parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
  // 4. Armor Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/armor_tracker/reset");
  // 5. Latency publisher for debugging
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/serial_driver/debug_latency", 10);

  // Messages received:
  gimbal_command_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalCommand>(
    "/serial_driver/gimbal_command", rclcpp::SensorDataQoS(),
    std::bind(&SentrySerialDriver::sendGimbalCommand, this, std::placeholders::_1));

  // Open serial port
  while (true) {
    try {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open()) {
        serial_driver_->port()->open();
        receive_thread_ = std::thread(&SentrySerialDriver::receiveData, this);
      }
      if (serial_driver_->port()->is_open()) break;
    } catch (const std::exception & ex) {
      // throw exception
      RCLCPP_ERROR(
        get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    }
  }

  while (true) {
    SentryGimbalCommand sentry_gimbal_command_msg;
    // header
    sentry_gimbal_command_msg.dataLen = sizeof(SentryGimbalCommand) - 5;
    sentry_gimbal_command_msg.protocolID = SENTRY_GIMBAL_CMD;
    // left gimbal control command
    sentry_gimbal_command_msg.left_target_pitch = 1;
    sentry_gimbal_command_msg.left_target_yaw = 2;
    sentry_gimbal_command_msg.left_shoot_mode = 3;
    // right gimbal control command
    sentry_gimbal_command_msg.right_target_pitch = 4;
    sentry_gimbal_command_msg.right_target_yaw = 5;
    sentry_gimbal_command_msg.right_shoot_mode = 6;
    // navigation control command
    sentry_gimbal_command_msg.vel_x = 7.0;
    sentry_gimbal_command_msg.vel_y = 8.0;
    sentry_gimbal_command_msg.vel_w = 9.0;

    crc16::Append_CRC16_Check_Sum(
      reinterpret_cast<uint8_t *>(&sentry_gimbal_command_msg), sizeof(SentryGimbalCommand));

    std::vector<uint8_t> data = toSentryVector(sentry_gimbal_command_msg);

    serial_driver_->port()->send(data);

    // control the rate to be 500 Hz
    rclcpp::sleep_for(std::chrono::milliseconds(2));
  }
}

SentrySerialDriver::~SentrySerialDriver()
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

void SentrySerialDriver::receiveData()
{
  std::vector<uint8_t> header(3);
  std::vector<uint8_t> data;
  data.reserve(sizeof(SentryGimbalMsg));
  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "start_byte: %d", header[0]);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "dataLen: %d", header[1]);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "protocolId: %d", header[2]);

      if (header[2] == SENTRY_GIMBAL_MSG) {
        data.resize(sizeof(SentryGimbalMsg) - 3);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[2]);
        data.insert(data.begin(), header[1]);
        data.insert(data.begin(), header[0]);
        SentryGimbalMsg packet = fromSentryVector(data);

        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 20, "cur_cv_mode: %d", packet.cur_cv_mode);
        // RCLCPP_WARN_THROTTLE(
        //   get_logger(), *get_clock(), 20, "target_color: %d", packet.target_color);
        // RCLCPP_ERROR_THROTTLE(
        //   get_logger(), *get_clock(), 20, "bullet_speed: %.2f", packet.bullet_speed);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));

        if (!crc_ok) {
          RCLCPP_ERROR(get_logger(), "CRC error!");
          continue;
        }

        // To save space, reuse the variable "target_color" to indicate whether to reset tracker
        if (packet.target_color == 2) {
          resetArmorTracker();
        } else {
          if (!initial_set_param_ || packet.target_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.target_color));
            // setParam(rclcpp::Parameter("detect_color", 1));
            // setParam(rclcpp::Parameter("detect_color", 0));
            previous_receive_color_ = packet.target_color;
          }
        }

        geometry_msgs::msg::TransformStamped t;
        timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
        t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
        t.header.frame_id = "odom";
        t.child_frame_id = "gimbal_link";
        tf2::Quaternion q(
          packet.left_gimbal_q_x, packet.left_gimbal_q_y, packet.left_gimbal_q_z,
          packet.left_gimbal_q_w);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void SentrySerialDriver::sendGimbalCommand(auto_aim_interfaces::msg::GimbalCommand::SharedPtr msg)
{
  try {
    SentryGimbalCommand sentry_gimbal_command_msg;
    // header
    sentry_gimbal_command_msg.dataLen = sizeof(SentryGimbalCommand) - 5;
    sentry_gimbal_command_msg.protocolID = SENTRY_GIMBAL_CMD;
    // left gimbal control command
    sentry_gimbal_command_msg.left_target_pitch = -msg->gimbal_command.x;
    sentry_gimbal_command_msg.left_target_yaw = msg->gimbal_command.y;
    sentry_gimbal_command_msg.left_shoot_mode = msg->gimbal_command.z;
    // right gimbal control command
    sentry_gimbal_command_msg.right_target_pitch = -msg->gimbal_command.x;
    sentry_gimbal_command_msg.right_target_yaw = msg->gimbal_command.y;
    sentry_gimbal_command_msg.right_shoot_mode = msg->gimbal_command.z;
    // navigation control command
    sentry_gimbal_command_msg.vel_x = 1.0;
    sentry_gimbal_command_msg.vel_y = 2.0;
    sentry_gimbal_command_msg.vel_w = 3.0;

    crc16::Append_CRC16_Check_Sum(
      reinterpret_cast<uint8_t *>(&sentry_gimbal_command_msg), sizeof(SentryGimbalCommand));

    std::vector<uint8_t> data = toSentryVector(sentry_gimbal_command_msg);

    serial_driver_->port()->send(data);
    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds();
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "s");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void SentrySerialDriver::initSerialParams()
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

void SentrySerialDriver::reopenPort()
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

void SentrySerialDriver::setParam(const rclcpp::Parameter & param)
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

void SentrySerialDriver::resetArmorTracker()
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
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::SentrySerialDriver)
