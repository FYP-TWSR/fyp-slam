// #ifndef RM_SERIAL_DRIVER__SENTRY_SERIAL_DRIVER_HPP_
// #define RM_SERIAL_DRIVER__SENTRY_SERIAL_DRIVER_HPP_

// // ROS
// #include <rclcpp/logging.hpp>
// #include <rclcpp/publisher.hpp>
// #include <rclcpp/qos.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/subscription.hpp>
// #include <rclcpp/utilities.hpp>
// #include <serial_driver/serial_driver.hpp>
// #include <visualization_msgs/msg/marker.hpp>

// // TF2
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>

// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// // C++ system
// #include <functional>
// #include <future>
// #include <map>
// #include <memory>
// #include <std_msgs/msg/float64.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <string>
// #include <thread>
// #include <vector>

// // rm_serial_driver and auto_aim_interfaces
// #include "auto_aim_interfaces/msg/cvmode.hpp"
// #include "auto_aim_interfaces/msg/gimbal_command.hpp"
// #include "auto_aim_interfaces/msg/target.hpp"
// #include "rm_serial_driver/crc.hpp"
// #include "rm_serial_driver/protocol.hpp"

// namespace rm_serial_driver
// {
// class SentrySerialDriver : public rclcpp::Node
// {
// public:
//   explicit SentrySerialDriver(const rclcpp::NodeOptions & options);

//   ~SentrySerialDriver() override;

// private:
//   // For initializtion
//   void initSerialParams();

//   // Receiving data from serial port
//   void receiveData();
//   void setParam(const rclcpp::Parameter & param);
//   void resetArmorTracker();

//   // Sending data to serial port
//   void sendGimbalCommand(auto_aim_interfaces::msg::GimbalCommand::SharedPtr msg);

//   // Reopen the port when close
//   void reopenPort();

//   // Broadcast tf from odom to gimbal_link
//   double timestamp_offset_ = 0;
//   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

//   // CVmode publisher and gimbal command subscriber
//   rclcpp::Publisher<auto_aim_interfaces::msg::Cvmode>::SharedPtr cvmode_pub_;
//   rclcpp::Subscription<auto_aim_interfaces::msg::GimbalCommand>::SharedPtr gimbal_command_sub_;

//   // Param client to set detect_color
//   using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
//   bool initial_set_param_ = false;
//   uint8_t previous_receive_color_ = 1;  // 1 for BLUE, 0 for RED
//   rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
//   ResultFuturePtr set_param_future_;

//   // Service client to reset tracker
//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

//   // For debug usage
//   rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

//   // Serial port related
//   std::unique_ptr<IoContext> owned_ctx_;
//   std::string device_name_;
//   std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
//   std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
//   std::thread receive_thread_;
// };
// }  // namespace rm_serial_driver

// #endif  // RM_SERIAL_DRIVER__SENTRY_SERIAL_DRIVER_HPP_
