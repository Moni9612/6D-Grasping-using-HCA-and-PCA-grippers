#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

// Control table addresses for XM430
#define ADDR_OPERATING_MODE      11
#define ADDR_TORQUE_ENABLE       64
#define ADDR_HARDWARE_ERROR      70
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_POSITION   132

#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define EXTENDED_POSITION_CONTROL_MODE 4

#define TICKS_PER_REVOLUTION    4096
#define ONE_TURN_TICKS          TICKS_PER_REVOLUTION

#define PROTOCOL_VERSION        2.0

#define DXL_ID                  0
#define BAUDRATE                57600
#define DEVICE_NAME             "/dev/ttyUSB0"

class DynamixelPositionNode : public rclcpp::Node
{
public:
  DynamixelPositionNode() : Node("dynamixel_position_node"), dxl_id_(DXL_ID), baudrate_(BAUDRATE)
  {
    RCLCPP_INFO(this->get_logger(), "Starting Dynamixel Position Node");

    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler_->openPort()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open port");
      throw std::runtime_error("Failed to open port");
    }

    if (!portHandler_->setBaudRate(baudrate_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to set baudrate");
      throw std::runtime_error("Failed to set baudrate");
    }

    uint16_t model_number = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->ping(
      portHandler_, dxl_id_, &model_number, &dxl_error);
    if (!check_comm_result(dxl_comm_result, dxl_error, "ping")) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not ping ID %u at baud %d. Trying auto-detect...",
        dxl_id_,
        baudrate_);
      if (!auto_detect_motor()) {
        throw std::runtime_error("ping");
      }
      dxl_error = 0;
      dxl_comm_result = packetHandler_->ping(
        portHandler_, dxl_id_, &model_number, &dxl_error);
      check_comm_result_or_throw(dxl_comm_result, dxl_error, "ping after auto-detect");
    }
    RCLCPP_INFO(this->get_logger(), "Connected to Dynamixel ID %u (model: %u)", dxl_id_, model_number);

    dxl_comm_result = packetHandler_->write1ByteTxRx(
      portHandler_, dxl_id_, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    check_comm_result_or_throw(dxl_comm_result, dxl_error, "disable torque before mode set");

    dxl_comm_result = packetHandler_->write1ByteTxRx(
      portHandler_, dxl_id_, ADDR_OPERATING_MODE, EXTENDED_POSITION_CONTROL_MODE, &dxl_error);
    check_comm_result_or_throw(dxl_comm_result, dxl_error, "set extended position control mode");

    dxl_comm_result = packetHandler_->write1ByteTxRx(
      portHandler_, dxl_id_, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    check_comm_result_or_throw(dxl_comm_result, dxl_error, "enable torque");

    RCLCPP_INFO(this->get_logger(), "Torque enabled");

    command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/motor_control",
      10,
      std::bind(&DynamixelPositionNode::on_command, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Listening on /motor_control for 'forward' and 'backward'");
  }

  ~DynamixelPositionNode()
  {
    uint8_t dxl_error = 0;
    packetHandler_->write1ByteTxRx(
      portHandler_, dxl_id_, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    portHandler_->closePort();
  }

private:
  void on_command(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data != "forward" && msg->data != "backward") {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring unknown command: '%s'",
        msg->data.c_str());
      return;
    }

    uint8_t dxl_error = 0;
    int dxl_comm_result = 0;

    uint32_t present_position = 0;
    dxl_comm_result = packetHandler_->read4ByteTxRx(
      portHandler_, dxl_id_, ADDR_PRESENT_POSITION, &present_position, &dxl_error);
    if (!check_comm_result(dxl_comm_result, dxl_error, "read present position")) {
      return;
    }

    int32_t goal_position = 0;
    if (msg->data == "forward") {
      goal_position = static_cast<int32_t>(present_position) + ONE_TURN_TICKS;
    } else {
      goal_position = static_cast<int32_t>(present_position) - ONE_TURN_TICKS;
    }

    dxl_comm_result = packetHandler_->write4ByteTxRx(
      portHandler_, dxl_id_, ADDR_GOAL_POSITION, goal_position, &dxl_error);
    if (!check_comm_result(dxl_comm_result, dxl_error, "write goal position")) {
      return;
    }

    uint8_t hw_error = 0;
    dxl_comm_result = packetHandler_->read1ByteTxRx(
      portHandler_, dxl_id_, ADDR_HARDWARE_ERROR, &hw_error, &dxl_error);
    if (!check_comm_result(dxl_comm_result, dxl_error, "read hardware error status")) {
      return;
    }

    if (hw_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "Hardware error status is non-zero: 0x%02X", hw_error);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Command: %s, Goal: %d, Present: %u, HW err: 0x%02X",
      msg->data.c_str(),
      goal_position,
      present_position,
      hw_error);
  }

  bool auto_detect_motor()
  {
    const int baud_candidates[] = {57600, 115200, 1000000, 2000000, 3000000, 4000000};

    for (const int baud : baud_candidates) {
      if (!portHandler_->setBaudRate(baud)) {
        continue;
      }

      for (uint8_t id = 0; id <= 20; ++id) {
        uint16_t model_number = 0;
        uint8_t dxl_error = 0;
        const int dxl_comm_result = packetHandler_->ping(
          portHandler_, id, &model_number, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
          dxl_id_ = id;
          baudrate_ = baud;
          RCLCPP_INFO(
            this->get_logger(),
            "Auto-detect success: ID %u at baud %d (model: %u)",
            dxl_id_,
            baudrate_,
            model_number);
          return true;
        }
      }
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "Auto-detect failed. Checked IDs 0-20 at common baud rates. Verify power/wiring/protocol/ID.");
    return false;
  }

  bool check_comm_result(int dxl_comm_result, uint8_t dxl_error, const char * action)
  {
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "%s failed: %s",
        action,
        packetHandler_->getTxRxResult(dxl_comm_result));
      return false;
    }

    if (dxl_error != 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "%s failed with DXL error: %s",
        action,
        packetHandler_->getRxPacketError(dxl_error));
      return false;
    }

    return true;
  }

  void check_comm_result_or_throw(int dxl_comm_result, uint8_t dxl_error, const char * action)
  {
    if (!check_comm_result(dxl_comm_result, dxl_error, action)) {
      throw std::runtime_error(action);
    }
  }

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  uint8_t dxl_id_;
  int baudrate_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DynamixelPositionNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
