// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node.hpp"

// Control Table Addresses
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 37

// Data Byte Lengths
#define LEN_GOAL_POSITION         2
#define LEN_PRESENT_POSITION      2

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default Settings
#define BAUDRATE                  1000000
#define DEVICE_NAME               "/dev/ttyUSB0"  // Update for your system
// #define BROADCAST_ID              0xFE  // Broadcast ID for disabling torque
#define DXL_ID1                   1
#define DXL_ID2                   2
#define DXL_ID3                   3

// Goal Positions
#define DXL_MIN_POSITION_VALUE    0
#define DXL_MAX_POSITION_VALUE    1023

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;
// dynamixel::GroupSyncRead * groupSyncRead;
dynamixel::GroupSyncWrite * groupSyncWrite;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_position_subscriber_ =
    this->create_subscription<Jointstate>(
    "set_position",
    QOS_RKL10V,
    [this](const Jointstate::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      // uint16_t goal_position = (unsigned int)msg->position;  // Convert int32 -> uint32
      // std::cout << "id : "  << int(msg->name[0]) <<"\n" ;
      // std::cout << "msg: " << goal_position <<"\n" ;

      // // Write Goal Position (length : 4 bytes)
      // // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      // dxl_comm_result =
      // packetHandler->write2ByteTxRx(
      //   portHandler,
      //   (uint8_t) std::stoi(msg->name[0]),
      //   ADDR_GOAL_POSITION,
      //   goal_position,
      //   &dxl_error
      // );

      // if (dxl_comm_result != COMM_SUCCESS) {
      //   RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      // } else if (dxl_error != 0) {
      //   RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      // } else {
      //   RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", std::stoi(msg->name[0]), msg->position[0]);
      // }

      // Read Multiple present position from motors
      // uint8_t goal_motor_cnt_msg = msg.position.size();
      // uint8_t goal_id_array_uint8[goal_motor_cnt_msg];
      // int16_t goal_position[goal_motor_cnt_msg];

      // for(int dxl_cnt = 0; dxl_cnt< goal_motor_cnt_msg; dxl_cnt++){
      //   goal_id_array_uint8[dxl_cnt] = (uint8_t) std::stoi(msg->name[dxl_cnt]);
      //   goal_position[dxl_cnt] = msg->position[dxl_cnt];    
      //   cout << "goal_id_array_uint8[dxl_cnt]: " << +dxl_cnt 
      //       << "  goal_position[dxl_cnt]" << +goal_position[dxl_cnt] << endl;
      // }

      // // Create a buffer with a pointer to the data
      // uint8_t *param_buffer_0 = &msg->position[0];
      // uint8_t *param_buffer_1 = &msg->position[1];
      // std::cout << "test: "  << param_buffer_0 << '\n';
      // groupSyncWrite->addParam((uint8_t) std::stoi(msg->name[0]), param_buffer_0);
      // groupSyncWrite->addParam((uint8_t) std::stoi(msg->name[1]), param_buffer_1);
      // std::cout << "test"  << '\n';

      // // Transmit SyncWrite Packet
      // int dxl_comm_result = groupSyncWrite->txPacket();
      // if (dxl_comm_result != COMM_SUCCESS) {
      //     RCLCPP_ERROR(rclcpp::get_logger("dynamixel_node"), packetHandler->getTxRxResult(dxl_comm_result));
      // }
      // groupSyncWrite->clearParam();
      // Goal positions
      // uint8_t goalPosition1; // Target position for motor 1
      // uint8_t goalPosition2; // Target position for motor 2

      // // Prepare parameter storage
      // uint8_t *paramGoalPosition1 = goalPosition1;
      // uint8_t paramGoalPosition2[1];

      // paramGoalPosition1[0] = DXL_LOBYTE(goalPosition1);
      // paramGoalPosition1[1] = DXL_HIBYTE(goalPosition1);

      // paramGoalPosition2[0] = DXL_LOBYTE(goalPosition2);
      // paramGoalPosition2[1] = DXL_HIBYTE(goalPosition2);
      uint8_t goal_position = (uint8_t)msg->position[0];
      groupSyncWrite->addParam((uint8_t) DXL_ID1, &goal_position);
      std::cout << "TEST" << '\n';
      // Add parameters to syncWrite
      // if (!groupSyncWrite->addParam(DXL_ID1, &goal_position)) { // Pass a pointer to goal_position
      //     std::cerr << "Failed to add motor 1 parameters." << std::endl;
      //     return;
      // }
      std::cout << 'TEST' << '\n';
      // if (!groupSyncWrite->addParam(DXL_ID2, paramGoalPosition2)) {
      //     std::cerr << "Failed to add motor 2 parameters." << std::endl;
      //     return;
      // }

      // Execute syncWrite
      if (groupSyncWrite->txPacket() != COMM_SUCCESS) {
          std::cerr << "Failed to execute syncWrite!" << std::endl;
      }

      // Clear parameter storage
      groupSyncWrite->clearParam();


    }
    );

    // set up Publisher
    jointstates_publisher = this->create_publisher<Jointstate>("motor_feedback", 30);
    // timer_ = this->create_wall_timer(
    // 30ms, std::bind(&ReadWriteNode::readDxl_publish_callback, this));


  auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint16_t *>(&present_position),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
}

ReadWriteNode::~ReadWriteNode()
{
}

// void ReadWriteNode::readDxl_publish_callback()
// {

// }

void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  
  // Initialize GroupSyncWrite
  // groupSyncWrite = dynamixel::GroupSyncWrite::(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  // Close Port
  portHandler->closePort();

  return 0;
}
