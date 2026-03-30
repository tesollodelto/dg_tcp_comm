// Copyright 2025 TESOLLO
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the TESOLLO nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "delto_tcp_comm/delto_developer_TCP.hpp"

namespace DeltoTCP {

// ============================================================================
// Helper Functions
// ============================================================================

int Communication::GetMotorCount(uint16_t model) {
  switch (model) {
    case static_cast<uint16_t>(ModelType::DG1F):
      return 3;
    case static_cast<uint16_t>(ModelType::DG2F):
      return 6;
    case static_cast<uint16_t>(ModelType::DG3F_B):
    case static_cast<uint16_t>(ModelType::DG3F_M):
      return 12;
    case static_cast<uint16_t>(ModelType::DG4F):
      return 18;
    case static_cast<uint16_t>(ModelType::DG5F):
    case static_cast<uint16_t>(ModelType::DG5F_L):
    case static_cast<uint16_t>(ModelType::DG5F_R):
    case static_cast<uint16_t>(ModelType::DG5F_L_S):
    case static_cast<uint16_t>(ModelType::DG5F_R_S):
      return 20;
    case static_cast<uint16_t>(ModelType::DG5F_L_S15):
    case static_cast<uint16_t>(ModelType::DG5F_R_S15):
      return 15;
    default:
      std::cerr << "Unknown model type: 0x" << std::hex << model << std::dec
                << std::endl;
      return 12;
  }
}

int Communication::GetBytePerMotor(uint16_t model) {
  // DG3F-B (구형): 5 bytes per motor
  // ID(1) + PosL(1) + PosH(1) + CurL(1) + CurH(1)
  if (model == static_cast<uint16_t>(ModelType::DG3F_B)) {
    return 5;
  }

  // 신형 모델들: 8 bytes per motor
  // ID(1) + PosL(1) + PosH(1) + CurL(1) + CurH(1) + TempL(1) + TempH(1) + Vel(1)
  return 8;
}

int Communication::GetFingerCount(uint16_t model) {
  switch (model) {
    case static_cast<uint16_t>(ModelType::DG1F):
      return 1;
    case static_cast<uint16_t>(ModelType::DG2F):
      return 2;
    case static_cast<uint16_t>(ModelType::DG3F_B):
    case static_cast<uint16_t>(ModelType::DG3F_M):
      return 3;
    case static_cast<uint16_t>(ModelType::DG4F):
      return 4;
    case static_cast<uint16_t>(ModelType::DG5F):
    case static_cast<uint16_t>(ModelType::DG5F_L):
    case static_cast<uint16_t>(ModelType::DG5F_R):
    case static_cast<uint16_t>(ModelType::DG5F_L_S):
    case static_cast<uint16_t>(ModelType::DG5F_R_S):
    case static_cast<uint16_t>(ModelType::DG5F_L_S15):
    case static_cast<uint16_t>(ModelType::DG5F_R_S15):
      return 5;
    default:
      return 5;
  }
}

bool Communication::IsNewModel() const {
  return model_ != static_cast<uint16_t>(ModelType::DG3F_B);
}

bool Communication::SupportsExtendedFeatures() const {
  return model_ != static_cast<uint16_t>(ModelType::DG3F_B);
}

int Communication::GetSensorBytesPerFinger() const {
  switch (sensor_type_) {
    case SensorType::FT_6AXIS:  return 12;  // 6 axes × 2 bytes
    case SensorType::FT_3AXIS:  return 12;  // 6 axes × 2 bytes (3 unused)
    case SensorType::FT_4AXIS:  return 12;  // 6 axes × 2 bytes (2 unused)
    case SensorType::TACTILE_M: return 15;  // 3×5 × 1 byte
    case SensorType::TACTILE_S: return 36;  // 3×6 × 2 bytes
    default:                    return 0;
  }
}

int Communication::GetSensorFingerCount() const {
  return finger_count_;  // 전체 손가락 슬롯 전송 (비활성은 0)
}

int16_t Communication::CalculateExpectedResponseLength() {
  int16_t length = HEADER_SIZE + motor_count_ * byte_per_motor_;

  if (SupportsExtendedFeatures()) {
    if (fingertip_sensor_ && sensor_type_ != SensorType::NONE) {
      int sensor_fingers = GetSensorFingerCount();
      length += GetSensorBytesPerFinger() * sensor_fingers;
    }
    if (io_) {
      length += GPIO_SIZE;
    }
  }

  return length;
}

int16_t Communication::CombineMsg(uint8_t data1, uint8_t data2) {
  // Big Endian: data1 = high byte, data2 = low byte
  return static_cast<int16_t>(static_cast<uint16_t>(data1 << 8) | data2);
}

int8_t Communication::ConvertByte(uint8_t byte) {
  if (byte >= 0x80) {
    return static_cast<int8_t>(byte - 0x100);
  }
  return static_cast<int8_t>(byte);
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

Communication::Communication(const std::string& ip, int port, uint16_t model,
                             bool fingertip_sensor, bool io)
    : ip_(ip),
      port_(port),
      model_(model),
      actual_model_(0),
      fingertip_sensor_(fingertip_sensor),
      io_(io),
      sensor_type_(SensorType::NONE),
      finger_sensor_mask_(0),
      socket_(io_context_),
      motor_count_(GetMotorCount(model)),
      byte_per_motor_(GetBytePerMotor(model)),
      finger_count_(GetFingerCount(model)),
      expected_response_length_(0) {}

std::string Communication::ModelToString(uint16_t model) {
  switch (model) {
    case static_cast<uint16_t>(ModelType::DG1F):
      return "DG1F (0x1F02)";
    case static_cast<uint16_t>(ModelType::DG2F):
      return "DG2F (0x2F03)";
    case static_cast<uint16_t>(ModelType::DG3F_B):
      return "DG3F-B (0x3F01)";
    case static_cast<uint16_t>(ModelType::DG3F_M):
      return "DG3F-M (0x3F02)";
    case static_cast<uint16_t>(ModelType::DG4F):
      return "DG4F (0x4F02)";
    case static_cast<uint16_t>(ModelType::DG5F):
      return "DG5F (0x5F02)";
    case static_cast<uint16_t>(ModelType::DG5F_L):
      return "DG5F-L (0x5F12)";
    case static_cast<uint16_t>(ModelType::DG5F_R):
      return "DG5F-R (0x5F22)";
    case static_cast<uint16_t>(ModelType::DG5F_L_S):
      return "DG5F-L-S (0x5F14)";
    case static_cast<uint16_t>(ModelType::DG5F_R_S):
      return "DG5F-R-S (0x5F24)";
    case static_cast<uint16_t>(ModelType::DG5F_L_S15):
      return "DG5F-L-S15 (0x5F34)";
    case static_cast<uint16_t>(ModelType::DG5F_R_S15):
      return "DG5F-R-S15 (0x5F44)";
    default:
      return "Unknown (0x" +
             ([](uint16_t v) {
               char buf[8];
               snprintf(buf, sizeof(buf), "%04X", v);
               return std::string(buf);
             })(model) + ")";
  }
}

Communication::~Communication() {
  if (socket_.is_open()) {
    socket_.close();
  }
}

// ============================================================================
// Connection Management
// ============================================================================

void Communication::Connect() {
  tcp::resolver resolver(io_context_);
  boost::system::error_code ec;

  // Ensure socket is closed before attempting to connect
  if (socket_.is_open()) {
    socket_.close();
  }

  boost::asio::connect(socket_, resolver.resolve(ip_, std::to_string(port_)),
                       ec);

  if (ec) {
    std::cerr << "Could not connect: " << ec.message() << std::endl;
    throw std::runtime_error("Connection failed: " + ec.message());
  }

  // Get firmware version and model info
  // Old firmware: 7 bytes (Length=7, no sensor info)
  // New firmware: 9 bytes (Length=9, includes SensorType + FingerMask)
  {
    std::array<uint8_t, 3> request;
    request[0] = 0x00;              // Length_h
    request[1] = 0x03;              // Length_l
    request[2] = GET_VERSION_CMD;   // CMD

    socket_.write_some(boost::asio::buffer(request), ec);
    if (ec) {
      std::cerr << "Error sending version request: " << ec.message()
                << std::endl;
      throw std::runtime_error("Failed to get device info: " + ec.message());
    }

    // Read length field first (2 bytes)
    std::array<uint8_t, 2> len_buf;
    boost::asio::read(socket_, boost::asio::buffer(len_buf),
                      boost::asio::transfer_exactly(2), ec);
    if (ec) {
      std::cerr << "Error reading version length: " << ec.message()
                << std::endl;
      throw std::runtime_error("Failed to read device info: " + ec.message());
    }

    uint16_t resp_length = (static_cast<uint16_t>(len_buf[0]) << 8) | len_buf[1];
    uint16_t remaining = resp_length - 2;  // Length includes the 2-byte length field

    std::vector<uint8_t> payload(remaining);
    boost::asio::read(socket_, boost::asio::buffer(payload),
                      boost::asio::transfer_exactly(remaining), ec);
    if (ec) {
      std::cerr << "Error reading version response: " << ec.message()
                << std::endl;
      throw std::runtime_error("Failed to read device info: " + ec.message());
    }

    // payload[0] = CMD (0x08)
    // payload[1-2] = Model (Big Endian)
    // payload[3-4] = FW Version
    // payload[5] = SensorType (if resp_length >= 9)
    // payload[6] = FingerMask (if resp_length >= 9)

    actual_model_ = CombineMsg(payload[1], payload[2]);
    firmware_version_ = {payload[3], payload[4]};

    if (resp_length >= 9) {
      sensor_type_ = static_cast<SensorType>(payload[5]);
      finger_sensor_mask_ = payload[6];
    } else {
      // Old firmware: use parameter-based defaults
      sensor_type_ = fingertip_sensor_ ? SensorType::FT_6AXIS : SensorType::NONE;
      finger_sensor_mask_ = fingertip_sensor_ ?
          static_cast<uint8_t>((1 << finger_count_) - 1) : 0x00;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "Device Info:" << std::endl;
    std::cout << "  Firmware Version: "
              << static_cast<int>(firmware_version_[0]) << "."
              << static_cast<int>(firmware_version_[1]) << std::endl;
    std::cout << "  Configured Model: " << ModelToString(model_) << std::endl;
    std::cout << "  Actual Model:     " << ModelToString(actual_model_) << std::endl;

    if (resp_length >= 9) {
      std::cout << "  Sensor Type:      0x" << std::hex
                << static_cast<int>(payload[5]) << std::dec << std::endl;
      // Finger mask in binary format
      std::string mask_bin = "0b";
      for (int b = 7; b >= 0; --b) {
        mask_bin += (finger_sensor_mask_ & (1 << b)) ? '1' : '0';
      }
      std::cout << "  Finger Mask:      " << mask_bin << std::endl;
      // Per-finger sensor status (F5 F4 F3 F2 F1 order, MSB→LSB)
      std::cout << "  Finger Sensor:    ";
      for (int i = finger_count_ - 1; i >= 0; --i) {
        bool equipped = finger_sensor_mask_ & (1 << i);
        std::cout << "F" << (i + 1) << ":" << (equipped ? "ON" : "--");
        if (i > 0) std::cout << " ";
      }
      std::cout << std::endl;
    }

    // Validate model
    if (model_ != actual_model_) {
      std::cerr << "========================================" << std::endl;
      std::cerr << "WARNING: Model mismatch detected!" << std::endl;
      std::cerr << "  Expected: " << ModelToString(model_) << std::endl;
      std::cerr << "  Got:      " << ModelToString(actual_model_) << std::endl;
      std::cerr << "  Please check your URDF/xacro configuration." << std::endl;
      std::cerr << "========================================" << std::endl;
    } else {
      std::cout << "  Status:           OK (Model matched)" << std::endl;
    }
    std::cout << "========================================" << std::endl;
  }

  // Calculate expected response length now that sensor info is known
  expected_response_length_ = CalculateExpectedResponseLength();

  std::cout << "Connected to Delto Gripper (Model: " << ModelToString(actual_model_)
            << ", Motors: " << motor_count_ << ")" << std::endl;
}

void Communication::Disconnect() {
  if (socket_.is_open()) {
    socket_.close();
    std::cout << "Disconnected from Delto Gripper" << std::endl;
  }
}

// ============================================================================
// Data Exchange
// ============================================================================

bool Communication::ReadFullPacket(std::vector<uint8_t>& buffer) {
  boost::system::error_code ec;

  buffer.resize(expected_response_length_);

  std::size_t bytes_read = boost::asio::read(
      socket_, boost::asio::buffer(buffer.data(), expected_response_length_),
      boost::asio::transfer_exactly(expected_response_length_), ec);

  if (ec) {
    std::cerr << "Read error: " << ec.message() << std::endl;
    return false;
  }

  return bytes_read == static_cast<std::size_t>(expected_response_length_);
}

DeltoReceivedData Communication::GetData() {
  // Build request packet
  std::vector<uint8_t> request;

  if (model_ == static_cast<uint16_t>(ModelType::DG3F_B)) {
    // DG3F-B: Length(2) + CMD(1) + ID(2)
    request = {0x00, 0x05, GET_DATA_CMD, 0x01, 0x02};
  } else {
    // New models: Length(2) + CMD(1) + ID(4) [+ optional IDs]
    request = {0x00, 0x07, GET_DATA_CMD, 0x01, 0x02, 0x03, 0x04};

    if (SupportsExtendedFeatures()) {
      if (fingertip_sensor_) {
        request.push_back(0x05);
      }
      if (io_) {
        request.push_back(0x06);
      }
      // Update length field
      request[1] = static_cast<uint8_t>(request.size());
    }
  }

  // Send request
  boost::system::error_code ec;
  socket_.write_some(boost::asio::buffer(request), ec);

  if (ec) {
    std::cerr << "Error sending request: " << ec.message() << std::endl;

    // Try to reconnect on connection errors
    if (ec == boost::asio::error::broken_pipe ||
        ec == boost::asio::error::connection_reset ||
        ec == boost::asio::error::connection_aborted) {
      std::cerr << "Connection lost, attempting to reconnect..." << std::endl;
      try {
        socket_.close();
        Connect();
        socket_.write_some(boost::asio::buffer(request), ec);
        if (ec) {
          std::cerr << "Failed to send after reconnection: " << ec.message()
                    << std::endl;
          return DeltoReceivedData{};
        }
      } catch (const std::exception& e) {
        std::cerr << "Reconnection failed: " << e.what() << std::endl;
        return DeltoReceivedData{};
      }
    } else {
      return DeltoReceivedData{};
    }
  }

  // Read response
  std::vector<uint8_t> response;
  if (!ReadFullPacket(response)) {
    std::cerr << "Failed to read full packet" << std::endl;
    return DeltoReceivedData{};
  }

  // Validate response
  uint16_t length = CombineMsg(response[0], response[1]);
  uint8_t cmd = response[2];

  if (cmd != GET_DATA_CMD ||
      static_cast<int>(response.size()) != expected_response_length_) {
    std::cerr << "Invalid header (CMD or LENGTH mismatch)" << std::endl;
    std::cerr << "Expected: " << expected_response_length_
              << ", Got: " << response.size() << std::endl;
    std::cerr << "Received header: CMD = 0x" << std::hex << static_cast<int>(cmd)
              << ", Length = 0x" << static_cast<int>(length) << std::dec
              << std::endl;
    return DeltoReceivedData{};
  }

  // Parse motor data
  DeltoReceivedData received_data;
  received_data.joint.resize(motor_count_);
  received_data.current.resize(motor_count_);

  if (IsNewModel()) {
    received_data.velocity.resize(motor_count_);
    received_data.temperature.resize(motor_count_);
  }

  for (int i = 0; i < motor_count_; i++) {
    size_t base = HEADER_SIZE + i * byte_per_motor_;
    [[maybe_unused]] uint8_t motor_id = response[base];

    uint8_t posL = response[base + 1];
    uint8_t posH = response[base + 2];
    uint8_t curL = response[base + 3];
    uint8_t curH = response[base + 4];

    int16_t raw_position = CombineMsg(posL, posH);
    int16_t raw_current = CombineMsg(curL, curH);

    received_data.joint[i] = raw_position * POSITION_SCALE;
    received_data.current[i] = raw_current * CURRENT_SCALE;

    // New models have temperature and velocity
    if (IsNewModel()) {
      uint8_t tempL = response[base + 5];
      uint8_t tempH = response[base + 6];
      int16_t raw_temperature = CombineMsg(tempL, tempH);
      received_data.temperature[i] = raw_temperature * 0.1;

      int8_t raw_vel = ConvertByte(response[base + 7]);
      received_data.velocity[i] =
          static_cast<double>(raw_vel) * VELOCITY_SCALE * -1;
    }
  }

  // Parse sensor data (type depends on GET_VERSION response)
  if (SupportsExtendedFeatures() && fingertip_sensor_ && sensor_type_ != SensorType::NONE) {
    size_t sensor_base = HEADER_SIZE + motor_count_ * byte_per_motor_;
    int bytes_per_finger = GetSensorBytesPerFinger();

    switch (sensor_type_) {
      case SensorType::FT_6AXIS:
      case SensorType::FT_3AXIS:
      case SensorType::FT_4AXIS: {
        // F/T sensor: 12 bytes/finger (6 axes × 2 bytes)
        // 전체 슬롯 전송, 비활성 손가락은 0
        received_data.fingertip_sensor.resize(finger_count_ * 6);

        for (int finger = 0; finger < finger_count_; finger++) {
          for (int axis = 0; axis < 6; axis++) {
            size_t offset = sensor_base + finger * bytes_per_finger + axis * 2;
            int16_t raw_value = CombineMsg(response[offset], response[offset + 1]);

            if (axis < 3) {
              received_data.fingertip_sensor[finger * 6 + axis] = raw_value * 0.1;
            } else {
              received_data.fingertip_sensor[finger * 6 + axis] = raw_value * 0.001;
            }
          }
        }
        break;
      }
      case SensorType::TACTILE_M: {
        // Tactile M: 15 bytes/finger (3×5, uint8)
        // 전체 슬롯 전송, 비활성 손가락은 0
        for (int finger = 0; finger < finger_count_; finger++) {
          std::vector<uint8_t> cells(15);
          size_t offset = sensor_base + finger * bytes_per_finger;
          for (int j = 0; j < 15; j++) {
            cells[j] = response[offset + j];
          }
          received_data.tactile_m.push_back(std::move(cells));
        }
        break;
      }
      case SensorType::TACTILE_S: {
        // Tactile S: 36 bytes/finger (3×6, uint16 Big-Endian)
        // 전체 슬롯 전송, 비활성 손가락은 0
        for (int finger = 0; finger < finger_count_; finger++) {
          std::vector<uint16_t> cells(18);
          size_t offset = sensor_base + finger * bytes_per_finger;
          for (int j = 0; j < 18; j++) {
            cells[j] = (static_cast<uint16_t>(response[offset + j * 2]) << 8) |
                        response[offset + j * 2 + 1];
          }
          received_data.tactile_s.push_back(std::move(cells));
        }
        break;
      }
      default:
        break;
    }
  }

  // Parse GPIO data (only for models with GPIO enabled)
  if (SupportsExtendedFeatures() && io_) {
    size_t gpio_base = HEADER_SIZE + motor_count_ * byte_per_motor_;
    if (fingertip_sensor_ && sensor_type_ != SensorType::NONE) {
      gpio_base += GetSensorBytesPerFinger() * GetSensorFingerCount();
    }

    received_data.gpio.resize(4);
    for (int i = 0; i < 4; i++) {
      received_data.gpio[i] = (response[gpio_base + i] != 0);
    }
  }

  return received_data;
}

void Communication::SendDuty(std::vector<int>& duty) {
  std::vector<uint8_t> tcp_data_send;

  uint16_t total_packet_size = 3 + motor_count_ * 3;  // Header + ID(1) + Duty(2) per motor
  tcp_data_send.resize(total_packet_size);

  tcp_data_send[0] = (total_packet_size >> 8) & 0xFF;  // Length_h
  tcp_data_send[1] = (total_packet_size) & 0xFF;       // Length_l
  tcp_data_send[2] = SET_DUTY_CMD;                     // CMD

  for (int i = 0; i < motor_count_; ++i) {
    tcp_data_send[3 + i * 3] = i + 1;                  // ID
    tcp_data_send[4 + i * 3] = (duty[i] >> 8) & 0xFF;  // High byte
    tcp_data_send[5 + i * 3] = (duty[i]) & 0xFF;       // Low byte
  }

  boost::system::error_code ec;
  socket_.write_some(boost::asio::buffer(tcp_data_send), ec);

  if (ec) {
    std::cerr << "Error sending duty: " << ec.message() << std::endl;

    // Try to reconnect on connection errors
    if (ec == boost::asio::error::broken_pipe ||
        ec == boost::asio::error::connection_reset ||
        ec == boost::asio::error::connection_aborted) {
      std::cerr << "Connection lost, attempting to reconnect..." << std::endl;
      try {
        socket_.close();
        Connect();
        socket_.write_some(boost::asio::buffer(tcp_data_send), ec);
        if (ec) {
          std::cerr << "Failed to send after reconnection: " << ec.message()
                    << std::endl;
        }
      } catch (const std::exception& e) {
        std::cerr << "Reconnection failed: " << e.what() << std::endl;
      }
    }
  }
}

// ============================================================================
// GPIO Control
// ============================================================================

void Communication::SetGPIO(bool output1, bool output2, bool output3) {
  if (!SupportsExtendedFeatures()) {
    std::cerr << "GPIO not supported on this model" << std::endl;
    return;
  }

  std::array<uint8_t, 6> request;
  request[0] = 0x00;                       // Length_h
  request[1] = 0x06;                       // Length_l
  request[2] = SET_GPIO_CMD;               // CMD 0x06
  request[3] = output1 ? 0x01 : 0x00;      // Output 1
  request[4] = output2 ? 0x01 : 0x00;      // Output 2
  request[5] = output3 ? 0x01 : 0x00;      // Output 3

  boost::system::error_code ec;
  socket_.write_some(boost::asio::buffer(request), ec);

  if (ec) {
    std::cerr << "Error sending GPIO command: " << ec.message() << std::endl;
  }
}

// ============================================================================
// F/T Sensor Control
// ============================================================================

void Communication::SetFTSensorOffset() {
  if (!SupportsExtendedFeatures()) {
    std::cerr << "F/T sensor not supported on this model" << std::endl;
    return;
  }

  std::array<uint8_t, 3> request;
  request[0] = 0x00;                       // Length_h
  request[1] = 0x03;                       // Length_l
  request[2] = SET_FT_SENSOR_OFFSET_CMD;   // CMD 0x0B

  boost::system::error_code ec;
  socket_.write_some(boost::asio::buffer(request), ec);

  if (ec) {
    std::cerr << "Error sending F/T sensor offset command: " << ec.message()
              << std::endl;
  }
}

// ============================================================================
// Version Info
// ============================================================================

std::vector<uint8_t> Communication::GetFirmwareVersion() {
  return firmware_version_;
}

}  // namespace DeltoTCP
