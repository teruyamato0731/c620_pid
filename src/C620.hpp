#ifndef RCT_C620_HPP
#define RCT_C620_HPP
/// @file
/// @brief Provides the C620 class for controlling the motor driver for M3508.
/// @copyright Copyright (c) 2024 Yoshikawa Teru
/// @license This project is released under the MIT License.

#include <mbed.h>

#include <array>

/// @brief The packet structure of the C620 motor driver.
struct C620Packet {
  uint16_t angle;
  int16_t rpm;
  int16_t ampere;
  uint8_t temp;
};

/// @brief The C620 motor driver class for M3508.
struct C620 {
  static constexpr int max = 16384;

  void set_current(float current) {
    raw_current_ = current * max;
  }
  uint16_t get_angle() {
    return rx_.angle;
  }
  int16_t get_rpm() {
    return rx_.rpm;
  }
  int16_t get_ampere() {
    return rx_.ampere;
  }
  uint8_t get_temp() {
    return rx_.temp;
  }
  int16_t get_raw_current() const {
    return raw_current_;
  }
  void parse(const uint8_t data[8]) {
    rx_.angle = uint16_t(data[0] << 8 | data[1]);
    rx_.rpm = int16_t(data[2] << 8 | data[3]);
    rx_.ampere = int16_t(data[4] << 8 | data[5]);
    rx_.temp = data[6];
  }

 private:
  int16_t raw_current_ = {};
  C620Packet rx_ = {};
};

/// @brief The C620 motor driver array for M3508.
struct C620Array {
  void parse_packet(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == 8 && 0x200 <= msg.id && msg.id <= 0x208) {
      arr_[msg.id - 0x201u].parse(msg.data);
    }
  }
  auto to_msgs() -> std::array<CANMessage, 2> const {
    uint8_t buf[16];
    for(int i = 0; i < 8; i++) {
      buf[i] = arr_[i].get_raw_current() >> 8;
      buf[i + 1] = arr_[i].get_raw_current() & 0xff;
    }
    return {CANMessage{0x200, buf}, CANMessage{0x1FF, buf + 8}};
  }
  auto& operator[](int index) {
    return arr_[index];
  }
  auto begin() {
    return std::begin(arr_);
  }
  auto end() {
    return std::end(arr_);
  }

 private:
  C620 arr_[8] = {};
};

#endif  /// RCT_C620_HPP
