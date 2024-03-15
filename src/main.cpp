/// @file
/// @brief Main program body
/// @copyright Copyright (c) 2024 Yoshikawa Teru
/// @license This project is released under the MIT License.
#include <mbed.h>

#include "C620.hpp"
#include "VelPid.hpp"

// Const variable
constexpr int target_rpm = 2000;
constexpr float output_limit = 0.5;
constexpr PidGain pid_gain{};
constexpr auto loop_period = 10ms;

// IO
CAN can1{PA_11, PA_12, (int)1e6};
// CAN can2{PB_12, PB_13, (int)1e6};

// Global variable
VelPid pid{{pid_gain, -output_limit, output_limit}};
C620Array c620;

/// @brief The application entry point.
int main() {
  // put your setup code here, to run once:
  printf("\nsetup\n");
  while(1) {
    // put your main code here, to run repeatedly:
    auto now = HighResClock::now();
    static auto pre = now;
    auto elapsed = now - pre;
    if(elapsed > loop_period) {
      // rpmの読み取り
      if(CANMessage msg; can1.read(msg)) {
        c620.parse_packet(msg);
      }

      // PIDで操作量を計算
      auto actual_rpm = c620[0].get_rpm();
      float output = pid.calc(target_rpm, actual_rpm, elapsed);
      c620[0].set_current(output);
      printf("\noutput: %d", c620[0].get_raw_current());

      // 操作量を送信
      const auto c620_msgs = c620.to_msgs();
      if(!can1.write(c620_msgs[0]) || !can1.write(c620_msgs[1])) {
        printf("\tfailed to write c620 msg");
      }

      pre = now;
    }
  }
}
