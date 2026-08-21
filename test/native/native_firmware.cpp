// Native Protocol-v1 bridge used by deterministic host CIL tests.
//
// This text/hex shell is test-only. FirmwareSession is the exact fixed-capacity
// application path also built into the Arduino image.

#include <stdint.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "navbench/firmware_session.hpp"

namespace {

class NativeFirmware {
 public:
  NativeFirmware() { reset(0U); }

  void reset(uint32_t now_ms) { session_.reset(now_ms); }

  void feed(uint32_t now_ms, const uint8_t* data, std::size_t size) {
    session_.feed(now_ms, data, size);
    // Match the Arduino loop: receive bytes, then poll the same cooperative
    // scheduler. FirmwareSession::feed itself only parses/enqueues sensors.
    session_.tick(now_ms, session_.last_step_id());
    drain();
  }

  void tick(uint32_t now_ms, uint32_t step_id) {
    session_.tick(now_ms, step_id);
    drain();
  }

 private:
  void drain() {
    uint8_t frame[navbench::protocol::kMaxWireFrameSize]{};
    std::size_t frame_size = 0U;
    while (session_.pop_frame(frame, sizeof(frame), &frame_size)) {
      const std::ios::fmtflags previous = std::cout.flags();
      const char previous_fill = std::cout.fill();
      for (std::size_t index = 0U; index < frame_size; ++index) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned int>(frame[index]);
      }
      std::cout.flags(previous);
      std::cout.fill(previous_fill);
      std::cout << '\n';
    }
  }

  navbench::FirmwareSession session_{};
};

bool decode_hex(const std::string& text, uint8_t* output,
                std::size_t capacity, std::size_t& size) {
  if ((text.size() % 2U) != 0U || text.size() / 2U > capacity) {
    return false;
  }
  size = text.size() / 2U;
  for (std::size_t index = 0U; index < size; ++index) {
    const std::string byte_text = text.substr(index * 2U, 2U);
    char* end = nullptr;
    const unsigned long value = std::strtoul(byte_text.c_str(), &end, 16);
    if (end == nullptr || *end != '\0' || value > 255UL) {
      return false;
    }
    output[index] = static_cast<uint8_t>(value);
  }
  return true;
}

}  // namespace

int main() {
  NativeFirmware firmware;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line == "QUIT") {
      return 0;
    }
    std::istringstream input(line);
    std::string command;
    uint32_t now_ms = 0U;
    input >> command >> now_ms;
    if (command == "RESET") {
      firmware.reset(now_ms);
    } else if (command == "TICK") {
      uint32_t step_id = 0U;
      input >> step_id;
      firmware.tick(now_ms, step_id);
    } else if (command == "FEED") {
      std::string hex;
      input >> hex;
      uint8_t bytes[4096]{};
      std::size_t size = 0U;
      if (!decode_hex(hex, bytes, sizeof(bytes), size)) {
        std::cout << "ERROR invalid_hex\n";
      } else {
        firmware.feed(now_ms, bytes, size);
      }
    } else {
      std::cout << "ERROR invalid_command\n";
    }
    std::cout << ".\n" << std::flush;
  }
  return 0;
}
