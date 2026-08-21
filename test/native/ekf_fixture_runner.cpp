#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "navbench/ekf.hpp"

namespace {

bool parse_expected(std::istringstream& input, int& expected) {
  input >> expected;
  return input.good() || input.eof();
}

void emit(std::size_t line_number, navbench::UpdateInfo result,
          const navbench::Ekf6& ekf, uint32_t now_ms) {
  const navbench::EkfState state = ekf.state();
  float covariance[navbench::kEkfStateSize * navbench::kEkfStateSize]{};
  ekf.covariance(covariance);
  std::cout << line_number << '\t' << static_cast<int>(result.result) << '\t'
            << std::setprecision(9) << result.nis << '\t' << state.x_m << '\t'
            << state.y_m << '\t' << state.heading_rad << '\t' << state.speed_mps
            << '\t' << state.yaw_rate_rad_s << '\t' << state.accel_bias_mps2;
  for (std::size_t index = 0U; index < navbench::kEkfStateSize; ++index) {
    std::cout << '\t'
              << covariance[index * navbench::kEkfStateSize + index];
  }
  std::cout << '\t' << static_cast<int>(ekf.navigation_mode(now_ms)) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: ekf_fixture_runner FIXTURE.tsv\n";
    return 2;
  }
  std::ifstream fixture(argv[1]);
  if (!fixture.good()) {
    std::cerr << "cannot open fixture\n";
    return 2;
  }

  navbench::Ekf6 ekf;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(fixture, line)) {
    ++line_number;
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream input(line);
    std::string operation;
    uint32_t timestamp_ms = 0U;
    input >> operation >> timestamp_ms;
    navbench::UpdateInfo result{};
    int expected = -1;
    if (operation == "INIT") {
      navbench::EkfState state{};
      input >> state.x_m >> state.y_m >> state.heading_rad >> state.speed_mps >>
          state.yaw_rate_rad_s >> state.accel_bias_mps2;
      if (!parse_expected(input, expected)) return 2;
      result = navbench::UpdateInfo(
          ekf.initialize(state, timestamp_ms) ? navbench::UpdateResult::Accepted
                                              : navbench::UpdateResult::InvalidMeasurement,
          0.0F);
    } else if (operation == "IMU") {
      float dt_s = 0.0F;
      navbench::ImuMeasurement measurement{};
      input >> dt_s >> measurement.longitudinal_accel_mps2 >>
          measurement.yaw_rate_rad_s;
      measurement.timestamp_ms = timestamp_ms;
      if (!parse_expected(input, expected)) return 2;
      result = ekf.predict(measurement, dt_s);
    } else if (operation == "WHEEL") {
      navbench::WheelSpeedMeasurement measurement{};
      input >> measurement.speed_mps;
      measurement.timestamp_ms = timestamp_ms;
      if (!parse_expected(input, expected)) return 2;
      result = ekf.update_wheel(measurement);
    } else if (operation == "GNSS") {
      navbench::GnssMeasurement measurement{};
      input >> measurement.x_m >> measurement.y_m;
      measurement.timestamp_ms = timestamp_ms;
      if (!parse_expected(input, expected)) return 2;
      result = ekf.update_gnss(measurement);
    } else if (operation == "LANDMARK") {
      navbench::LandmarkMeasurement measurement{};
      input >> measurement.landmark_x_m >> measurement.landmark_y_m >>
          measurement.range_m >> measurement.bearing_rad;
      measurement.timestamp_ms = timestamp_ms;
      if (!parse_expected(input, expected)) return 2;
      result = ekf.update_landmark(measurement);
    } else if (operation == "MODE") {
      if (!parse_expected(input, expected)) return 2;
      result = navbench::UpdateInfo(navbench::UpdateResult::Accepted, 0.0F);
    } else {
      std::cerr << "unknown operation at line " << line_number << '\n';
      return 2;
    }
    if (!input.eof() || expected < 0 || expected > 4 ||
        static_cast<int>(result.result) != expected) {
      std::cerr << "fixture status mismatch at line " << line_number << '\n';
      return 1;
    }
    emit(line_number, result, ekf, timestamp_ms);
  }
  return 0;
}
