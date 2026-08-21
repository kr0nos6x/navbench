#include <cmath>
#include <cstdio>
#include <limits>

#include "navbench/runtime.hpp"

namespace {

int checks = 0;

#define CHECK(condition)                                                      \
  do {                                                                        \
    ++checks;                                                                 \
    if (!(condition)) {                                                       \
      std::fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, \
                   #condition);                                               \
      return false;                                                           \
    }                                                                         \
  } while (false)

bool close(float lhs, float rhs, float tolerance = 1.0e-6F) {
  return std::fabs(lhs - rhs) <= tolerance;
}

bool test_fixed_queue() {
  navbench::FixedQueue<int, 2U> queue;
  CHECK(queue.empty());
  CHECK(queue.push(11));
  CHECK(queue.push(22));
  CHECK(queue.full());
  CHECK(!queue.push(33));
  CHECK(queue.overflow_count() == 1U);
  int value = 0;
  CHECK(queue.peek(value));
  CHECK(value == 11);
  CHECK(queue.pop(value));
  CHECK(value == 11);
  CHECK(queue.push(33));
  CHECK(queue.pop(value));
  CHECK(value == 22);
  CHECK(queue.pop(value));
  CHECK(value == 33);
  CHECK(!queue.pop(value));
  return true;
}

bool test_scheduler_release_and_timing_counters() {
  navbench::SchedulerConfig config = navbench::SchedulerConfig::defaults();
  config.period_ms[static_cast<std::size_t>(navbench::RuntimeTask::Estimator)] =
      10U;
  config.execution_budget_us[static_cast<std::size_t>(
      navbench::RuntimeTask::Estimator)] = 100U;
  navbench::CooperativeScheduler scheduler(config);
  scheduler.reset(100U);
  navbench::ScheduleDecision decision = scheduler.poll(109U);
  CHECK(!decision.due[static_cast<std::size_t>(
      navbench::RuntimeTask::Estimator)]);
  decision = scheduler.poll(110U);
  CHECK(decision.due[static_cast<std::size_t>(
      navbench::RuntimeTask::Estimator)]);
  decision = scheduler.poll(150U);
  CHECK(decision.due[static_cast<std::size_t>(
      navbench::RuntimeTask::Estimator)]);
  CHECK(scheduler.timing(navbench::RuntimeTask::Estimator).missed_releases ==
        3U);

  scheduler.record_execution(navbench::RuntimeTask::Estimator, 80U);
  scheduler.record_execution(navbench::RuntimeTask::Estimator, 120U);
  const navbench::TaskTiming& timing =
      scheduler.timing(navbench::RuntimeTask::Estimator);
  CHECK(timing.runs == 2U);
  CHECK(timing.overruns == 1U);
  CHECK(timing.maximum_duration_us == 120U);
  CHECK(timing.total_duration_us == 200U);
  return true;
}

bool prepare_runtime(navbench::RuntimeCore& runtime, uint32_t now_ms = 0U) {
  runtime.begin(now_ms);
  CHECK(runtime.state() == navbench::SafetyState::Startup);
  CHECK(runtime.start_self_test());
  CHECK(runtime.state() == navbench::SafetyState::SelfTest);
  CHECK(runtime.complete_self_test(true, now_ms + 1U));
  CHECK(runtime.state() == navbench::SafetyState::Ready);
  runtime.notify_handshake(now_ms + 2U);
  return true;
}

bool test_safety_state_machine_and_watchdog() {
  navbench::RuntimeConfig config = navbench::RuntimeConfig::defaults();
  config.host_timeout_ms = 100U;
  config.unavailable_grace_ms = 50U;
  config.queue_overflow_safe_stop_threshold = 2U;
  navbench::RuntimeCore runtime(config);
  CHECK(prepare_runtime(runtime));

  navbench::RuntimeDecision decision = runtime.tick(
      3U, navbench::NavigationMode::Unavailable, true);
  CHECK(decision.state == navbench::SafetyState::Ready);
  CHECK(decision.output_neutral);

  runtime.accept_input(10U);
  decision = runtime.tick(10U, navbench::NavigationMode::GnssAided, true);
  CHECK(decision.state == navbench::SafetyState::Running);
  CHECK(decision.permit_control);

  runtime.record_rejected_input(navbench::InputDisposition::Corrupt);
  decision = runtime.tick(11U, navbench::NavigationMode::GnssAided, true);
  CHECK(decision.state == navbench::SafetyState::Running);
  CHECK(runtime.stats().corrupt_inputs == 1U);

  decision = runtime.tick(12U, navbench::NavigationMode::DeadReckoning, true);
  CHECK(decision.state == navbench::SafetyState::Degraded);
  CHECK(decision.permit_control);
  decision = runtime.tick(13U, navbench::NavigationMode::LandmarkAided, true);
  CHECK(decision.state == navbench::SafetyState::Running);

  decision = runtime.tick(111U, navbench::NavigationMode::GnssAided, true);
  CHECK(decision.state == navbench::SafetyState::SafeStop);
  CHECK(decision.force_safe_stop);
  CHECK(runtime.stats().watchdog_timeouts == 1U);
  runtime.accept_input(112U);
  decision = runtime.tick(112U, navbench::NavigationMode::GnssAided, true);
  CHECK(decision.state == navbench::SafetyState::SafeStop);
  return true;
}

bool test_unavailable_queue_manual_and_fault_paths() {
  navbench::RuntimeConfig config = navbench::RuntimeConfig::defaults();
  config.host_timeout_ms = 1000U;
  config.unavailable_grace_ms = 50U;
  config.queue_overflow_safe_stop_threshold = 2U;

  navbench::RuntimeCore unavailable(config);
  CHECK(prepare_runtime(unavailable));
  unavailable.accept_input(10U);
  CHECK(unavailable.tick(10U, navbench::NavigationMode::GnssAided, true).state ==
        navbench::SafetyState::Running);
  navbench::RuntimeDecision decision = unavailable.tick(
      20U, navbench::NavigationMode::Unavailable, true);
  CHECK(decision.state == navbench::SafetyState::Degraded);
  CHECK(decision.output_neutral);
  decision = unavailable.tick(70U, navbench::NavigationMode::Unavailable, true);
  CHECK(decision.state == navbench::SafetyState::SafeStop);

  navbench::RuntimeCore overflow(config);
  CHECK(prepare_runtime(overflow));
  overflow.record_queue_overflow();
  CHECK(overflow.state() == navbench::SafetyState::Ready);
  overflow.record_queue_overflow();
  CHECK(overflow.state() == navbench::SafetyState::SafeStop);

  navbench::RuntimeCore manual(config);
  CHECK(prepare_runtime(manual));
  manual.request_safe_stop();
  CHECK(manual.state() == navbench::SafetyState::SafeStop);
  CHECK(manual.stats().manual_safe_stops == 1U);

  navbench::RuntimeCore fault(config);
  CHECK(prepare_runtime(fault));
  fault.report_numerical_fault();
  CHECK(fault.state() == navbench::SafetyState::Fault);
  CHECK(fault.stats().numerical_faults == 1U);
  CHECK(fault.tick(20U, navbench::NavigationMode::GnssAided, true)
            .force_safe_stop);

  navbench::RuntimeCore failed_self_test(config);
  failed_self_test.begin(0U);
  CHECK(failed_self_test.start_self_test());
  CHECK(failed_self_test.complete_self_test(false, 1U));
  CHECK(failed_self_test.state() == navbench::SafetyState::Fault);
  return true;
}

bool test_controller_facade_and_rejected_frame_isolation() {
  navbench::EmbeddedControllerCore core;
  core.begin(0U);
  CHECK(core.start_self_test());
  CHECK(core.complete_self_test(true, 1U));
  core.notify_handshake(2U);
  const navbench::Waypoint route[2] = {
      {0.0F, 0.0F, 1.0F},
      {5.0F, 0.0F, 0.0F},
  };
  CHECK(core.set_route(route, 2U));

  navbench::ControllerStepInput input{};
  input.now_ms = 20U;
  input.dt_s = 0.02F;
  input.has_sensor_frame = true;
  input.sensor_frame.timestamp_ms = 20U;
  input.sensor_frame.step_id = 1U;
  input.sensor_frame.disposition = navbench::InputDisposition::Accepted;
  input.sensor_frame.has_imu = true;
  input.sensor_frame.imu.timestamp_ms = 20U;
  input.sensor_frame.imu.step_id = 1U;
  input.sensor_frame.imu.longitudinal_accel_mps2 = 0.0F;
  input.sensor_frame.imu.yaw_rate_rad_s = 0.0F;
  input.sensor_frame.has_wheel_speed = true;
  input.sensor_frame.wheel_speed.timestamp_ms = 20U;
  input.sensor_frame.wheel_speed.step_id = 1U;
  input.sensor_frame.wheel_speed.speed_mps = 0.0F;
  input.sensor_frame.has_gnss = true;
  input.sensor_frame.gnss.timestamp_ms = 20U;
  input.sensor_frame.gnss.step_id = 1U;
  input.sensor_frame.gnss.x_m = 0.0F;
  input.sensor_frame.gnss.y_m = 0.0F;

  navbench::ControllerStepOutput output = core.step(input);
  CHECK(output.estimator_initialized);
  CHECK(output.estimator_healthy);
  CHECK(output.navigation_mode == navbench::NavigationMode::GnssAided);
  CHECK(output.safety_state == navbench::SafetyState::Running);
  CHECK(output.command.valid);
  CHECK(!output.command.safe_stop);
  CHECK(output.command.acceleration_mps2 > 0.0F);

  const navbench::EkfState estimate_before = output.estimate;
  input.now_ms = 25U;
  input.sensor_frame.timestamp_ms = 25U;
  input.sensor_frame.disposition = navbench::InputDisposition::Accepted;
  input.sensor_frame.imu.longitudinal_accel_mps2 =
      std::numeric_limits<float>::quiet_NaN();
  input.sensor_frame.gnss.x_m = 1000.0F;
  output = core.step(input);
  CHECK(close(output.estimate.x_m, estimate_before.x_m));
  CHECK(close(output.estimate.y_m, estimate_before.y_m));
  CHECK(core.runtime().stats().accepted_inputs == 1U);
  CHECK(core.runtime().stats().invalid_inputs == 1U);

  input.now_ms = 30U;
  input.sensor_frame.timestamp_ms = 10U;
  input.sensor_frame.disposition = navbench::InputDisposition::Stale;
  input.sensor_frame.imu.longitudinal_accel_mps2 = 0.0F;
  input.sensor_frame.gnss.x_m = 1000.0F;
  output = core.step(input);
  CHECK(close(output.estimate.x_m, estimate_before.x_m));
  CHECK(close(output.estimate.y_m, estimate_before.y_m));
  CHECK(core.runtime().stats().accepted_inputs == 1U);
  CHECK(core.runtime().stats().stale_inputs == 1U);
  CHECK(output.safety_state == navbench::SafetyState::Running);

  input.has_sensor_frame = false;
  input.now_ms = 521U;
  output = core.step(input);
  CHECK(output.safety_state == navbench::SafetyState::SafeStop);
  CHECK(output.command.safe_stop);

  navbench::EmbeddedControllerCore manual;
  manual.begin(0U);
  CHECK(manual.start_self_test());
  CHECK(manual.complete_self_test(true, 1U));
  navbench::ControllerStepInput stop{};
  stop.now_ms = 2U;
  stop.dt_s = 0.02F;
  stop.manual_safe_stop = true;
  output = manual.step(stop);
  CHECK(output.safety_state == navbench::SafetyState::SafeStop);
  CHECK(output.command.safe_stop);
  return true;
}

bool test_wire_enum_contract() {
  CHECK(static_cast<uint8_t>(navbench::NavigationMode::Unavailable) == 0U);
  CHECK(static_cast<uint8_t>(navbench::NavigationMode::DeadReckoning) == 1U);
  CHECK(static_cast<uint8_t>(navbench::NavigationMode::LandmarkAided) == 2U);
  CHECK(static_cast<uint8_t>(navbench::NavigationMode::GnssAided) == 3U);
  CHECK(static_cast<uint8_t>(navbench::NavigationMode::Degraded) == 4U);
  CHECK(static_cast<uint8_t>(navbench::SafetyState::Startup) == 0U);
  CHECK(static_cast<uint8_t>(navbench::SafetyState::SelfTest) == 1U);
  CHECK(static_cast<uint8_t>(navbench::SafetyState::Ready) == 2U);
  CHECK(static_cast<uint8_t>(navbench::SafetyState::Running) == 3U);
  CHECK(static_cast<uint8_t>(navbench::SafetyState::Degraded) == 4U);
  CHECK(static_cast<uint8_t>(navbench::SafetyState::SafeStop) == 5U);
  CHECK(static_cast<uint8_t>(navbench::SafetyState::Fault) == 6U);
  return true;
}

}  // namespace

int main() {
  if (!test_fixed_queue() || !test_scheduler_release_and_timing_counters() ||
      !test_safety_state_machine_and_watchdog() ||
      !test_unavailable_queue_manual_and_fault_paths() ||
      !test_controller_facade_and_rejected_frame_isolation() ||
      !test_wire_enum_contract()) {
    return 1;
  }
  std::printf("test_runtime: PASS (%d checks)\n", checks);
  return 0;
}
