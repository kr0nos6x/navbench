#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace navbench {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;

inline bool finite(float value) {
  return std::isfinite(value);
}

inline float clamp(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

inline float square(float value) {
  return value * value;
}

inline float normalize_angle(float angle_rad) {
  if (!finite(angle_rad)) {
    return angle_rad;
  }
  angle_rad = std::fmod(angle_rad + kPi, kTwoPi);
  if (angle_rad < 0.0F) {
    angle_rad += kTwoPi;
  }
  return angle_rad - kPi;
}

inline uint32_t elapsed_ms(uint32_t now_ms, uint32_t then_ms) {
  return now_ms - then_ms;
}

inline bool invert_symmetric_2x2(const float matrix[4], float inverse[4],
                                 float minimum_determinant = 1.0e-12F) {
  if (!finite(matrix[0]) || !finite(matrix[1]) || !finite(matrix[2]) ||
      !finite(matrix[3])) {
    return false;
  }

  const float determinant = matrix[0] * matrix[3] - matrix[1] * matrix[2];
  if (!finite(determinant) || determinant <= minimum_determinant) {
    return false;
  }

  const float reciprocal = 1.0F / determinant;
  inverse[0] = matrix[3] * reciprocal;
  inverse[1] = -matrix[1] * reciprocal;
  inverse[2] = -matrix[2] * reciprocal;
  inverse[3] = matrix[0] * reciprocal;
  return finite(inverse[0]) && finite(inverse[1]) && finite(inverse[2]) &&
         finite(inverse[3]);
}

template <typename T, std::size_t Capacity>
class FixedQueue {
 public:
  static_assert(Capacity > 0U, "FixedQueue capacity must be positive");

  bool push(const T& value) {
    if (size_ == Capacity) {
      ++overflow_count_;
      return false;
    }
    storage_[tail_] = value;
    tail_ = (tail_ + 1U) % Capacity;
    ++size_;
    return true;
  }

  bool pop(T& value) {
    if (size_ == 0U) {
      return false;
    }
    value = storage_[head_];
    head_ = (head_ + 1U) % Capacity;
    --size_;
    return true;
  }

  bool peek(T& value) const {
    if (size_ == 0U) {
      return false;
    }
    value = storage_[head_];
    return true;
  }

  void clear() {
    head_ = 0U;
    tail_ = 0U;
    size_ = 0U;
  }

  std::size_t size() const { return size_; }
  constexpr std::size_t capacity() const { return Capacity; }
  bool empty() const { return size_ == 0U; }
  bool full() const { return size_ == Capacity; }
  uint32_t overflow_count() const { return overflow_count_; }

 private:
  T storage_[Capacity]{};
  std::size_t head_{0U};
  std::size_t tail_{0U};
  std::size_t size_{0U};
  uint32_t overflow_count_{0U};
};

}  // namespace navbench
