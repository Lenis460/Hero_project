#include "c620.h"

#include "config.h"

namespace c620 {

namespace {

inline int16_t BeI16(const uint8_t *p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

} // namespace

bool OnFrame(uint8_t esc_id, const uint8_t *d, Feedback fb_out[4]) {
  for (uint8_t w = 0; w < kWheelCount; ++w) {
    if (cfg::kWheelCanId[w] == esc_id) {
      Feedback &f = fb_out[w];
      f.rotor_rpm = static_cast<float>(BeI16(d));
      f.torque_raw = BeI16(d + 2);
      f.angle_raw = static_cast<uint16_t>((static_cast<uint16_t>(d[4]) << 8) | d[5]);
      f.temp_c = d[6];
      return true;
    }
  }
  return false;
}

float RpmToWheelOmega(float rpm) {
  return rpm * (2.0f * cfg::kPi / 60.0f) / cfg::kWheelGearRatio;
}

} // namespace c620
