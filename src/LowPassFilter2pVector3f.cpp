#include <LowPassFilter2pVector3f.hpp>
#include <cmath>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#ifndef M_PI_2_F
#define M_PI_2_F (M_PI / 2.0f)
#endif

namespace mc_att_control {

void LowPassFilter2pVector3f::set_cutoff_frequency(scalar_t sample_freq, scalar_t cutoff_freq) {
  _cutoff_freq = cutoff_freq;

  // reset delay elements on filter change
  _delay_element_1.fill(0);
  _delay_element_2.fill(0);

  if (_cutoff_freq <= 0.0f) {
    // no filtering
    _b0 = 1.0f;
    _b1 = 0.0f;
    _b2 = 0.0f;

    _a1 = 0.0f;
    _a2 = 0.0f;

    return;
  }

  const scalar_t fr = sample_freq / _cutoff_freq;
  const scalar_t ohm = tanf(M_PI_F / fr);
  const scalar_t c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

  _b0 = ohm * ohm / c;
  _b1 = 2.0f * _b0;
  _b2 = _b0;

  _a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  _a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

vec3 LowPassFilter2pVector3f::reset(const vec3 &sample) {
  const vec3 dval = sample / (_b0 + _b1 + _b2);

  if (PX4_ISFINITE(dval(0)) && PX4_ISFINITE(dval(1)) && PX4_ISFINITE(dval(2))) {
    _delay_element_1 = dval;
    _delay_element_2 = dval;

  } else {
   _delay_element_1 = sample;
   _delay_element_2 = sample;
  }

  return apply(sample);
}

} // namespace mc_att_control
