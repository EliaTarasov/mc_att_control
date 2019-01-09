#ifndef LOW_PASS_FILTER
#define LOW_PASS_FILTER

#include <common.h>

namespace mc_att_control { 
  
class LowPassFilter2pVector3f {
public:
  LowPassFilter2pVector3f(scalar_t sample_freq, scalar_t cutoff_freq) {
    // set initial parameters
    set_cutoff_frequency(sample_freq, cutoff_freq);
  }

  // Change filter parameters
  void set_cutoff_frequency(scalar_t sample_freq, scalar_t cutoff_freq);

  /**
   * Add a new raw value to the filter
   *
   * @return retrieve the filtered result
   */
  inline vec3 apply(const vec3 &sample) {
    // do the filtering
    const vec3 delay_element_0{sample - _delay_element_1 *_a1 - _delay_element_2 * _a2};
    const vec3 output{delay_element_0 *_b0 + _delay_element_1 *_b1 + _delay_element_2 * _b2};

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
  }

  // Return the cutoff frequency
  scalar_t get_cutoff_freq() const { return _cutoff_freq; }

  // Reset the filter state to this value
  vec3 reset(const vec3 &sample);

private:

  scalar_t _cutoff_freq{0.0f};

  scalar_t _a1{0.0f};
  scalar_t _a2{0.0f};

  scalar_t _b0{0.0f};
  scalar_t  _b1{0.0f};
  scalar_t  _b2{0.0f};

  vec3 _delay_element_1{0.0f, 0.0f, 0.0f};	// buffered sample -1
  vec3 _delay_element_2{0.0f, 0.0f, 0.0f};	// buffered sample -2
};

} // namespace mc_att_control

#endif
