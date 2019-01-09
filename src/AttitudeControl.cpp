#include <AttitudeControl.hpp>

namespace mc_att_control {

  AttitudeControl::AttitudeControl() {
    /* initialize quaternions in messages to be valid */
    _v_att.w() = 1.f;
    _v_att_sp.w() = 1.f;

    _rates_prev.fill(0);
    _rates_prev_filtered.fill(0);
    _rates_sp.fill(0);
    _rates_int.fill(0);
    _thrust_sp = 0.0f;
    _yawspeed_sp = 0.0f;
    _att_control.fill(0);

    /* roll gains */
    _attitude_p(0) = MC_ROLL_P;
    _rate_p(0) = MC_ROLLRATE_P;
    _rate_i(0) = MC_ROLLRATE_I;
    _rate_int_lim(0) = MC_RR_INT_LIM;
    _rate_d(0) = MC_ROLLRATE_D;
    _rate_ff(0) = MC_ROLLRATE_FF;

    /* pitch gains */
    _attitude_p(1) = MC_PITCH_P;
    _rate_p(1) = MC_PITCHRATE_P;
    _rate_i(1) = MC_PITCHRATE_I;
    _rate_int_lim(1) = MC_PR_INT_LIM;
    _rate_d(1) = MC_PITCHRATE_D;
    _rate_ff(1) = MC_PITCHRATE_FF;

    /* yaw gains */
    _attitude_p(2) = MC_YAW_P;
    _rate_p(2) = MC_YAWRATE_P;
    _rate_i(2) = MC_YAWRATE_I;
    _rate_int_lim(2) = MC_YR_INT_LIM;
    _rate_d(2) = MC_YAWRATE_D;
    _rate_ff(2) = MC_YAWRATE_FF;

    /* angular rate limits */
    _mc_rate_max(0) = radians(MC_ROLLRATE_MAX);
    _mc_rate_max(1) = radians(MC_PITCHRATE_MAX);
    _mc_rate_max(2) = radians(MC_YAWRATE_MAX);

    _man_tilt_max = radians(MPC_MAN_TILT_MAX);
    _yaw_rate_scaling = radians(MPC_MAN_Y_MAX);
  }

  void AttitudeControl::updateSticks(const mavros_msgs::ManualControlConstPtr& control) {
    _manual_control_sp.x = control->x;
    _manual_control_sp.y = control->y;
    _manual_control_sp.z = control->z;
    _manual_control_sp.r = control->r;
  }

  void AttitudeControl::updateState(const AttitudeControlStates &states) {
    // transform the vehicle orientation from the ENU to the NED frame
    // q_nb is the quaternion that represents the orientation of the vehicle
    // the NED earth/local
    _v_att = q_ng * states.attitude * q_rb;
    _rates = q_rb.toRotationMatrix() * states.rates;
    manual_mode = states.manual_mode;
    if(!manual_mode) {
      _v_att_sp = q_ng * states.attitude_sp * q_rb;
      _thrust_sp = states.thrust_sp;
      _yawspeed_sp = -states.yawspeed_sp;
    }
  }

  void AttitudeControl::generate_attitude_setpoint(scalar_t dt, bool reset_yaw_sp) {
    vec3 euler321 = dcm2vec(quat2dcm(_v_att));
    const scalar_t yaw = euler321(2);

    /* reset yaw setpoint to current position if needed */
    if (reset_yaw_sp) {
      _man_yaw_sp = yaw;

    } else if (_manual_control_sp.z > 0.05f) {

      _man_yaw_sp = wrap_pi(_man_yaw_sp + _manual_control_sp.r * _yaw_rate_scaling * dt);
    }

    /*
     * Input mapping for roll & pitch setpoints
     * ----------------------------------------
     * We control the following 2 angles:
     * - tilt angle, given by sqrt(x*x + y*y)
     * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
     *
     * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
     * points to, and changes of the stick input are linear.
     */
    const scalar_t x = _manual_control_sp.x * _man_tilt_max;
    const scalar_t y = _manual_control_sp.y * _man_tilt_max;

    // we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
    vec2 v = vec2(y, -x);
    scalar_t v_norm = v.norm(); // the norm of v defines the tilt angle

    if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
      v *= _man_tilt_max / v_norm;
    }

    quat q_sp_rpy = from_axis_angle(vec3(v(0), v(1), 0.f));
    vec3 euler_sp = dcm2vec(quat2dcm(q_sp_rpy));
    euler_sp(2) += _man_yaw_sp;

    /* copy quaternion setpoint to attitude setpoint topic */
    _v_att_sp = from_euler(euler_sp);

    // physical thrust axis is the negative of body z axis
    _thrust_sp = throttle_curve(_manual_control_sp.z);
  }

  scalar_t AttitudeControl::throttle_curve(scalar_t throttle_stick_input) {
    // throttle_stick_input is in range [0, 1]
    switch (MPC_THR_CURVE) {
      case 1: { // no rescaling to hover throttle 
        return MPC_MANTHR_MIN + throttle_stick_input * (MPC_THR_MAX - MPC_MANTHR_MIN);
      }
      default: { // 0 or other: rescale to hover throttle at 0.5 stick
        if (throttle_stick_input < 0.5f) {
          return (MPC_THR_HOVER - MPC_MANTHR_MIN) / 0.5f * throttle_stick_input + MPC_MANTHR_MIN;
        } else {
          return (MPC_THR_MAX - MPC_THR_HOVER) / 0.5f * (throttle_stick_input - 1.0f) + MPC_THR_MAX;
        }
      }
    }
  }

  /**
   * Attitude controller.
   * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
   * Output: '_rates_sp' vector, '_thrust_sp'
   */
  void AttitudeControl::control_attitude() {

    /* prepare yaw weight from the ratio between roll/pitch and yaw gains */
    vec3 attitude_gain = _attitude_p;
    const scalar_t roll_pitch_gain = (attitude_gain(0) + attitude_gain(1)) / 2.0;
    attitude_gain(2) = roll_pitch_gain;

    /* get estimated and desired vehicle attitude */
    quat q = _v_att;
    quat qd = _v_att_sp;

    /* ensure input quaternions are exactly normalized because acosf(1.00001) == NaN */
    q.normalize();
    qd.normalize();

    /* quaternion attitude control law, qe is rotation from q to qd */
    quat qe = q.inverse() * qd;

    /* using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
     * also taking care of the antipodal unit quaternion ambiguity */
    vec3 eq = 2.f * signNoZero(qe.w()) * qe.vec();

    /* calculate angular rates setpoint */
    _rates_sp = eq.cwiseProduct(attitude_gain);

    /* Feed forward the yaw setpoint rate.
     * yaw_sp_move_rate is the feed forward commanded rotation around the world z-axis,
     * but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
     * Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
     * and multiply it by the yaw setpoint rate (yaw_sp_move_rate).
     * This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
     * such that it can be added to the rates setpoint.
     */
    _rates_sp += q.inverse().toRotationMatrix().col(2) * _yawspeed_sp;

    /* limit rates */
    for (int i = 0; i < 3; i++)
      _rates_sp(i) = constrain(_rates_sp(i), -_mc_rate_max(i), _mc_rate_max(i));
  }

  /*
   * Attitude rates controller.
   * Input: '_rates_sp' vector, '_thrust_sp'
   * Output: '_att_control' vector
   */
  void AttitudeControl::control_attitude_rates(scalar_t dt) {
    /* angular rates error */
    vec3 rates_err = _rates_sp - _rates;

    /* apply low-pass filtering to the rates for D-term */
    vec3 rates_filtered(_lp_filters_d.apply(_rates));

    _att_control = _rate_p.cwiseProduct(rates_err) - _rate_d.cwiseProduct(rates_filtered - _rates_prev_filtered) / dt + _rate_ff.cwiseProduct(_rates_sp);

    _rates_prev = _rates;
    _rates_prev_filtered = rates_filtered;
  }

  void AttitudeControl::run(const scalar_t dt) {
    if(manual_mode) {
      bool reset_yaw_sp = !in_air;
      generate_attitude_setpoint(dt, reset_yaw_sp);
    }
    control_attitude();
    control_attitude_rates(dt);
  }

  vec3 AttitudeControl::getAttitudeControls() {
    return _att_control;
  }

  scalar_t AttitudeControl::getThrust() {
    return _thrust_sp;
  }

  void AttitudeControl::updateLandedState(uint8_t landed_state) {
    in_air = landed_state;
  }
}