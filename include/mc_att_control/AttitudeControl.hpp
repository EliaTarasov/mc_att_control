#ifndef ATTITUDE_CONTROL_HPP_
#define ATTITUDE_CONTROL_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ActuatorControl.h>
#include <LowPassFilter2pVector3f.hpp>
#include <common.h>

namespace mc_att_control {

  struct AttitudeControlStates {
    bool manual_mode;
    quat attitude_sp;
    scalar_t yawspeed_sp;
    scalar_t thrust_sp;
    quat attitude;
    vec3 rates;
  };

  class AttitudeControl {
  public:
    AttitudeControl();
    ~AttitudeControl() = default;

    void updateSticks(const mavros_msgs::ManualControlConstPtr& control);

    /**
     * Update the current vehicle state.
     * @param AttitudeControlStates structure
     */
    void updateState(const AttitudeControlStates &states);
    void updateLandedState(uint8_t landed_state);
    void run(const scalar_t dt);
    vec3 getAttitudeControls();
    scalar_t getThrust();

  private:
    /**
     * Generate & publish an attitude setpoint from stick inputs
     */
    void generate_attitude_setpoint(scalar_t dt, bool reset_yaw_sp);

    /**
     * Attitude controller.
     */
    void control_attitude();

    /**
     * Attitude rates controller.
     */
    void control_attitude_rates(scalar_t dt);

    scalar_t throttle_curve(scalar_t throttle_stick_input);

    quat _v_att; /**< vehicle attitude */
    quat _v_att_sp; /**< vehicle attitude setpoint */
    vec3 _v_rates_sp; /**< vehicle rates setpoint */
    mavros_msgs::ManualControl _manual_control_sp; /**< manual control setpoint */
    mavros_msgs::ActuatorControl _actuators {}; /**< actuator controls */

    vec3 _rates; /**< angular rates on current step */
    vec3 _rates_prev; /**< angular rates on previous step */
    vec3 _rates_prev_filtered; /**< angular rates on previous step (low-pass filtered) */
    vec3 _rates_sp; /**< angular rates setpoint */
    vec3 _rates_int; /**< angular rates integral error */

    vec3 _att_control; /**< attitude control vector */
    scalar_t _thrust_sp; /**< thrust setpoint */
    scalar_t _yawspeed_sp;
    scalar_t _man_yaw_sp; /**< current yaw setpoint in manual mode */

    vec3 _attitude_p; /**< P gain for attitude control */
    vec3 _rate_p; /**< P gain for angular rate error */
    vec3 _rate_i; /**< I gain for angular rate error */
    vec3 _rate_int_lim; /**< integrator state limit for rate loop */
    vec3 _rate_d; /**< D gain for angular rate error */
    vec3 _rate_ff; /**< Feedforward gain for desired rates */

    vec3 _mc_rate_max; /**< attitude rate limits in stabilized modes */
    scalar_t _man_tilt_max; /**< maximum tilt allowed for manual flight [rad] */
    scalar_t _yaw_rate_scaling; /**< scaling factor from stick to yaw rate */

    LowPassFilter2pVector3f _lp_filters_d{initial_update_rate_hz, 5.f};	/**< low-pass filters for D-term (roll, pitch & yaw) */
    static constexpr const scalar_t initial_update_rate_hz = 50.f; /**< loop update rate used for initialization */
    scalar_t _loop_update_rate_hz{initial_update_rate_hz};          /**< current rate-controller loop update rate in [Hz] */

    scalar_t MC_ROLL_P = 6.0; 
    scalar_t MC_ROLLRATE_P = 0.2;
    scalar_t MC_ROLLRATE_I = 0.05;
    scalar_t MC_RR_INT_LIM = 0.3;
    scalar_t MC_ROLLRATE_D = 0.003;
    scalar_t MC_ROLLRATE_FF = 0.0;
    
    scalar_t MC_PITCH_P = 6.0;
    scalar_t MC_PITCHRATE_P = 0.2;
    scalar_t MC_PITCHRATE_I = 0.05;
    scalar_t MC_PR_INT_LIM = 0.3;
    scalar_t MC_PITCHRATE_D = 0.003;
    scalar_t MC_PITCHRATE_FF = 0.0;
    
    scalar_t MC_YAW_P = 2.8;
    scalar_t MC_YAWRATE_P = 0.2;
    scalar_t MC_YAWRATE_I = 0.1;
    scalar_t MC_YR_INT_LIM = 0.3;
    scalar_t MC_YAWRATE_D = 0.0;
    scalar_t MC_YAWRATE_FF = 0.0;

    scalar_t MC_ROLLRATE_MAX = 220;
    scalar_t MC_PITCHRATE_MAX = 220;
    scalar_t MC_YAWRATE_MAX = 100;
    scalar_t MPC_MAN_Y_MAX = 200; /**< scaling factor from stick to yaw rate */

    /* Stabilized mode params */
    scalar_t MPC_MAN_TILT_MAX = 35; /**< maximum tilt allowed for manual flight */
    scalar_t MPC_MANTHR_MIN = 0.08; /**< minimum throttle for stabilized */
    scalar_t MPC_THR_MAX = 1.0; /**< maximum throttle for stabilized */
    scalar_t MPC_THR_MIN = 0.12; /**< minimum throttle for stabilized */
    scalar_t MPC_THR_HOVER = 0.5; /**< throttle at which vehicle is at hover equilibrium */
    int MPC_THR_CURVE = 0; /**< throttle curve behavior */

    bool in_air;
    bool manual_mode;
  };
}

#endif // MC_ATT_CONTROL_HPP_