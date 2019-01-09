#ifndef COMMON_H_
#define COMMON_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <mavros_msgs/AttitudeTarget.h>
#include <cmath>
#include <eigen_conversions/eigen_msg.h>

#define PX4_ISFINITE(x) std::isfinite(x)
  
namespace mc_att_control {

  typedef double scalar_t;
  typedef Eigen::Matrix<scalar_t, 4, 1> vec4; /// Vector in R4
  typedef Eigen::Matrix<scalar_t, 3, 1> vec3; /// Vector in R3
  typedef Eigen::Matrix<scalar_t, 2, 1> vec2; /// Vector in R2
  typedef Eigen::Matrix<scalar_t, 3, 3> mat3; /// Matrix in R3
  typedef Eigen::Quaternion<scalar_t> quat; /// Member of S4

  struct Constraints {
    scalar_t yawspeed; // in radians/sec
    scalar_t speed_xy; // in meters/sec
    scalar_t speed_up; // in meters/sec
    scalar_t speed_down; // in meters/sec
    scalar_t tilt; // in radians [0, PI]
  };

  /**
    * Quaternion for rotation between ENU and NED frames
    *
    * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
    * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
    */
  const quat q_ng = quat(0, 0.70711, 0.70711, 0);

  /**
    * Quaternion for rotation between body FLU and body FRD frames
    *
    * FLU to FRD: +PI rotation about X(forward)
    * FRD to FLU: -PI rotation about X(forward)
    */
  const quat q_rb = quat(0, 1, 0, 0);

  // Type-safe signum function
  template<typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
  }

  // Type-safe signum function with zero treated as positive
  template<typename T> int signNoZero(T val) {
    return (T(0) <= val) - (val < T(0));
  }

  template<typename Scalar>
  static inline constexpr const Scalar &constrain(const Scalar &val, const Scalar &min_val, const Scalar &max_val) {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
  }

  /*
   * So called exponential curve function implementation.
   * It is essentially a linear combination between a linear and a cubic function.
   * @param value [-1,1] input value to function
   * @param e [0,1] function parameter to set ratio between linear and cubic shape
   * 		0 - pure linear function
   * 		1 - pure cubic function
   * @return result of function output
   */
  template<typename T> const T expo(const T &value, const T &e) {
    T x = constrain(value, (T) - 1, (T) 1);
    T ec = constrain(e, (T) 0, (T) 1);
    return (1 - ec) * x + ec * x * x * x;
  }

  /*
   * So called SuperExpo function implementation.
   * It is a 1/(1-x) function to further shape the rc input curve intuitively.
   * I enhanced it compared to other implementations to keep the scale between [-1,1].
   * @param value [-1,1] input value to function
   * @param e [0,1] function parameter to set ratio between linear and cubic shape (see expo)
   * @param g [0,1) function parameter to set SuperExpo shape
   * 		0 - pure expo function
   * 		0.99 - very strong bent curve, stays zero until maximum stick input
   * @return result of function output
   */
  template<typename T> const T superexpo(const T &value, const T &e, const T &g) {
    T x = constrain(value, (T) - 1, (T) 1);
    T gc = constrain(g, (T) 0, (T) 0.99);
    return expo(x, e) * (1 - gc) / (1 - fabsf(x) * gc);
  }

  /*
   * Deadzone function being linear and continuous outside of the deadzone
   * 1                ------
   *                /
   *             --
   *           /
   * -1 ------
   *        -1 -dz +dz 1
   * @param value [-1,1] input value to function
   * @param dz [0,1) ratio between deazone and complete span
   * 		0 - no deadzone, linear -1 to 1
   * 		0.5 - deadzone is half of the span [-0.5,0.5]
   * 		0.99 - almost entire span is deadzone
   */
  template<typename T> const T deadzone(const T &value, const T &dz) {
    T x = constrain(value, (T) - 1, (T) 1);
    T dzc = constrain(dz, (T) 0, (T) 0.99);
    // Rescale the input such that we get a piecewise linear function that will be continuous with applied deadzone
    T out = (x - sign(x) * dzc) / (1 - dzc);
    // apply the deadzone (values zero around the middle)
    return out * (fabsf(x) > dzc);
  }

  template<typename T> const T expo_deadzone(const T &value, const T &e, const T &dz) {
    return expo(deadzone(value, dz), e);
  }

  template <typename T> inline T max(T val1, T val2) {
    return (val1 > val2) ? val1 : val2;
  }

  template <typename T> inline T min(T val1, T val2) {
    return (val1 < val2) ? val1 : val2;
  }

  template<typename T> constexpr T radians(const T degrees) {
    return degrees * (static_cast<T>(M_PI) / static_cast<T>(180));
  }

  template<typename Type>
  Type wrap_pi(Type x) {
    while (x >= Type(M_PI)) {
      x -= Type(2.0 * M_PI);
    }

    while (x < Type(-M_PI)) {
      x += Type(2.0 * M_PI);
    }
    return x;
  }

  ///< tf functions
  inline quat from_axis_angle(const vec3 &axis, scalar_t theta) {
    quat q;

    if (theta < scalar_t(1e-10)) {
      q.w() = scalar_t(1.0);
      q.x() = q.y() = q.z() = 0;
    }

    scalar_t magnitude = sin(theta / 2.0f);

    q.w() = cos(theta / 2.0f);
    q.x() = axis(0) * magnitude;
    q.y() = axis(1) * magnitude;
    q.z() = axis(2) * magnitude;
    
    return q;
  }

  inline quat from_axis_angle(vec3 vec) {
    quat q;
    scalar_t theta = vec.norm();

    if (theta < scalar_t(1e-10)) {
      q.w() = scalar_t(1.0);
      q.x() = q.y() = q.z() = 0;
      return q;
    }

    vec3 tmp = vec / theta;
    return from_axis_angle(tmp, theta);
  }

  inline vec3 to_axis_angle(const quat& q) {
    scalar_t axis_magnitude = scalar_t(sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z()));
    vec3 vec;
    vec(0) = q.x();
    vec(1) = q.y();
    vec(2) = q.z();

    if (axis_magnitude >= scalar_t(1e-10)) {
      vec = vec / axis_magnitude;
      vec = vec * wrap_pi(scalar_t(2.0) * atan2(axis_magnitude, q.w()));
    }

    return vec;
  }

  inline mat3 quat2dcm(const quat& q) {
    mat3 dcm;
    scalar_t a = q.w();
    scalar_t b = q.x();
    scalar_t c = q.y();
    scalar_t d = q.z();
    scalar_t aSq = a * a;
    scalar_t bSq = b * b;
    scalar_t cSq = c * c;
    scalar_t dSq = d * d;
    dcm(0, 0) = aSq + bSq - cSq - dSq;
    dcm(0, 1) = 2 * (b * c - a * d);
    dcm(0, 2) = 2 * (a * c + b * d);
    dcm(1, 0) = 2 * (b * c + a * d);
    dcm(1, 1) = aSq - bSq + cSq - dSq;
    dcm(1, 2) = 2 * (c * d - a * b);
    dcm(2, 0) = 2 * (b * d - a * c);
    dcm(2, 1) = 2 * (a * b + c * d);
    dcm(2, 2) = aSq - bSq - cSq + dSq;
    return dcm;
  }

  inline vec3 dcm2vec(const mat3& dcm) {
    scalar_t phi_val = atan2(dcm(2, 1), dcm(2, 2));
    scalar_t theta_val = asin(-dcm(2, 0));
    scalar_t psi_val = atan2(dcm(1, 0), dcm(0, 0));
    scalar_t pi = M_PI;

    if (fabs(theta_val - pi / 2) < 1.0e-3) {
      phi_val = 0.0;
      psi_val = atan2(dcm(1, 2), dcm(0, 2));
    } else if (fabs(theta_val + pi / 2) < 1.0e-3) {
      phi_val = 0.0;
      psi_val = atan2(-dcm(1, 2), -dcm(0, 2));
    }
    return vec3(phi_val, theta_val, psi_val);
  }

  // calculate the inverse rotation matrix from a quaternion rotation
  inline mat3 quat_to_invrotmat(const quat &q) {
    scalar_t q00 = q.w() * q.w();
    scalar_t q11 = q.x() * q.x();
    scalar_t q22 = q.y() * q.y();
    scalar_t q33 = q.z() * q.z();
    scalar_t q01 = q.w() * q.x();
    scalar_t q02 = q.w() * q.y();
    scalar_t q03 = q.w() * q.z();
    scalar_t q12 = q.x() * q.y();
    scalar_t q13 = q.x() * q.z();
    scalar_t q23 = q.y() * q.z();

    mat3 dcm;
    dcm(0, 0) = q00 + q11 - q22 - q33;
    dcm(1, 1) = q00 - q11 + q22 - q33;
    dcm(2, 2) = q00 - q11 - q22 + q33;
    dcm(0, 1) = 2.0f * (q12 - q03);
    dcm(0, 2) = 2.0f * (q13 + q02);
    dcm(1, 0) = 2.0f * (q12 + q03);
    dcm(1, 2) = 2.0f * (q23 - q01);
    dcm(2, 0) = 2.0f * (q13 - q02);
    dcm(2, 1) = 2.0f * (q23 + q01);
    
    return dcm;
  }

  /**
   * Constructor from euler angles
   *
   * This sets the instance to a quaternion representing coordinate transformation from
   * frame 2 to frame 1 where the rotation from frame 1 to frame 2 is described
   * by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
   *
   * @param euler euler angle instance
   */
  inline quat from_euler(const vec3& euler) {
    quat q;
    scalar_t cosPhi_2 = cos(euler(0) / 2.0);
    scalar_t cosTheta_2 = cos(euler(1) / 2.0);
    scalar_t cosPsi_2 = cos(euler(2) / 2.0);
    scalar_t sinPhi_2 = sin(euler(0) / 2.0);
    scalar_t sinTheta_2 = sin(euler(1) / 2.0);
    scalar_t sinPsi_2 = sin(euler(2) / 2.0);
    q.w() = cosPhi_2 * cosTheta_2 * cosPsi_2 +
           sinPhi_2 * sinTheta_2 * sinPsi_2;
    q.x() = sinPhi_2 * cosTheta_2 * cosPsi_2 -
           cosPhi_2 * sinTheta_2 * sinPsi_2;
    q.y() = cosPhi_2 * sinTheta_2 * cosPsi_2 +
           sinPhi_2 * cosTheta_2 * sinPsi_2;
    q.z() = cosPhi_2 * cosTheta_2 * sinPsi_2 -
           sinPhi_2 * sinTheta_2 * cosPsi_2;
    return q;
  }
}

#endif