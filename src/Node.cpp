#include <Node.hpp>
#include <eigen_conversions/eigen_msg.h>

namespace mc_att_control
{

Node::Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : nh_(pnh), init_(false) {
  ROS_INFO("Subscribing to imu");
  subImu_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1000, &Node::imuCallback, this, ros::TransportHints().tcpNoDelay(true));

  ROS_INFO("Subscribing to extended state");
  subExtendedState_ = nh_.subscribe("/mavros/extended_state", 1, &Node::extendedStateCallback, this);

  ROS_INFO("Subscribing to state");
  subState_ = nh_.subscribe("/mavros/state", 1, &Node::stateCallback, this);

  ROS_INFO("Subscribing to manual control");
  subManualControl_ = nh_.subscribe<mavros_msgs::ManualControl>("/mavros/manual_control/control", 1, &Node::manualControlCallback, this);

  ROS_INFO("Subscribing to attitude target");
  subAttitudeTarget_ = nh_.subscribe<mavros_msgs::AttitudeTarget>("/mc_pos_control/attitude_target", 1, &Node::attitudeTargetCallback, this);

  pubActuatorControls_ = nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);
  int publish_rate = default_publish_rate_;
  ros::param::get("~publish_rate", publish_rate);
  pubTimer_ = nh_.createTimer(ros::Duration(1.0f/publish_rate), &Node::publishState, this);

  states_.manual_mode = true;
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr& imu) {
  tf::quaternionMsgToEigen(imu->orientation, states_.attitude);
  tf::vectorMsgToEigen(imu->angular_velocity, states_.rates);
}

void Node::extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr& extendedStateMsg) {
  attControl_.updateLandedState(extendedStateMsg->landed_state & mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR);
}

void Node::attitudeTargetCallback(const mavros_msgs::AttitudeTargetConstPtr& attitudeTarget) {
  tf::quaternionMsgToEigen(attitudeTarget->orientation, states_.attitude_sp);
  states_.thrust_sp = attitudeTarget->thrust;
  states_.yawspeed_sp = attitudeTarget->body_rate.z;
  states_.manual_mode = false;
}

void Node::stateCallback(const mavros_msgs::StateConstPtr& stateMsg) {
  armed_ = stateMsg->armed;
}

void Node::manualControlCallback(const mavros_msgs::ManualControl::ConstPtr& manualControl) {
  if(armed_ && states_.manual_mode)
    attControl_.updateSticks(manualControl);
}

void Node::publishState(const ros::TimerEvent&) {
  ros::Time now = ros::Time::now();

  if (prevStamp_.sec != 0) {
    const double delta = (now - prevStamp_).toSec();
    if(armed_) {
      attControl_.updateState(states_);
      attControl_.run(delta);
    }
  }
  prevStamp_ = now;

  static size_t trace_id = 0;
  std_msgs::Header header;
  header.frame_id = "/actuator_control";
  header.seq = trace_id++;
  header.stamp = ros::Time::now();
  mavros_msgs::ActuatorControl cmd;
  cmd.header = header;
  vec3 controls = attControl_.getAttitudeControls();
  scalar_t thrust = attControl_.getThrust();
  cmd.controls[0] = controls[0];
  cmd.controls[1] = controls[1];
  cmd.controls[2] = controls[2];
  cmd.controls[3] = thrust;
  pubActuatorControls_.publish(cmd);
}

}