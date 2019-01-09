#ifndef MC_ATT_CONTROL_NODE_HPP_
#define MC_ATT_CONTROL_NODE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/State.h>
#include <message_filters/subscriber.h>
#include <AttitudeControl.hpp>

namespace mc_att_control {

  class Node {
  public:
    static constexpr int default_publish_rate_ = 100;

    Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  private:
    ros::NodeHandle nh_;

    // subscribers
    ros::Subscriber subImu_;
    ros::Subscriber subExtendedState_;
    ros::Subscriber subState_;
    ros::Subscriber subManualControl_;
    ros::Subscriber subAttitudeTarget_;

    // timestamps
    ros::Time prevStamp_;

    // publishers
    ros::Publisher pubActuatorControls_;
    // implementation
    mc_att_control::AttitudeControl attControl_;
    mc_att_control::AttitudeControlStates states_;
    ros::Timer pubTimer_;
    bool init_;
    bool armed_;

    //  callbacks
    void imuCallback(const sensor_msgs::ImuConstPtr&);
    void extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr&);
    void attitudeTargetCallback(const mavros_msgs::AttitudeTargetConstPtr&);
    void publishState(const ros::TimerEvent&);
    void stateCallback(const mavros_msgs::StateConstPtr&);
    void manualControlCallback(const mavros_msgs::ManualControl::ConstPtr&);
  };
} //  namespace mc_att_control

#endif // MC_ATT_CONTROL_NODE_HPP_