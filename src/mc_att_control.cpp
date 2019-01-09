#include <Node.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mc_att_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  mc_att_control::Node node(nh, pnh);
  ros::spin();
  return 0;
}