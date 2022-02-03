#include "pick_and_place_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place");
  PickAndPlaceNode pick_and_place_node {20.0};
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(3.0).sleep();
  std::cerr << "Start Pick And Place\n";
  // pick_and_place_node.prepare();

  pick_and_place_node.replace_blocks();

  // ros::WallDuration(1.0).sleep();

  ros::waitForShutdown();
  return 0;
}