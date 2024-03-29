#include "pick_and_place_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place");
  PickAndPlaceNode pick_and_place_node {20.0};
  ros::AsyncSpinner spinner(1);
  spinner.start();

  pick_and_place_node.replace_blocks();

  ros::waitForShutdown();
  return 0;
}