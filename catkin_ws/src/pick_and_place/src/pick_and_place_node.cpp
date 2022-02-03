#include"pick_and_place_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/console.h>

#include <string>
// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
constexpr auto Z_OFFSET = 0.7;
constexpr auto PRE_Z_DIFF = 0.26;
constexpr auto GRIP_Z_DIFF = 0.2;  

const std::string ROBOT = "robot";
const std::string BLOCK = "block_";


PickAndPlaceNode::PickAndPlaceNode( double planning_time)
{
  move_group_.setPlanningTime(planning_time);
}

void PickAndPlaceNode::get_blocks() {
  blocks_.resize(5);
  gazebo_msgs::GetModelState srv;
  srv.request.relative_entity_name = ROBOT;
  for (int i=0; i<5; ++i){
    srv.request.model_name = BLOCK + std::to_string(i+1);
    while(!gaz_client_.call(srv)){
      ROS_INFO_STREAM_THROTTLE(1.0, "waiting for the position of the block " << i + 1);
      ros::WallDuration(0.5).sleep();
    };
    blocks_[i] = std::move(srv.response.pose);
  }
}

void PickAndPlaceNode::open_gripper()
{
  auto msg = std_msgs::Float64();
  msg.data = 0.05;
  
  //for safety publish trice
  for (int i = 0; i < 3; ++i) {
    gripper_command_pub_.publish(msg);
    ros::WallDuration(0.05).sleep();
  }
}

void PickAndPlaceNode::close_gripper()
{
  auto msg = std_msgs::Float64();
  msg.data = -0.5;
  //for safety publish trice
  for (int i = 0; i < 3; ++i) {
    gripper_command_pub_.publish(msg);
    ros::WallDuration(0.05).sleep();
  }
}

void PickAndPlaceNode::prepare(){ 
  //add collisions and get blocks positions
    get_blocks(); 
    fill_collision_objects();
    
    //init the target orientation
    auto gripper_pose = move_group_.getCurrentPose().pose; 
    tf2::fromMsg(gripper_pose.orientation, q_orient_);
    tf2::Quaternion rot;
    rot.setRPY(0, 0, M_PI_2);

    q_orient_ = rot * q_orient_;
}

void PickAndPlaceNode::replace_blocks(){
 
  prepare();

  //get sure that initially gripper is opened
  open_gripper();
  for (auto& block : blocks_){
    pick(block);
    place(block);
  }
}


void PickAndPlaceNode::move(const geometry_msgs::Pose& block, const bool is_picking)
{
  
  //set pre grip/leave and grip positin
  auto pre_pos = block;
  auto grip_pos = block;

  pre_pos.position.z += PRE_Z_DIFF;
  grip_pos.position.z += GRIP_Z_DIFF;


  if (!is_picking) {
    pre_pos.position.y *= -1;
    grip_pos.position.y *= -1;
  }
 
  pre_pos.orientation = tf2::toMsg(q_orient_);
  grip_pos.orientation = tf2::toMsg(q_orient_);

  //go to pre_pos position;
  move_group_.setPoseTarget(pre_pos);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    move_group_.execute(my_plan);
  } else {
    ROS_WARN("NO Trajectory was foud");
  }

  //go to grip position;
  move_group_.setPoseTarget(grip_pos);

  if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    move_group_.execute(my_plan);
  } else {
      ROS_WARN("NO Trajectory was foud");
  }

  //grip / leave 
  if (is_picking) {
    close_gripper();
  } else {
    open_gripper();
  }

  //wait and get back to pre pos
  ros::WallDuration(1.0).sleep();
  move_group_.setPoseTarget(pre_pos);
  if(move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    move_group_.execute(my_plan);
  } else {
      ROS_WARN("NO Trajectory was foud");
  }
}

void PickAndPlaceNode::fill_collision_objects()
{
  // Add the column
  collision_objects_.emplace_back();
  collision_objects_[0].id = "column";
  collision_objects_[0].header.frame_id = "base_link_inertia";

  collision_objects_[0].primitives.resize(1);
  collision_objects_[0].primitives[0].type = collision_objects_[0].primitives[0].BOX;
  collision_objects_[0].primitives[0].dimensions.resize(3);
  collision_objects_[0].primitives[0].dimensions[0] = 0.042;
  collision_objects_[0].primitives[0].dimensions[1] = 0.042;
  collision_objects_[0].primitives[0].dimensions[2] = 0.74;

  collision_objects_[0].primitive_poses.resize(1);
  collision_objects_[0].primitive_poses[0].position.x = 0;
  collision_objects_[0].primitive_poses[0].position.y = 0;
  collision_objects_[0].primitive_poses[0].position.z = 0.37 - Z_OFFSET;

  collision_objects_[0].operation = collision_objects_[0].ADD;

  // Add the first table
  moveit_msgs::CollisionObject table;
  table.id = "cafe_table_1";
  table.header.frame_id = "base_link_inertia";

  table.primitives.resize(1);
  table.primitives[0].type = collision_objects_[0].primitives[0].BOX;
  table.primitives[0].dimensions.resize(3);
  table.primitives[0].dimensions[0] = 0.913;
  table.primitives[0].dimensions[1] = 0.913;
  table.primitives[0].dimensions[2] = 0.04;

  table.primitive_poses.resize(1);
  table.primitive_poses[0].position.x = 0.0;
  table.primitive_poses[0].position.y = -0.6;
  table.primitive_poses[0].position.z = 0.755 - Z_OFFSET;
  
  table.operation = collision_objects_[1].ADD;

  collision_objects_.push_back(table);

  //Add the second table

  table.id = "cafe_table_2";
  table.header.frame_id = "base_link_inertia";
  table.primitive_poses[0].position.y = 0.6; 
  collision_objects_.push_back(std::move(table));

  planning_scene_interface_.applyCollisionObjects(collision_objects_);
}


