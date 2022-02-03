#pragma once

#include <vector>
#include <utility>

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_msgs/Float64.h>

class PickAndPlaceNode : public ros::NodeHandle {
public:
    PickAndPlaceNode( double planning_time = 10.0);
    void prepare();
    void replace_blocks();
    void open_gripper();
    void close_gripper();
    
private:
    void fill_collision_objects();
    void get_blocks();
    void pick(const geometry_msgs::Pose& block) { move(block); };
    void place(const geometry_msgs::Pose& block) { move(block, false); };
    void move(
        const geometry_msgs::Pose& block, 
        const bool is_picking = true);
    void initially_rotate_gripper();

private:

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_ {};
    moveit::planning_interface::MoveGroupInterface move_group_ {"arm"};
    std::vector<moveit_msgs::CollisionObject> collision_objects_{};
    std::vector<geometry_msgs::Pose> blocks_ {};
    ros::ServiceClient gaz_client_ = serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::Publisher gripper_command_pub_ = advertise<std_msgs::Float64>("/gripper_joint_position/command", 50);
    trajectory_msgs::JointTrajectory posture_ {};
    tf2::Quaternion q_orient_{};
    geometry_msgs::Pose initial_gripper_pose_ {};
};