#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{	
  ros::init(argc, argv, "move_group_1");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //----------------------------
  //Setup
  //----------------------------

  static const std::string PLANNING_GROUP = "arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  //Using :planning_scene_interface:'PlanningSceneInterface' class to deal directly with the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  move_group.setEndEffectorLink("Link_5");
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  // We can print the name of the reference frame for this robot.
  // also printing the current position and orientation of the robot.
  ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
  ROS_INFO_NAMED("moveo", "x position: %f", current_pose.pose.position.x);
  ROS_INFO_NAMED("moveo", "y position: %f", current_pose.pose.position.y);
  ROS_INFO_NAMED("moveo", "z position: %f", current_pose.pose.position.z);
  ROS_INFO_NAMED("moveo", "x orientation: %f", current_pose.pose.orientation.x);
  ROS_INFO_NAMED("moveo", "y orientation: %f", current_pose.pose.orientation.y);
  ROS_INFO_NAMED("moveo", "z orientation: %f", current_pose.pose.orientation.z);
  ROS_INFO_NAMED("moveo", "w orientation: %f", current_pose.pose.orientation.w);
 
  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.0; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Moveo Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();


  //-----------------------------
  //Getting Basic Information
  //-----------------------------

  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("moveo", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("moveo", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  //-----------------------------
  //Planning to a Pose Goal
  //-----------------------------

  //Plan a motion for this group to a desired pose for end-effector
  // hardcode desired position here before running node in a separate terminal
  geometry_msgs::Pose target_pose1;
  //default pose
  target_pose1.position.x = 0.120679;
  target_pose1.position.y = 0.072992;
  target_pose1.position.z = 0.569166;
  target_pose1.orientation.x = -0.386473;
  target_pose1.orientation.y =  -0.418023;
  target_pose1.orientation.z = -0.760978;
  target_pose1.orientation.w = 0.311139;

  //upright pose
  // target_pose1.position.x = 0.000130;
  // target_pose1.position.y = -0.240464;
  // target_pose1.position.z = 0.756570;
  // target_pose1.orientation.x = 0.359602;
  // target_pose1.orientation.y =  0.240924;
  // target_pose1.orientation.z = -0.747187;
  // target_pose1.orientation.w = 0.504335;


//upright pose using robot_pose_publisher
// position: 
//   x: 0.450865569212
//   y: -0.0923533864181
//   z: -0.646847372618
// orientation: 
//   x: -0.359579060437
//   y: -0.240936531262
//   z: 0.747165791213
//   w: 0.5043766129


  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);

  ROS_INFO_NAMED("moveo", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.
  ROS_INFO_NAMED("moveo", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group.move();

  ros::shutdown();  
  return 0;
}
