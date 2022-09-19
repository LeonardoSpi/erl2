#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <std_srvs/SetBool.h>

bool reach(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
  
  // We start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/noetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "panda_arm" of the Panda
  // robot.
  
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  moveit::planning_interface::MoveGroupInterface group("arm");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  
  // Move to default positions
  
  group.setStartStateToCurrentState();
  group.setGoalOrientationTolerance(0.01);
  group.setGoalPositionTolerance(0.01);
  
  	if(req.data){
	group.setNamedTarget("grab_high");
	group.move();
	resp.message = "grab_high";
	}
	else {
	group.setNamedTarget("grab_low");
	group.move();  
	resp.message = "grab_low";
	}
	
   std::cout << "moving arm..." << std::endl;
  
  //group.setStartStateToCurrentState();
  //group.setGoalOrientationTolerance(0.5);
  //group.setGoalPositionTolerance(0.5);
  //resp.success = true;
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::NodeHandle nh;
  //ros::AsyncSpinner spinner(1);
  //spinner.start();
  
  ros::ServiceServer service = nh.advertiseService("move_arm", reach);
  ROS_INFO("Ready to move arm");
  ros::spin();

  return 0;
}


