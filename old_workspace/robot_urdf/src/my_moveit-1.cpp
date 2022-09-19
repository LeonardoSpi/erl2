#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <std_srvs/SetBool.h>

bool reach(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
  
	moveit::planning_interface::MoveGroupInterface group("arm");
	group.setEndEffectorLink("arm_link_04");
	group.setPoseReferenceFrame("base_link");
	group.setPlannerId("RRTstar");
	group.setNumPlanningAttempts(10);
	group.setPlanningTime(10.0);
	group.allowReplanning(true);
	group.setGoalJointTolerance(0.1);
	group.setGoalPositionTolerance(0.1);
	group.setGoalOrientationTolerance(0.1);

  
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

  return true;
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "custom_planning");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("move_arm", reach);
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::waitForShutdown();
	//ros::spin();

}


