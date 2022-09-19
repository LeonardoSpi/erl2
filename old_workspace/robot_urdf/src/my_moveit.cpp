#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_srvs/SetBool.h>
#include "std_msgs/String.h"

bool reach(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
  
  	ros::AsyncSpinner spinner(1);
	spinner.start();
  
	moveit::planning_interface::MoveGroupInterface group("arm");
	group.setEndEffectorLink("cluedo_link");
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
		resp.success = 1;
		}
	else {
		
		group.setNamedTarget("grab_low");
		group.move();  
		resp.message = "grab_low";
		resp.success = 1;
		}
	
   std::cout << "moving arm..." << std::endl;

  return true;
}



/*void move(const std_msgs::String::ConstPtr& msg)
  {
  	int counter = 0;
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
  
  	
  
       if(msg->data=="gh"){
		group.setNamedTarget("grab_high");
		while(counter<10){
			group.move();
			sleep(1);
			counter=counter+1;
			}
		counter = 0;
		}
	else {
		group.setNamedTarget("grab_low");
		while(counter<10){
			group.move();
			sleep(1);
			counter=counter+1;
			}
		counter = 0;
		}
  }
*/


int main(int argc, char** argv)
{

	ros::init(argc, argv, "custom_planning");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("move_arm", reach);
	//ros::Subscriber sub = nh.subscribe("movearm", 1000, move);
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::waitForShutdown();
	//ros::spin();

}


