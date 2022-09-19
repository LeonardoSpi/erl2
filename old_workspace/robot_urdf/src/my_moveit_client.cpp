#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_srvs/SetBool.h>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"

int movearm = 0;

void checkposition(const nav_msgs::Odometry::ConstPtr& msg)
{

  	ROS_INFO("Seq: [%d]", msg->header.seq);
  	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);

	if((msg->pose.pose.position.x==2.5)&&(msg->pose.pose.position.y==0.0))
	{
		movearm = 1;
		//srv.request.data = 1;
		//client.call(srv);
	}
	
	if((msg->pose.pose.position.x==0.0)&&(msg->pose.pose.position.y==-2.5))
	{
		movearm = 2;
		//srv.request.data = 1;
		//client.call(srv);
	}
	
	if((msg->pose.pose.position.x==-2.5)&&(msg->pose.pose.position.y==0.0))
	{
		movearm = 3;
		//srv.request.data = 0;
		//client.call(srv);
	}
	
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "moveit_client");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("odom", 1000, checkposition);
	
	ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("move_arm");
	
	std_srvs::SetBool srv;
	
	if(movearm = 1)
	{
		srv.request.data = 1;
		client.call(srv);
		sleep(5);
		srv.request.data = 0;
		client.call(srv);
		sleep(5);
	}
	
	if(movearm = 2)
	{
		srv.request.data = 1;
		client.call(srv);
		sleep(5);
		srv.request.data = 0;
		client.call(srv);
		sleep(5);
	}
	
	if(movearm = 3)
	{
		srv.request.data = 1;
		client.call(srv);
		sleep(5);
		srv.request.data = 0;
		client.call(srv);
		sleep(5);
	}
	
	//std_srvs::SetBool srv;
	
	//srv.request.data = 1;
	//client.call(srv);
	
	//srv.request.data = 0;
	//client.call(srv);
	ros::spin();
}



