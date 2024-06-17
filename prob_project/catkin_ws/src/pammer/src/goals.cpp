#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "send_goal");
	ros::NodeHandle nh;

	// tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	// wait for the action server to come up
	while (!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal home;

	nh.getParam("/goals_node/homeX", home.target_pose.pose.position.x);
	nh.getParam("/goals_node/homeY", home.target_pose.pose.position.y);
	nh.getParam("/goals_node/homeTh", home.target_pose.pose.orientation.w);

	move_base_msgs::MoveBaseGoal goal;
	geometry_msgs::Pose goalAct;
	std::vector<geometry_msgs::Pose> goals;

	nh.getParam("/goals_node/goalXone", goalAct.position.x);
	nh.getParam("/goals_node/goalYone", goalAct.position.y);
	nh.getParam("/goals_node/goalThone", goalAct.orientation.w);
	goals.push_back(goalAct);

	ROS_INFO("Read Goal 1 with X: %f, Y: %f, W: %f", goalAct.position.x, goalAct.position.y, goalAct.orientation.w);

	nh.getParam("/goals_node/goalXtwo", goalAct.position.x);
	nh.getParam("/goals_node/goalYtwo", goalAct.position.y);
	nh.getParam("/goals_node/goalThtwo", goalAct.orientation.w);
	goals.push_back(goalAct);

	ROS_INFO("Read Goal 2 with X: %f, Y: %f, W: %f", goalAct.position.x, goalAct.position.y, goalAct.orientation.w);

	nh.getParam("/goals_node/goalXthree", goalAct.position.x);
	nh.getParam("/goals_node/goalYthree", goalAct.position.y);
	nh.getParam("/goals_node/goalThthree", goalAct.orientation.w);
	goals.push_back(goalAct);

	ROS_INFO("Read Goal 3 with : %f, Y: %f, W: %f", goalAct.position.x, goalAct.position.y, goalAct.orientation.w);

	nh.getParam("/goals_node/goalXfour", goalAct.position.x);
	nh.getParam("/goals_node/goalYfour", goalAct.position.y);
	nh.getParam("/goals_node/goalThfour", goalAct.orientation.w);
	goals.push_back(goalAct);

	ROS_INFO("Read Goal 4 with X: %f, Y: %f, W: %f", goalAct.position.x, goalAct.position.y, goalAct.orientation.w);

	ROS_INFO("Going Home - X: %f, Y: %f, W: %f", home.target_pose.pose.position.x, home.target_pose.pose.position.y, home.target_pose.pose.orientation.w);
	home.target_pose.header.frame_id = "map";
	home.target_pose.header.stamp = ros::Time::now();

	ac.sendGoal(home);

	ac.waitForResult();

	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Reached Goal!");
	}
	else
	{
		ROS_INFO("Unable to reach goal!");
	}

	for (int i = 0; i < static_cast<int>(goals.size()); i++)
	{
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = goals[i].position.x;
		goal.target_pose.pose.position.y = goals[i].position.y;
		goal.target_pose.pose.orientation.w = goals[i].orientation.w;

		ROS_INFO("Sending goal!");
		ROS_INFO("Sent goal with values X: %f, Y: %f, W: %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.w);

		ac.sendGoal(goal);

		ac.waitForResult();

		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Reached Goal!");
		}
		else
		{
			ROS_INFO("Unable to reach goal!");
		}
	}

	return 0;
}
