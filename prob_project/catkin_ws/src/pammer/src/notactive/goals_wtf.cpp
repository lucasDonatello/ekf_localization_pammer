#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_navigation");
    ros::NodeHandle nh;

    // Initialize move_base client
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Define waypoints
    std::vector<std::pair<double, double>> waypoints = {
        {1.0, 1.0},  // Waypoint 1
        {2.0, 2.0},  // Waypoint 2
        {3.0, 3.0},  // Waypoint 3
        {4.0, 4.0}   // Waypoint 4
    };

    // Send navigation goals to move_base for each waypoint
    for (size_t i = 0; i < waypoints.size(); ++i) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = waypoints[i].first;
        goal.target_pose.pose.position.y = waypoints[i].second;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal to waypoint %zu: (%.2f, %.2f)", i+1, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Successfully reached waypoint %zu", i+1);
        } else {
            ROS_WARN("Failed to reach waypoint %zu", i+1);
        }

        // Pause between waypoints (for demonstration purposes)
        ros::Duration(5.0).sleep();
    }

    return 0;
}
