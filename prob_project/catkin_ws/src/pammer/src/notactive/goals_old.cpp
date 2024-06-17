#include <ros/ros.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("goals", 10);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {

        std::vector<geometry_msgs::PoseStamped> poses(4);

        ros::Time current_time = ros::Time::now();
        std::string frame_id = "world";

        for (int i = 0; i < 4; ++i)
        {
            poses[i].header.stamp = current_time;
            poses[i].header.frame_id = frame_id;
            poses[i].pose.position.x = 1.0 * (i + 1);
            poses[i].pose.position.y = 2.0 * (i + 1);
            poses[i].pose.position.z = 3.0 * (i + 1);
            poses[i].pose.orientation.x = 0.0;
            poses[i].pose.orientation.y = 0.0;
            poses[i].pose.orientation.z = 0.0;
            poses[i].pose.orientation.w = 1.0;

            pub.publish(poses[i]);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
