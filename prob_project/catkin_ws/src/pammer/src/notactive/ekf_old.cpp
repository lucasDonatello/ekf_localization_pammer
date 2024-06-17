#include <ros/ros.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <vector>

using namespace std;

std::vector<geometry_msgs::PoseStamped> poses;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    poses.push_back(*msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_subscriber_and_saver");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("goals", 10, poseCallback);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        ros::spinOnce();

        // for (int i = 0; i < static_cast<int>(poses.size()); ++i)
        // {
        //     cout << "Seq: " << poses[i].header.seq << endl;
        //     cout << "X of " << i + 1 << " " << poses[i].pose.position.x << endl;
        //     cout << "Y of " << i + 1 << " " << poses[i].pose.position.y << endl;
        //     cout << "Z of " << i + 1 << " " << poses[i].pose.position.z << endl;
        //     cout << "------------" << endl;
        // }

        for (size_t i = 0; i < poses.size(); ++i)
        {
            ROS_INFO("Seq %u: x=%f, y=%f, z=%f",
                     poses[i].header.seq,
                     poses[i].pose.position.x,
                     poses[i].pose.position.y,
                     poses[i].pose.position.z);
        }

        loop_rate.sleep();
    }

    return 0;
}
