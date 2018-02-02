#include <ros/ros.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/TwistStamped.h>
#include <apo_localization/localizer_pose.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define LOCALIZATIION_FREQ 30

static void OnTimer(const ros::TimerEvent &event)
{
    //fuse data and publish.
}

static void ReadTrajectoryFile()
{
    //read csv file
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_replay_node");
    ros::NodeHandle nh;

    ros::Publisher  pub_trajectory = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory", 2, true);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/LOCALIZATIION_FREQ), OnTimer);

    ros::spin();
    return 0;
}

