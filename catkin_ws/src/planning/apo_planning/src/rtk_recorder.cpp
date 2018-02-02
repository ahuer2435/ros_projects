#include <ros/ros.h>
#include <apo_localization/localizer_pose.h>

static void pose_callback(const apo_localization::localizer_pose& input)
{
    //call WriteTrajectoryFile() to produce csv file
}

static void WriteTrajectoryFile()
{
    //write csv file
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_recorder_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_pose = nh.subscribe("localization_pose", 10, pose_callback);

    ros::spin();
    return 0;
}

