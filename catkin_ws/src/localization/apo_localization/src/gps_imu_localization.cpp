#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <apo_localization/localizer_pose.h>

#define LOCALIZATIION_FREQ 30
static void gps_callback(const nmea_msgs::Sentence& input)
{
    //get gps data.
}

static void imu_callback(const sensor_msgs::Imu& input)
{
    //get imu data.
}

static void OnTimer(const ros::TimerEvent &event)
{
    //fuse data and publish.
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_imu_localization_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_gps = nh.subscribe("nmea_sentence", 10, gps_callback);
    ros::Subscriber sub_imu = nh.subscribe("imu_raw", 10, imu_callback);

    ros::Publisher  pub_localization_pose = nh.advertise<apo_localization::localizer_pose>("localization_pose", 2, true);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/LOCALIZATIION_FREQ), OnTimer);

    ros::spin();
    return 0;
}

