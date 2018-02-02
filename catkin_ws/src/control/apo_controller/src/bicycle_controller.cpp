#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <apo_controller/VehicleCmd.h>

static void trajectory_process_callback(const trajectory_msgs::MultiDOFJointTrajectory& input)
{
    //output cmd
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bicycle_controller_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_trajectory = nh.subscribe("trajectory", 2, trajectory_process_callback);
    ros::Publisher pub_vehicleCmd = nh.advertise<apo_controller::VehicleCmd>("VehicleCmd",10,true);
    ros::spin();
    return 0;
}

