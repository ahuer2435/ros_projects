#! /bin/bash
roscore&
sleep 5s
rosrun apo_localization gps_imu_localization_node&
rosrun apo_planning rtk_recorder_node& 
rosrun apo_planning rtk_replay_node&
rosrun apo_controller bicycle_controller_node&

while true;
do
	sleep 120s
done

