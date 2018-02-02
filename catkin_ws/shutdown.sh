#! /bin/bash
rosnode kill -a
ps -ef|grep roscore| awk '{print $2}'|xargs kill
ps -ef|grep startup.sh| awk '{print $2}'|xargs kill
