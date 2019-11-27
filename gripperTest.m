clear; clc; close all;
rosshutdown;

setenv('ROS_MASTER_URI', 'http://192.168.1.20');
setenv('ROS_HOSTNAME', '192.168.1.20');
rosinit('192.168.1.20','NodeHost','192.168.1.20')

clear
close all;
cmdSpecialPub = rospublisher('/cmd_special', 'std_msgs/String');
msgGripper = rosmessage('std_msgs/String');
open = "open";
close = "close";
msgGripper.Data = close
send(cmdSpecialPub, msgGripper)