clear
clc
close all

bagselectIN = rosbag('MSIbag.bag');

% odomBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/odom');
odomBag = select(bagselectIN, 'Topic', '/odom');
odomStructs = readMessages(odomBag, 'DataFormat','struct');
odomTime = odomBag.MessageList.Time;


% cmdBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/cmd_vel');
cmdBag = select(bagselectIN, 'Topic', '/cmd_vel');
cmdStructs = readMessages(cmdBag, 'DataFormat','struct');
cmdTime = cmdBag.MessageList.Time;

