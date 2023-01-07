clear
clc
close all

%% Information
% Alternative to Simulink model for publication of /cmd_vel

%% Loading
load('PublishingSmooth.mat')

%% Imposing another frequency

desiredF = 10;
dt = 1/desiredF;
tO = odomTime(1:end-1);
VL = CmdVelLon;
VA = CmdVelAng;

tN = (tO(1):dt:tO(end)).' - tO(1);
VLN = VL(1:3:length(VL));
VAN = VA(1:3:length(VA));

siminVLN.time = tN;
siminVLN.signals.values = VLN;
siminVLN.signals.dimensions = 1;

siminVAN.time = tN;
siminVAN.signals.values = VAN;
siminVAN.signals.dimensions = 1;



%% Initialize publication with Matlab

%%%%%% rosinit %%%%%%
tbot = turtlebot('localhost');

odom = getOdometry(tbot); 
tbot.Velocity.TopicName = '/cmd_vel';

desiredRate = 10;
rate = rosrate(desiredRate);
reset(rate)

time = odomTime(1:end-1) - odomTime(1);

%% Publication

for ii = 1:length(time)
    setVelocity(tbot, CmdVelLon(ii), CmdVelAng(ii));
    waitfor(rate);
end

