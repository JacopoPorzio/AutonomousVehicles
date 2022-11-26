clear
clc
close all

%% Loading
% load('PublishingNEW.mat')
load('PublishingSmooth.mat')
% CmdVelLon and CmdVelAng run at 30 Hz
AvgFreq = 1/(mean(diff(odomTime)));

% odomTime = (0:1:3315)';

%% Imposing another frequency
% 10 Hz is the original /cmd_vel publishing frequency
if 1
    desiredF = 10;
    dt = 1/desiredF;
    tO = odomTime(1:end-1);
    VL = CmdVelLon;
    VA = CmdVelAng;
    
    tN = (tO(1):dt:tO(end)).' - tO(1);
%     VLN = interp1(tO, VL, tN);
%     VAN = interp1(tO, VA, tN);
    VLN = VL(1:3:length(VL));
    VAN = VA(1:3:length(VA));
    
    siminVLN.time = tN;
    siminVLN.signals.values = VLN;
    siminVLN.signals.dimensions = 1;

    siminVAN.time = tN;
    siminVAN.signals.values = VAN;
    siminVAN.signals.dimensions = 1;
end
 
% odomTime = tN; CmdVelLon = VLN; CmdVelAng = VAN;

%% Comparison published bag

% bagselectIN = rosbag('PublishTest.bag');
bagselectIN = rosbag('PublishTest2.bag');
cmdBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/cmd_vel');

cmdStructs = readMessages(cmdBag, 'DataFormat','struct');
cmdTime = cmdBag.MessageList.Time;

AvgFreqN = 1/(mean(diff(cmdTime)));
% with this test, I've acknowledged that the publishing frequency is almost
% half of the desired one!
cmdVL = zeros(length(cmdStructs), 1); cmdVA = zeros(length(cmdStructs), 1); 
for ii = 1:length(cmdStructs)
    cmdVL(ii) = cmdStructs{ii}.Linear.X;
    cmdVA(ii) = cmdStructs{ii}.Angular.Z;
end

T1 = odomTime(1:end-1) - odomTime(1);
% T2 = (cmdTime - cmdTime(1) + 3.236)*AvgFreqN/30;
T2 = (cmdTime - cmdTime(1) + 3.236);
% T2 = cmdTime - cmdTime(1);


figure
plot(T1, CmdVelLon, 'r')
hold on
plot(T2, cmdVL, 'b')
grid on
legend('Put', 'Found')

%% Publishing with Matlab

% load('PublishingTWIST.mat')
% CmdVelLon = odomTLX; CmdVelAng = odomTAZ;

tbot = turtlebot('localhost');

odom = getOdometry(tbot); 
tbot.Velocity.TopicName = '/cmd_vel';

% desiredRate = 23.6; % 30 Hz
% desiredRate = 30;
% desiredRate = 30.25;
desiredRate = 10;
% rate = rateControl(desiredRate);
rate = rosrate(desiredRate);
reset(rate)

time = odomTime(1:end-1) - odomTime(1);

%%
for ii = 1:length(time)
%     tstart = tic;
    setVelocity(tbot, CmdVelLon(ii), CmdVelAng(ii));
    waitfor(rate);
%     pause(1/30)
%     tEla = toc(tstart);
%     FrEq = 1/( tEla - tstart)
end

% apply filter on cmdvel -> fatto test usando twist in CmdVel
% il problema che non sia il rumore sulla stima della velocita'
