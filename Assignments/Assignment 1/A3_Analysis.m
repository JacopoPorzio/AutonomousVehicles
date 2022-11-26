clear
clc
close all

%%
load('Analysis.mat')
OX = odomX; OY = odomY; OA = odomAngle(:,1); OT = odomTime;
OT = OT - OT(1);
clear odomX; clear odomY; clear odomTime


%%
% bagselectIN = rosbag('OK_BAG2.bag');
% tSelect = 34;
% bagselectIN = rosbag('TwistPerfect.bag');
% tSelect = 200;
% bagselectIN = rosbag('BagSmooth.bag'); %prova con BagSmooth2
% tSelect = 150;
% bagselectIN = rosbag('Bag10HzPerf.bag'); %prova con BagSmooth2
% tSelect = 90;
% bagselectIN = rosbag('BagPubSimulink.bag'); %prova con BagSmooth2
% tSelect = 90;
bagselectIN = rosbag('BagPubSimulink2.bag'); %prova con BagSmooth2
tSelect = 90;

odomBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/odom');

odomStructs = readMessages(odomBag, 'DataFormat','struct');
odomTime = odomBag.MessageList.Time;

odomTime = odomTime - odomTime(tSelect);

odomX = zeros(length(odomStructs), 1);
odomY = zeros(length(odomStructs), 1);
odomZ = zeros(length(odomStructs), 1);

odomOX = zeros(length(odomStructs), 1);
odomOY = zeros(length(odomStructs), 1);
odomOZ = zeros(length(odomStructs), 1);
odomOW = zeros(length(odomStructs), 1);

odomAngle = zeros(length(odomStructs), 3);

for ii = 1:length(odomStructs)
   
    odomX(ii) = odomStructs{ii}.Pose.Pose.Position.X;
    odomY(ii) = odomStructs{ii}.Pose.Pose.Position.Y;
    odomZ(ii) = odomStructs{ii}.Pose.Pose.Position.Z;
    
    odomOX(ii) = odomStructs{ii}.Pose.Pose.Orientation.X;
    odomOY(ii) = odomStructs{ii}.Pose.Pose.Orientation.Y;
    odomOZ(ii) = odomStructs{ii}.Pose.Pose.Orientation.Z;
    odomOW(ii) = odomStructs{ii}.Pose.Pose.Orientation.W;

    odomAngle(ii, :) = quat2eul([odomOW(ii) odomOX(ii) odomOY(ii) odomOZ(ii)]);

end

% odomAngle(2907:3240,1) = odomAngle(2907:3240,1)- 2*odomAngle(2907,1);
% odomAngle(2842:3181,1) = odomAngle(2842:3181,1)- 2*odomAngle(2842,1);
odomAngle(2787:3119,1) = odomAngle(2787:3119,1)- 2*odomAngle(2787,1);


figure
title('$Linear\ Odometry:\ x\ vs\ y$', 'Interpreter', 'LaTex')
% plot(odomX, odomY, 'LineWidth', 2)
scatter(odomX, odomY, [], odomTime)
colormap copper
h = colorbar;
ylabel(h, 'Time')
grid on

%% Position comparison

figure
title('$Linear\ Odometry\ comparison$', 'Interpreter', 'LaTex')
% plot(odomX, odomY, 'r', 'LineWidth', 2)
plot(odomX(tSelect:end), odomY(tSelect:end), 'r', 'LineWidth', 2)
hold on
plot(OX, OY, '--b', 'LineWidth', 2)
grid on
legend('Found', 'Original')


figure
sgtitle('$Linear\ Odometry$', 'Interpreter', 'LaTex')

subplot(3,1,1)
% plot(odomTime - odomTime(1), odomX, 'r', 'LineWidth', 2)
plot(odomTime, odomX, 'r', 'LineWidth', 2)
hold on
% plot(OT - OT(1), OX, '--b', 'LineWidth', 2)
plot(OT, OX, '--b', 'LineWidth', 2)
grid on
legend('Found', 'Original')
title('$x$', 'Interpreter', 'LaTex')
xlim([0 120])

subplot(3,1,2)
% plot(odomTime - odomTime(1), odomY, 'r', 'LineWidth', 2)
plot(odomTime, odomY, 'r', 'LineWidth', 2)
hold on
% plot(OT - OT(1), OY, '--b', 'LineWidth', 2)
plot(OT, OY, '--b', 'LineWidth', 2)
grid on
legend('Found', 'Original')
title('$y$', 'Interpreter', 'LaTex')
xlim([0 120])

subplot(3,1,3)
% plot(odomTime - odomTime(1), odomAngle(:,1), 'r', 'LineWidth', 2)
plot(odomTime, odomAngle(:,1), 'r', 'LineWidth', 2)
hold on
% plot(OT - OT(1), OA, '--b', 'LineWidth', 2)
plot(OT, OA, '--b', 'LineWidth', 2)
grid on
legend('Found', 'Original')
title('$\theta$', 'Interpreter', 'LaTex')
xlim([0 120])

%% Velocity comparison

% check here: I published smooth velocities, didn't I?

load('PublishingNEW.mat', 'CmdVelAng', 'CmdVelLon')
CVL = CmdVelLon; CVA = CmdVelAng;
clear CmdVelLon; clear CmVelAng;

xVel = zeros(length(odomTime) - 1, 1);
yVel = zeros(length(odomTime) - 1, 1);
thVel = zeros(length(odomTime) - 1, 1);

for ii = 1 : length(odomTime) - 1
    dt = odomTime(ii + 1) - odomTime(ii);
    xVel(ii) = (odomX(ii + 1) - odomX(ii))/dt;
    yVel(ii) = (odomY(ii + 1) - odomY(ii))/dt;
    thVel(ii) = (odomAngle(ii + 1) - odomAngle(ii))/dt;
end

% % % smoothing
    
    xVel = movmean(xVel, 6);
    yVel = movmean(yVel, 6);
    thVel = movmean(thVel, 6);

% % %

CmdVelLon = (xVel.^2 + yVel.^2).^(1/2);
CmdVelAng = thVel;  

figure
sgtitle('$Comparison\ between\ found\ and\ imposed\ velocity$', 'Interpreter', 'LaTex')

subplot(2,1,1)
plot(odomTime(1:end-1), CmdVelLon, 'r', 'LineWidth', 2)
hold on
plot(OT(1:end-1), CVL, 'b', 'LineWidth', 1)
legend('Found', 'Original')
title('$Longitudinal\ velocity$', 'Interpreter', 'LaTex')
grid on
xlim([0 120])

subplot(2,1,2)
plot(odomTime(1:end-1), CmdVelAng, 'r', 'LineWidth', 2)
hold on
plot(OT(1:end-1), CVA, 'b', 'LineWidth', 1)
legend('Found', 'Original')
title('$Angular\ velocity$', 'Interpreter', 'LaTex')
grid on
xlim([0 120])

AvgFreqFou = 1/mean(diff([odomTime(1:end-2); odomTime(end)]));
AvgFreqImp = 1/mean(diff(OT));

%% Estimation errors
% ddt = 1/30;
ddt = 1/10;
timeV = 0:ddt:OT(end);

[odomTimeR, POS] = unique(odomTime(150:end));
odomXR = odomX(150:end); odomYR = odomY(150:end); odomThR = odomAngle(150:end,1);

errPosX  = interp1(odomTimeR, odomXR(POS), timeV) - interp1(OT, OX, timeV);
errPosY  = interp1(odomTimeR, odomYR(POS), timeV) - interp1(OT, OY, timeV);
errPosTh = interp1(odomTimeR, odomThR(POS), timeV) - interp1(OT, OA, timeV);

figure
sgtitle('$Position\ estimation\ error$', 'Interpreter', 'LaTex')

subplot(3,1,1)
plot(timeV, errPosX)
grid on
title('$x$', 'Interpreter', 'LaTex')

subplot(3,1,2)
plot(timeV, errPosY)
grid on
title('$y$', 'Interpreter', 'LaTex')

subplot(3,1,3)
plot(timeV, errPosTh)
grid on
title('$\theta$', 'Interpreter', 'LaTex')

