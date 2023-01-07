clear
clc
close all

%%
load('Analysis.mat')
OX = odomX; OY = odomY; OA = odomAngle(:,1); OT = odomTime;
OT = OT - OT(1);
clear odomX; clear odomY; clear odomTime


%%
bagselectIN = rosbag('MSIbag.bag');
tSelect = 375;

% bagselectIN = rosbag('MSIsupersmooth.bag'); 
% tSelect = 290;

% bagselectIN = rosbag('MSIsupersmooth2.bag'); 
% tSelect = 290;

odomBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/odom');

odomStructs = readMessages(odomBag, 'DataFormat','struct');
odomTime = extractTime(odomStructs, length(odomStructs));


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

% to unwrap
% odomAngle(2907:3240,1) = odomAngle(2907:3240,1)- 2*odomAngle(2907,1);
% odomAngle(2842:3181,1) = odomAngle(2842:3181,1)- 2*odomAngle(2842,1);
% odomAngle(2787:3119,1) = odomAngle(2787:3119,1)- 2*odomAngle(2787,1);
% odomAngle(3076:3405,1) = odomAngle(3076:3405,1) - 2*odomAngle(3076,1);
odomAngle(2980:3311,1) = odomAngle(2980:3311,1) - 2*odomAngle(2980,1);

figure
sgtitle('\boldmath$Linear\ odometry:\ x\ vs\ y$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax1 = axes;
scatter(odomX, odomY, [], odomTime);
xlim([-2 0.5])
xlabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
colormap hsv
h = colorbar;
ylabel(h, '$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
h.TickLabelInterpreter = 'LaTex';
ax1.TickLabelInterpreter = 'LaTex';
ax1.FontSize = 16;
set(gcf,'color','w');
grid on

%% Position comparison

figure
ax2 = axes;
sgtitle('\boldmath$Linear\ Odometry\ comparison$', 'FontSize', 16, 'Interpreter', 'LaTex')
plot(odomX(tSelect:end), odomY(tSelect:end), 'r', 'LineWidth', 2)
hold on
plot(OX, OY, '--b', 'LineWidth', 2)
grid on
legend('Found', 'Original', 'FontSize', 10, 'Interpreter', 'LaTex')
ylim([-0.7 0.7])
xlim([-2 0.5])
xlabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax2.TickLabelInterpreter = 'LaTex';
ax2.FontSize = 16;
set(gcf,'color','w');


figure
sgtitle('\boldmath$Odometry\ comparison$', 'FontSize', 16, 'Interpreter', 'LaTex')

ax3 = subplot(3,1,1);
plot(odomTime, odomX, 'r', 'LineWidth', 2)
hold on
plot(OT, OX, '--b', 'LineWidth', 2)
grid on
legend('Found', 'Original', 'FontSize', 10, 'Interpreter', 'LaTex')
ylabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax3.TickLabelInterpreter = 'LaTex';
ax3.FontSize = 16;
xlim([0 OT(end)])

ax4 = subplot(3,1,2);
plot(odomTime, odomY, 'r', 'LineWidth', 2)
hold on
plot(OT, OY, '--b', 'LineWidth', 2)
grid on
legend('Found', 'Original', 'FontSize', 10, 'Interpreter', 'LaTex')
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax4.TickLabelInterpreter = 'LaTex';
ax4.FontSize = 16;
xlim([0 OT(end)])

ax5 = subplot(3,1,3);
plot(odomTime, odomAngle(:,1), 'r', 'LineWidth', 2)
hold on
plot(OT, OA, '--b', 'LineWidth', 2)
grid on
legend('Found', 'Original', 'FontSize', 10, 'Interpreter', 'LaTex')
ylabel('$\theta\ [rad]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax5.TickLabelInterpreter = 'LaTex';
ax5.FontSize = 16;
xlim([0 OT(end)])

set(gcf,'color','w');

%% Velocity comparison

load('PublishingSmooth.mat', 'CmdVelAng', 'CmdVelLon')
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
sgtitle('\boldmath$Comparison\ between\ found\ and\ imposed\ velocity$', 'FontSize', 16, 'Interpreter', 'LaTex')

ax6 = subplot(2,1,1);
plot(odomTime(1:end-1), CmdVelLon, 'r', 'LineWidth', 2)
hold on
plot(OT(1:end-1), CVL, 'b', 'LineWidth', 1)
legend('Found', 'Original', 'FontSize', 10, 'Interpreter', 'LaTex')
title('$Longitudinal\ velocity$', 'FontSize', 12, 'Interpreter', 'LaTex')
ylabel('$v\ [m/s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
grid on
xlim([0 OT(end)])
ax6.TickLabelInterpreter = 'LaTex';
ax6.FontSize = 12;

ax7 = subplot(2,1,2);
plot(odomTime(1:end-1), CmdVelAng, 'r', 'LineWidth', 2)
hold on
plot(OT(1:end-1), CVA, 'b', 'LineWidth', 1)
legend('Found', 'Original', 'FontSize', 10, 'Interpreter', 'LaTex')
title('$Angular\ velocity$', 'FontSize', 12, 'Interpreter', 'LaTex')
ylabel('$\dot{\theta}\ [rad/s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
grid on
xlim([0 OT(end)])
ax7.TickLabelInterpreter = 'LaTex';
ax7.FontSize = 12;

set(gcf,'color','w');

AvgFreqFou = 1/mean(diff([odomTime(1:end-2); odomTime(end)]));
AvgFreqImp = 1/mean(diff(OT));

%% Estimation errors
ddt = 1/10;
timeV = 0:ddt:OT(end);

[odomTimeR, POS] = unique(odomTime(150:end));
odomXR = odomX(150:end); odomYR = odomY(150:end); odomThR = odomAngle(150:end,1);

errPosX  = interp1(odomTimeR, odomXR(POS), timeV) - interp1(OT, OX, timeV);
errPosY  = interp1(odomTimeR, odomYR(POS), timeV) - interp1(OT, OY, timeV);
errPosTh = interp1(odomTimeR, odomThR(POS), timeV) - interp1(OT, OA, timeV);

figure
sgtitle('\boldmath$Position\ estimation\ error$', 'FontSize', 16, 'Interpreter', 'LaTex')

ax8 = subplot(3,1,1);
plot(timeV, errPosX, 'LineWidth', 2)
grid on
title('$x\ direction$', 'FontSize', 12, 'Interpreter', 'LaTex')
ylabel('$err_x \ [m]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlim([0 OT(end)])
ax8.TickLabelInterpreter = 'LaTex';
ax8.FontSize = 12;

ax9 = subplot(3,1,2);
plot(timeV, errPosY, 'LineWidth', 2)
grid on
title('$y\ direction$', 'FontSize', 12, 'Interpreter', 'LaTex')
ylabel('$err_{y}\ [m]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlim([0 OT(end)])
ax9.TickLabelInterpreter = 'LaTex';
ax9.FontSize = 12;

ax10 = subplot(3,1,3);
plot(timeV, errPosTh, 'LineWidth', 2)
grid on
title('$\theta\ direction$', 'FontSize', 12, 'Interpreter', 'LaTex')
ylabel('$err_{\theta}\ [rad]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlim([0 OT(end)])
ax10.TickLabelInterpreter = 'LaTex';
ax10.FontSize = 12;

set(gcf,'color','w');