clear
clc
close all

%% Loading

% bagselect = rosbag('FirstBag.bag');
% bagselect = rosbag('SecondBag.bag');
bagselect = rosbag('ThirdBag.bag');

cmdBag = select(bagselect, 'Time', [bagselect.StartTime bagselect.EndTime], 'Topic', '/cmd_vel');
odomBag = select(bagselect, 'Time', [bagselect.StartTime bagselect.EndTime], 'Topic', '/odom');
cpBag = select(bagselect, 'Time', [bagselect.StartTime bagselect.EndTime], 'Topic', '/centroid_position');

cmdStructs = readMessages(cmdBag, 'DataFormat','struct');
cmdTime = cmdBag.MessageList.Time;
cmdTime = cmdTime - cmdTime(1);

odomStructs = readMessages(odomBag, 'DataFormat','struct');
odomTime = extractTime(odomStructs, length(odomStructs));
odomTime = odomTime - odomTime(1);

CPStructs = readMessages(cpBag, 'DataFormat','struct');
cpTime = cpBag.MessageList.Time;
cpTime = cpTime - cpTime(1);

clear cmdBag; clear odomBag; clear cpBag; clear bagselect;

%% Odometry extraction

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

%% Command velocity extraction

cmdL = zeros(length(cmdStructs), 1);
cmdA = zeros(length(cmdStructs), 1);

for ii = 1:length(cmdStructs)
    
    cmdL(ii) = cmdStructs{ii}.Linear.X;
    cmdA(ii) = cmdStructs{ii}.Angular.Z;
end

%% Trajectory plot

figure
ax1 = axes;
scatter(odomX, odomY, [], odomTime)
colormap hsv
h = colorbar;
ylabel(h, 'Time [s]', 'FontSize', 16, 'Interpreter', 'LaTex')
grid on
ylim([-1 6.5])
xlim([-0.2 0.2])
xlabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
title('$Map\ odometry:\ x\ vs\ y$', 'Interpreter', 'LaTex', 'FontSize', 20)
h.TickLabelInterpreter = 'LaTex';
ax1.TickLabelInterpreter = 'LaTex';
ax1.FontSize = 16;
set(gcf,'color','w');
grid on


figure
sgtitle('\boldmath$Odometry$', 'FontSize', 20, 'Interpreter', 'LaTex')
ax3 = subplot(3,1,1);
plot(odomTime, odomX, 'k', 'LineWidth', 1.5)
xlim([odomTime(1) odomTime(end)])
% ylim([-6 6])
ylabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax3.TickLabelInterpreter = 'LaTex';
ax3.FontSize = 16;
grid on

ax4 = subplot(3,1,2);
plot(odomTime, odomY, 'b', 'LineWidth', 1.5)
xlim([odomTime(1) odomTime(end)])
ylim([-0.5 6.2])
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax4.TickLabelInterpreter = 'LaTex';
ax4.FontSize = 16;
grid on

ax5 = subplot(3,1,3);
plot(odomTime, odomAngle(:, 1), 'r', 'LineWidth', 1.5)
ylabel('$\theta\ [rad]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlim([odomTime(1) odomTime(end)])
ylim([-0.1 1.7])
ax5.TickLabelInterpreter = 'LaTex';
ax5.FontSize = 16;
grid on

set(gcf,'color','w');

%% Command velocity plot

figure
sgtitle('\boldmath$Imposed\ command\ velocities$', 'FontSize', 20, 'Interpreter', 'LaTex')

ax6 = subplot(2,1,1);
plot(cmdTime, cmdL, 'b', 'LineWidth', 1.5)
ylim([-0.1 0.3])
xlim([cmdTime(1) cmdTime(end)+2])
grid on
ylabel('$v\ [m/s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
ax6.TickLabelInterpreter = 'LaTex';
ax6.FontSize = 12;

ax7 = subplot(2,1,2);
plot(cmdTime, cmdA, 'r', 'LineWidth', 1.5)
ylim([-0.2 0.45])
xlim([cmdTime(1) cmdTime(end)+2])
grid on
ylabel('$\dot{\theta}\ [rad/s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
ax7.TickLabelInterpreter = 'LaTex';
ax7.FontSize = 12;
set(gcf,'color','w');


%% Centroid analysis

CPMat = zeros(2, length(cpTime));

for ii = 1:length(cpTime)
    
    CPMat(:, ii) = CPStructs{ii}.Covariance(1:2);

    if CPMat(:, ii) == [-1, -1]
        CPMat(:, ii) = [NaN, NaN];
    end
end

figure
ax8 = axes;
scatter(CPMat(1, :), CPMat(2, :), [], cpTime, 'filled', 'LineWidth', 2)
colormap jet
h = colorbar;
ylabel(h, 'Time [s]', 'FontSize', 16, 'Interpreter', 'LaTex')
grid on
xlim([0 640])
ylim([0 480])
title('$Centroid\ position$', 'Interpreter', 'LaTex', 'FontSize', 20)
xlabel('$x\ (camera\ frame)\ [pix]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ylabel('$y\ (camera\ frame)\ [pix]$', 'FontSize', 16, 'Interpreter', 'LaTex')
h.TickLabelInterpreter = 'LaTex';
ax8.TickLabelInterpreter = 'LaTex';
ax8.FontSize = 16;
set(gcf,'color','w');
grid on
axis ij
