clear
clc
close all

%% Bag loading

% bagselect = rosbag('EmptyWorldFirst.bag');

% bagselect = rosbag('EmptyWorldSecond.bag');

% bagselect = rosbag('FastBagNotPerfect.bag');

bagselect = rosbag('FastBag.bag');

cmdBag = select(bagselect, 'Time', [bagselect.StartTime bagselect.EndTime], 'Topic', '/cmd_vel');
odomBag = select(bagselect, 'Time', [bagselect.StartTime bagselect.EndTime], 'Topic', '/odom');

cmdStructs = readMessages(cmdBag, 'DataFormat','struct');
cmdTime = cmdBag.MessageList.Time;
cmdTime = cmdTime - cmdTime(1);

odomStructs = readMessages(odomBag, 'DataFormat','struct');
odomTime = odomBag.MessageList.Time;
odomTime = odomTime - odomTime(1);

%% Waypoint list loading

load('WList.mat')

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

% figure
% scatter(odomX, odomY, [], odomTime)
% colormap hsv
% h = colorbar;
% ylabel(h, 'Time')
figure
% sgtitle('\boldmath$Linear\ odometry:\ x\ vs\ y$', 'FontSize', 16, 'Interpreter', 'LaTex')
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

hold on

scatter(WaypointList(:,1), WaypointList(:,2), '*k', 'LineWidth', 8)

for ii = 1:length(WaypointList)
    hold on
    a = WaypointList(ii,1);
    b = a + 1*cos(WaypointList(ii,3));
    c = WaypointList(ii,2);
    d = c + 1*sin(WaypointList(ii,3));
    plot([a, b], [c, d], 'r', 'LineWidth', 3)
%     h = quiver(a, c, b, d);
%     h.Color = 'r';
%     h.LineWidth = 1.5;
end

title('\boldmath$Map\ odometry:\ x\ vs\ y$', 'Interpreter', 'LaTex', 'FontSize', 20)
xticks(-5.5:1.5:5.5)
yticks(-5.5:1.5:5.5)
% xlim([-5.5 5.5])
% ylim([-5.5 5.5])
xlim([-6 6])
ylim([-6 6])
grid on

figure
sgtitle('\boldmath$Odometry$', 'FontSize', 20, 'Interpreter', 'LaTex')

ax3 = subplot(3,1,1);
plot(odomTime, odomX, 'k', 'LineWidth', 1.5)
% title('$x$', 'Interpreter', 'LaTex')
xlim([odomTime(1) odomTime(end)])
ylim([-6 6])
ylabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax3.TickLabelInterpreter = 'LaTex';
ax3.FontSize = 16;
grid on

ax4 = subplot(3,1,2);
plot(odomTime, odomY, 'b', 'LineWidth', 1.5)
% title('$y$', 'Interpreter', 'LaTex')
xlim([odomTime(1) odomTime(end)])
ylim([-6 6])
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax4.TickLabelInterpreter = 'LaTex';
ax4.FontSize = 16;
grid on

ax5 = subplot(3,1,3);
plot(odomTime, odomAngle(:, 1), 'r', 'LineWidth', 1.5)
% title('$\theta$', 'Interpreter', 'LaTex')
ylabel('$\theta\ [rad]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlim([odomTime(1) odomTime(end)])
ax5.TickLabelInterpreter = 'LaTex';
ax5.FontSize = 16;
grid on

set(gcf,'color','w');


%% Command velocity plot

figure
sgtitle('\boldmath$Imposed\ command\ velocities$', 'FontSize', 20, 'Interpreter', 'LaTex')

ax6 = subplot(2,1,1);
plot(cmdTime, cmdL, 'b', 'LineWidth', 1.5)
% title('$Imposed\ linear\ velocity$', 'Interpreter', 'LaTex', 'FontSize', 15)
ylim([-0.1 0.3])
xlim([cmdTime(1) cmdTime(end)])
grid on
ylabel('$v\ [m/s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
ax6.TickLabelInterpreter = 'LaTex';
ax6.FontSize = 12;

ax7 = subplot(2,1,2);
plot(cmdTime, cmdA, 'r', 'LineWidth', 1.5)
% title('$Imposed\ angular\ velocity$', 'Interpreter', 'LaTex', 'FontSize', 15)
ylim([-0.6 0.6])
xlim([cmdTime(1) cmdTime(end)])
grid on
ylabel('$\dot{\theta}\ [rad/s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
ax7.TickLabelInterpreter = 'LaTex';
ax7.FontSize = 12;

set(gcf,'color','w');
