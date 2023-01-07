clear
clc
close all

%% Load

load('PathGlobal2.mat')
bagselectIN = rosbag('FP2Pub2Perfect.bag');

% load('PathGlobalTest3.mat')
% bagselectIN = rosbag('FP2Test3Perfect.bag');

%% Analysis of odom

odomBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/odom');
odomStructs = readMessages(odomBag, 'DataFormat','struct');
odomTime = extractTime(odomStructs, length(odomStructs));
odomTime = odomTime - odomTime(1);

odomX = zeros(length(odomStructs), 1);
odomY = zeros(length(odomStructs), 1);
odomZ = zeros(length(odomStructs), 1);

odomOX = zeros(length(odomStructs), 1);
odomOY = zeros(length(odomStructs), 1);
odomOZ = zeros(length(odomStructs), 1);
odomOW = zeros(length(odomStructs), 1);

odomAngle = zeros(length(odomStructs), 3);

odomTLX = zeros(length(odomStructs), 1);
odomTLY = zeros(length(odomStructs), 1);

odomTAZ = zeros(length(odomStructs), 1);

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
cmdBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/cmd_vel');
cmdStructs = readMessages(cmdBag, 'DataFormat','struct');
cmdTime = cmdBag.MessageList.Time;
cmdTime = cmdTime - cmdTime(1);

cmdL = zeros(length(cmdStructs), 1);
cmdA = zeros(length(cmdStructs), 1);

for ii = 1:length(cmdStructs)
    
    cmdL(ii) = cmdStructs{ii}.Linear.X;
    cmdA(ii) = cmdStructs{ii}.Angular.Z;
end


%% TF extraction
tfBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/tf');
tfStructs = readMessages(tfBag, 'DataFormat','struct');

MOTra = zeros(length(tfStructs), 3);
MORot = zeros(length(tfStructs), 3);
MOtime = zeros(length(tfStructs), 1);
ii = 1;

for jj = 1:length(tfStructs)
    TMP = tfStructs{jj}.Transforms.Header;
    con = strcmp(TMP.FrameId, 'map');
    if con

        MOTra(ii, :) = [tfStructs{jj}.Transforms.Transform.Translation.X, tfStructs{jj}.Transforms.Transform.Translation.Y, ...
            tfStructs{jj}.Transforms.Transform.Translation.Z];

        tmp = [tfStructs{jj}.Transforms.Transform.Rotation.W, tfStructs{jj}.Transforms.Transform.Rotation.X, ...
            tfStructs{jj}.Transforms.Transform.Rotation.Y, tfStructs{jj}.Transforms.Transform.Rotation.Z];
        MORot(ii, :) = quat2eul(tmp);

        sec = cast(tfStructs{jj}.Transforms.Header.Stamp.Sec, 'double');
        nsec = cast(tfStructs{jj}.Transforms.Header.Stamp.Nsec, 'double');
        MOtime(ii, 1) = sec + nsec*1e-9;

        ii = ii + 1;
    end
end

MOTra = MOTra(1:ii - 1, :);
MORot = MORot(1:ii - 1, :);
MOtime = MOtime(1:ii - 1, :);
MOtime = MOtime - MOtime(1);


%% Path comparison

odomX    = odomX(tSel1:end);
odomY    = odomY(tSel1:end);
odomTime = odomTime(tSel1:end) - odomTime(tSel1);
figure
ax1 = axes;
plot(XYpathGlob(:, 1), XYpathGlob(:, 2), 'k', 'LineWidth', 2)
hold on
scatter(odomX, odomY, [], odomTime);
grid on
xlabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
colormap hsv
h = colorbar;
ylabel(h, '$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
h.TickLabelInterpreter = 'LaTex';
ax1.TickLabelInterpreter = 'LaTex';
ax1.FontSize = 16;
set(gcf,'color','w');
sgtitle('\boldmath$Odometry\ comparison$', 'FontSize', 16, 'Interpreter', 'LaTex')
grid on
legend('Theoretical', 'Simulated', 'interpreter', 'latex')


%% Odom: x vs y vs theta

figure
sgtitle('\boldmath$Odometry$', 'FontSize', 20, 'Interpreter', 'LaTex')

ax3 = subplot(3,1,1);
plot(odomTime, odomX, 'k', 'LineWidth', 1.5)
xlim([odomTime(1) odomTime(end)])
ylim([min(odomX) - 0.7 max(odomX) + 0.7])
ylabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax3.TickLabelInterpreter = 'LaTex';
ax3.FontSize = 16;
grid on

ax4 = subplot(3,1,2);
plot(odomTime, odomY, 'b', 'LineWidth', 1.5)
xlim([odomTime(1) odomTime(end)])
ylim([min(odomY) - 0.7 max(odomY) + 0.7])
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax4.TickLabelInterpreter = 'LaTex';
ax4.FontSize = 16;
grid on

ax5 = subplot(3,1,3);
plot(odomTime, odomAngle(tSel1:end, 1), 'r', 'LineWidth', 1.5)
ylabel('$\theta\ [rad]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlim([odomTime(1) odomTime(end)])
ax5.TickLabelInterpreter = 'LaTex';
ax5.FontSize = 16;
grid on

set(gcf,'color','w');

%% Command velocity plot
cmdL    = cmdL(tSel2:end);
cmdA    = cmdA(tSel2:end);
cmdTime = cmdTime(tSel2:end) - cmdTime(tSel2);

figure
sgtitle('\boldmath$Imposed\ command\ velocities$', 'FontSize', 20, 'Interpreter', 'LaTex')

ax6 = subplot(2,1,1);
plot(cmdTime, cmdL, 'b', 'LineWidth', 1.5)
ylim([-0.1 0.3])
xlim([cmdTime(1) cmdTime(end)])
grid on
ylabel('$v\ [m/s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
ax6.TickLabelInterpreter = 'LaTex';
ax6.FontSize = 12;

ax7 = subplot(2,1,2);
plot(cmdTime, cmdA, 'r', 'LineWidth', 1.5)
ylim([-0.6 0.6])
xlim([cmdTime(1) cmdTime(end)])
grid on
ylabel('$\dot{\theta}\ [rad/s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
ax7.TickLabelInterpreter = 'LaTex';
ax7.FontSize = 12;

set(gcf,'color','w');


%% TF plot
figure
sgtitle('\boldmath$Odometry\ frame\ with\ respect\ to\ map\ frame$', 'FontSize', 20, 'Interpreter', 'LaTex')

ax3 = subplot(2,1,1);
plot(MOtime, MOTra(:, 1), 'k', MOtime, MOTra(:, 2), 'r', MOtime, MOTra(:, 3), 'b', 'LineWidth', 1.5)
title('Position: cartesian coordinates', 'FontSize', 16, 'Interpreter', 'LaTex')
xlim([MOtime(1) MOtime(end)])
ylabel('$-\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
legend('x', 'y', 'z', 'Interpreter', 'LaTex')
ax3.TickLabelInterpreter = 'LaTex';
ax3.FontSize = 16;
grid on

ax4 = subplot(2,1,2);
plot(MOtime, MORot(:, 1), 'k', MOtime, MORot(:, 2), 'r', MOtime, MORot(:, 3), 'b', 'LineWidth', 1.5)
title('Orientation: Euler angles', 'FontSize', 16, 'Interpreter', 'LaTex')
xlim([MOtime(1) MOtime(end)])
ylabel('$-\ [rad]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
legend('Z', 'Y', 'Z', 'Interpreter', 'LaTex')
ax4.TickLabelInterpreter = 'LaTex';
ax4.FontSize = 16;
grid on

set(gcf,'color','w');