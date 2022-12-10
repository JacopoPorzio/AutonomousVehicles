clear
clc
close all

% strange: for some k, it can happen that timeX(k) = timeX(k+1)
% even if X(k) ~= X(k+1)
% this happens especially when dt is small

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
odomTime = odomBag.MessageList.Time;
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
scatter(odomX, odomY, [], odomTime)
colormap hsv
h = colorbar;
ylabel(h, 'Time')
grid on
ylim([-1 6])
xlim([-0.25 0.25])
% axis equal
title('$Map\ odometry:\ x\ vs\ y$', 'Interpreter', 'LaTex', 'FontSize', 20)

figure
subplot(3, 1, 1)
plot(odomTime, odomX, 'k', 'LineWidth', 1.5)
title('$x$', 'Interpreter', 'LaTex')
grid on

subplot(3, 1, 2)
plot(odomTime, odomY, 'b', 'LineWidth', 1.5)
title('$y$', 'Interpreter', 'LaTex')
grid on

subplot(3, 1, 3)
plot(odomTime, odomAngle(:, 1), 'r', 'LineWidth', 1.5)
title('$\theta$', 'Interpreter', 'LaTex')
grid on

sgtitle('$x,\ y,\ \theta\ odometry$', 'Interpreter', 'LaTex', 'FontSize', 20)

%% Command velocity plot

figure
subplot(2, 1, 1)
plot(cmdTime, cmdL, 'b', 'LineWidth', 1.5)
title('$Imposed\ linear\ velocity$', 'Interpreter', 'LaTex', 'FontSize', 15)
% ylim([-0.1 0.3])
grid on

subplot(2, 1, 2)
plot(cmdTime, cmdA, 'r', 'LineWidth', 1.5)
title('$Imposed\ angular\ velocity$', 'Interpreter', 'LaTex', 'FontSize', 15)
% ylim([-0.6 0.6])
grid on

%% Centroid analysis

CPMat = zeros(2, length(cpTime));

for ii = 1:length(cpTime)
    
    CPMat(:, ii) = CPStructs{ii}.Covariance(1:2);

    if CPMat(:, ii) == [-1, -1]
        CPMat(:, ii) = [NaN, NaN];
    end
end

% CPMatP = [640*ones(1, length(cpTime)) - CPMat(1, :);
%          480*ones(1, length(cpTime)) - CPMat(2, :)];

figure
scatter(CPMat(1, :), CPMat(2, :), [], cpTime)
colormap hsv
% select another colormap
h = colorbar;
ylabel(h, 'Time')
grid on
xlim([0 640])
ylim([0 480])
title('$Centroid\ position$', 'Interpreter', 'LaTex', 'FontSize', 20)
