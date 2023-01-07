clear
clc
close all

%%
bagselectIN = rosbag('ex_A1.bag');

scanBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/scan');
odomBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/odom');

scanStructs = readMessages(scanBag, 'DataFormat','struct');
scanTime = scanBag.MessageList.Time;

odomStructs = readMessages(odomBag, 'DataFormat','struct');
odomTime = odomBag.MessageList.Time;


%% Odometry

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

figure
ax1 = axes;
sgtitle('\boldmath$Linear\ odometry:\ x\ vs\ y$', 'FontSize', 16, 'Interpreter', 'LaTex')
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

% to unwrap
odomAngle(2701:3029,1) = odomAngle(2701:3029,1) - 2*odomAngle(2701,1);

figure
sgtitle('\boldmath$Angular\ odometry$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax2 = axes;
plot(odomTime, odomAngle(:,1), 'LineWidth', 2.5)
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ylabel('$\theta\ [rad]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlim([odomTime(1) odomTime(end)])
ax2.TickLabelInterpreter = 'LaTex';
ax2.FontSize = 16;
grid on
set(gcf,'color','w');


xVel = zeros(length(odomTime), 1);
yVel = zeros(length(odomTime), 1);
thVel = zeros(length(odomTime), 1);

for ii = 1 : length(odomTime) - 1
    dt = odomTime(ii + 1) - odomTime(ii);
    xVel(ii) = (odomX(ii + 1) - odomX(ii))/dt;
    yVel(ii) = (odomY(ii + 1) - odomY(ii))/dt;
    thVel(ii) = (odomAngle(ii + 1) - odomAngle(ii))/dt;
end

   
xVel = xVel(1:end-1);    
yVel = yVel(1:end-1); 
thVel = thVel(1:end-1);


xVel = movmean(xVel, 8);
yVel = movmean(yVel, 8);
thVel = movmean(thVel, 6);

figure
sgtitle('\boldmath$Estimated\ velocities$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax3 = subplot(3,1,1);
plot(odomTime(1:end-1), xVel, 'k', 'LineWidth', 2)
xlim([odomTime(1) odomTime(end-1)])
grid on
ylabel('$\dot{x}\ [m/s]$','FontSize', 12, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 12, 'Interpreter', 'LaTex')
ax3.TickLabelInterpreter = 'LaTex';
ax3.FontSize = 12;

ax4 = subplot(3,1,2);
plot(odomTime(1:end-1), yVel, 'r', 'LineWidth', 2)
xlim([odomTime(1) odomTime(end-1)])
grid on
ylabel('$\dot{y}\ [m/s]$','FontSize', 12, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 12, 'Interpreter', 'LaTex')
ax4.TickLabelInterpreter = 'LaTex';
ax4.FontSize = 12;

ax5 = subplot(3,1,3);
plot(odomTime(1:end-1), thVel, 'b', 'LineWidth', 2)
xlim([odomTime(1) odomTime(end-1)])
grid on
ylabel('$\dot{\theta}\ [m/s]$','FontSize', 12, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 12, 'Interpreter', 'LaTex')
ax5.TickLabelInterpreter = 'LaTex';
ax5.FontSize = 12;
ylim([-0.3 0.4])
set(gcf,'color','w');


CmdVelLon = (xVel.^2 + yVel.^2).^(1/2);
CmdVelAng = thVel;

figure
sgtitle('\boldmath$Estimated\ command\ velocity$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax6 = subplot(2,1,1);
plot(odomTime(1:end-1), CmdVelLon, 'r', 'LineWidth', 2)
xlim([odomTime(1) odomTime(end-1)])
grid on
title('$Longitudinal\ command\ velocity$', 'FontSize', 14, 'Interpreter', 'LaTex')
ylabel('$v\ [m/s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
ax6.TickLabelInterpreter = 'LaTex';
ax6.FontSize = 14;
xlim([odomTime(1) odomTime(end-1)])

ax7 = subplot(2,1,2);
plot(odomTime(1:end-1), CmdVelAng, 'b', 'LineWidth', 2)
grid on
title('$Angular\ command\ velocity$', 'FontSize', 14, 'Interpreter', 'LaTex')
ylabel('$\dot{\theta}\ [rad/s]$','FontSize', 14, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$','FontSize', 14, 'Interpreter', 'LaTex')
ax7.TickLabelInterpreter = 'LaTex';
ax7.FontSize = 14;
ylim([-0.4 0.4])
xlim([odomTime(1) odomTime(end-1)])
set(gcf,'color','w');

%% Velocity as vector tangent to trajectory

figure
ax8 = axes;
sgtitle('\boldmath$Velocity\ tangent\ to\ the\ trajectory$', 'FontSize', 16, 'Interpreter', 'LaTex')
plot(odomX, odomY, 'b', 'LineWidth', 1.5)
ax8.TickLabelInterpreter = 'LaTex';
ax8.FontSize = 16;
xlabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlim([-2.5 1])
ylim([-1 1])
set(gcf,'color','w');
grid on

hold on
h = quiver(odomX(1), odomY(1), xVel(1), yVel(1), 10);
h.Color = 'r';
h.LineWidth = 1.5;

for ii = 1:5:length(xVel)
     h.XData = odomX(ii);
     h.YData = odomY(ii);
     h.UData = xVel(ii);
     h.VData = yVel(ii);
     pause(0.01)
end


%% Scan
scanXMat = zeros(360, length(scanStructs));
scanYMat = zeros(360, length(scanStructs));
Angles = 1:1:360;
Angles = Angles*pi/180;
figure

for ii = 1:length(scanTime)

    scanX = (scanStructs{ii}.Ranges.').*cos(Angles);
    scanY = (scanStructs{ii}.Ranges.').*sin(Angles);
    scanXMat(:,ii) = scanX.*isfinite(scanX);
    scanYMat(:,ii) = scanY.*isfinite(scanY);
    scanX = scanXMat(:,ii);
    scanY = scanYMat(:,ii);
    plot(scanX,scanY,marker=".",LineStyle="none", MarkerSize=8)
    set(gca, 'xlim', [-4 4], 'ylim', [-4 4])
    
    if ii == length(scanTime)
        dt = scanTime(ii) - scanTime(ii - 1);
    else
        dt = scanTime(ii + 1) - scanTime(ii);
    end
%    pause(dt)

end


%% Accomodation of odom to scanTime

Xn = interp1(odomTime,odomX,scanTime);
Yn = interp1(odomTime,odomY,scanTime);
Thn = interp1(odomTime,odomAngle(:,1),scanTime);

%% Plot turtlebot moving in the scanned space

figure
ax9 = axes;
sgtitle('\boldmath$Turtlebot\ moving\ in\ scanned\ space$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax9.TickLabelInterpreter = 'LaTex';
ax9.FontSize = 16;
xlabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
set(gcf,'color','w');
grid on

% save a gif
% filename = 'SCANGIF.gif';

for ii = 1:length(scanTime)
    [~, POS] = min( (scanXMat(:, ii).^2 + scanYMat(:, ii).^2).^(1/2) );
    xMat = scanXMat(:,ii)*cos(Thn(ii)) - scanYMat(:,ii)*sin(Thn(ii)) + Xn(ii);
    yMat = scanXMat(:,ii)*sin(Thn(ii)) + scanYMat(:,ii)*cos(Thn(ii)) + Yn(ii);
    a = xMat(POS); b = yMat(POS);
    plot(xMat, yMat, 'b*', Xn(ii), Yn(ii), 'ro', [Xn(ii) a], [Yn(ii) b], 'k', 'LineWidth', 2)
    
    set(gca, 'xlim', [-4 4], 'ylim', [-4 4])

    if ii == length(scanTime)
        dt = scanTime(ii) - scanTime(ii - 1);
    else
        dt = scanTime(ii + 1) - scanTime(ii);
    end
    grid on

    ax9.TickLabelInterpreter = 'LaTex';
    ax9.FontSize = 16;
    xlabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
    ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')

%       save gif    
%       drawnow
%       frame = getframe(1);
%       im = frame2im(frame);
%       [imind,cm] = rgb2ind(im,256);
%       if ii == 1
%           imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
%       else
%           imwrite(imind,cm,filename,'gif','WriteMode','append');
%       end
    

%     pause(dt)
end

%% Evaluate minimum distance

MinDist = zeros(length(scanTime), 1);

figure

for ii = 1:length(scanTime)
    [MinDist(ii), POS] = min( (scanXMat(:, ii).^2 + scanYMat(:, ii).^2).^(1/2) );
    a = Xn(ii) + scanXMat(POS, ii);
    b = Yn(ii) + scanYMat(POS, ii);
    subplot(1,2,1)
    plot(scanXMat(:,ii) + Xn(ii), scanYMat(:,ii) + Yn(ii), 'b*', Xn(ii), Yn(ii), 'ro', [Xn(ii) a], [Yn(ii) b], 'k', 'LineWidth', 2)
    set(gca, 'xlim', [-4 4], 'ylim', [-4 4])

    subplot(1,2,2)
    plot(scanTime(1:ii), MinDist(1:ii))
    set(gca, 'xlim', [scanTime(1) scanTime(end)], 'ylim', [0.2 0.8])

    if ii == length(scanTime)
        dt = scanTime(ii) - scanTime(ii - 1);
    else
        dt = scanTime(ii + 1) - scanTime(ii);
    end
    pause(dt)
end

%% Plot only minimum distance

figure
ax10 = axes;
sgtitle('\boldmath$Minimum\ distance\ from\ obstacles$', 'FontSize', 16, 'Interpreter', 'LaTex')
plot(scanTime, MinDist, 'r', 'LineWidth', 2)
ax10.TickLabelInterpreter = 'LaTex';
ax10.FontSize = 16;
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ylabel('$distance\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
set(gcf,'color','w');
xlim([scanTime(1) scanTime(end)])
grid on
