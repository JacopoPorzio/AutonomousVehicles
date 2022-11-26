clear
clc
close all

%%
bagselectIN = rosbag('ex_A1.bag');

% clockBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/clock');
% clockStructs = readMessages(clockBag, 'DataFormat','struct');

scanBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/scan');
odomBag = select(bagselectIN, 'Time', [bagselectIN.StartTime bagselectIN.EndTime], 'Topic', '/odom');

scanStructs = readMessages(scanBag, 'DataFormat','struct');
scanTime = scanBag.MessageList.Time;

odomStructs = readMessages(odomBag, 'DataFormat','struct');
odomTime = odomBag.MessageList.Time;

% from now on, I should do a thing like
%
% in a for loop, take odomStructs{ii}.Pose.Pose.X, extract X and put it in
% a vector (same for Y and Z)
% for the scan thing, should we use function used in Lec2ExampleModel?

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

    % do I need these variables?
    odomTLX(ii) = odomStructs{ii}.Twist.Twist.Linear.X;
    odomTLY(ii) = odomStructs{ii}.Twist.Twist.Linear.Y;

    odomTAZ(ii) = odomStructs{ii}.Twist.Twist.Angular.Z;


end

testAngle = atan(odomTLY./odomTLX);

% figure
% sgtitle('$Linear\ Odometry$', 'Interpreter', 'LaTex')
% subplot(1,3,1)
% plot(odomTime, odomX)
% grid on
% title('$x$', 'Interpreter', 'LaTex')
% subplot(1,3,2)
% plot(odomTime, odomY)
% grid on
% title('$y$', 'Interpreter', 'LaTex')
% subplot(1,3,3)
% plot(odomTime, odomZ)
% grid on
% title('$z$', 'Interpreter', 'LaTex')

figure
title('$Linear\ Odometry:\ x\ vs\ y$', 'Interpreter', 'LaTex')
% plot(odomX, odomY, 'LineWidth', 2)
scatter(odomX, odomY, [], odomTime)
colormap copper
h = colorbar;
ylabel(h, 'Time')
grid on

% odomAngle(2701:3029,1) = odomAngle(2701:3029,1)*(-1);
odomAngle(2701:3029,1) = odomAngle(2701:3029,1) - 2*odomAngle(2701,1);

figure
sgtitle('$Angular\ Odometry$', 'Interpreter', 'LaTex')
subplot(3,1,1)
plot(odomTime, odomAngle(:,1))
grid on
subplot(3,1,2)
plot(odomTime, odomAngle(:,2))
grid on
subplot(3,1,3)
plot(odomTime, odomAngle(:,3))
grid on

xVel = zeros(length(odomTime), 1); % -1
yVel = zeros(length(odomTime), 1);
thVel = zeros(length(odomTime), 1);

for ii = 1 : length(odomTime) - 1
    dt = odomTime(ii + 1) - odomTime(ii);
    xVel(ii) = (odomX(ii + 1) - odomX(ii))/dt;
    yVel(ii) = (odomY(ii + 1) - odomY(ii))/dt;
    thVel(ii) = (odomAngle(ii + 1) - odomAngle(ii))/dt;
%     thVel(ii) = (testAngle(ii + 1) - testAngle(ii))/dt;
end

% xVelI = zeros(length(odomTime), 1); % -1
% yVelI = zeros(length(odomTime), 1);
% thVelI = zeros(length(odomTime), 1);
% 
% for ii = 2 : length(odomTime)
%     dt = odomTime(ii) - odomTime(ii - 1);
%     xVelI(ii) = (odomX(ii) - odomX(ii - 1))/dt;
%     yVelI(ii) = (odomY(ii) - odomY(ii - 1))/dt;
%     thVelI(ii) = (odomAngle(ii) - odomAngle(ii - 1))/dt;
% end
% 
% xVelC = zeros(length(odomTime), 1); % -2
% yVelC = zeros(length(odomTime), 1);
% thVelC = zeros(length(odomTime), 1);
% 
% for ii = 2 : length(odomTime) - 1
%     dt = odomTime(ii + 1) - odomTime(ii - 1);
%     xVelC(ii) = (odomX(ii + 1) - odomX(ii - 1))/dt;
%     yVelC(ii) = (odomY(ii + 1) - odomY(ii - 1))/dt;
%     thVelC(ii) = (odomAngle(ii + 1) - odomAngle(ii - 1))/dt;
% end
% 
% xVel = (xVel + xVelI + xVelC)/3;     
xVel = xVel(1:end-1);    %xVel = lowpass(xVel, 10, 30);
% yVel = (yVel + yVelI + yVelC)/3;     
yVel = yVel(1:end-1);    %yVel = lowpass(yVel, 10, 30);
% thVel = (thVel + thVelI + thVelC)/3; 
thVel = thVel(1:end-1);  %thVel = lowpass(thVel, 10, 30);


xVel = movmean(xVel, 8);
yVel = movmean(yVel, 8);
thVel = movmean(thVel, 6);

% xVel = robustDiff(odomTLX, 1/30, 5);  xVel = xVel(1:end-1);
% yVel = robustDiff(odomTLY, 1/30, 5);  yVel = yVel(1:end-1);
% thVel = robustDiff(odomTAZ, 1/30, 5); thVel = thVel(1:end-1);

figure
sgtitle('$Estimated\ velocities$', 'Interpreter', 'LaTex')
subplot(3,1,1)
plot(odomTime(1:end-1), xVel)
grid on
title('$x\ direction$', 'Interpreter', 'LaTex')

subplot(3,1,2)
plot(odomTime(1:end-1), yVel)
grid on
title('$y\ direction$', 'Interpreter', 'LaTex')

subplot(3,1,3)
plot(odomTime(1:end-1), thVel)
grid on
title('$\theta\ direction$', 'Interpreter', 'LaTex')
ylim([-0.3 0.4])

CmdVelLon = (xVel.^2 + yVel.^2).^(1/2); % this is twistLX
CmdVelAng = thVel;                      % this is twistAZ

figure
subplot(2,1,1)
plot(odomTime(1:end-1), CmdVelLon)
hold on
plot(odomTime, odomTLX, 'LineWidth', 2)
grid on
legend('Estimated', 'Twist')

subplot(2,1,2)
plot(odomTime(1:end-1), CmdVelAng)
hold on
plot(odomTime, odomTAZ, 'LineWidth', 2)
grid on
ylim([-0.4 0.4])
legend('Estimated', 'Twist')

%% Velocity as vector tangent to trajectory

figure
plot(odomX, odomY, 'b', 'LineWidth', 1.5)
xlim([-2.5 1])
ylim([-1 1])
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
% to get the minimum distance, I now have to understand the position of the
% robot in the scanned space: should we try to plot (x,y) position of the
% robot in the plot of scanned space?

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


%% attempt to plot scan and odom
% they have nothing in common

Xn = interp1(odomTime,odomX,scanTime);
Yn = interp1(odomTime,odomY,scanTime);
Thn = interp1(odomTime,odomAngle(:,1),scanTime);

if 0
    figure
    for ii = 1:length(scanTime)
%         plot(-scanXMat(:,ii), scanYMat(:,ii), 'b*', Xn(ii), Yn(ii), 'r*', 'LineWidth', 2) % minus sign as test
        plot(scanXMat(:,ii), scanYMat(:,ii), 'b*')
        set(gca, 'xlim', [-4 4], 'ylim', [-4 4])
    
        if ii == length(scanTime)
            dt = scanTime(ii) - scanTime(ii - 1);
        else
            dt = scanTime(ii + 1) - scanTime(ii);
        end
        pause(dt)
    end
end

%% Plot turtlebot moving in the scanned space

figure

for ii = 1:length(scanTime)
%     plot(scanXMat(:,ii) + Xn(ii), scanYMat(:,ii) + Yn(ii), 'b*', Xn(ii), Yn(ii), 'r*', 'LineWidth', 2)
    [~, POS] = min( (scanXMat(:, ii).^2 + scanYMat(:, ii).^2).^(1/2) );
    xMat = scanXMat(:,ii)*cos(Thn(ii)) - scanYMat(:,ii)*sin(Thn(ii)) + Xn(ii);
    yMat = scanXMat(:,ii)*sin(Thn(ii)) + scanYMat(:,ii)*cos(Thn(ii)) + Yn(ii);
    a = xMat(POS); b = yMat(POS);
%     plot(xMat, yMat, 'b*', Xn(ii), Yn(ii), 'r*', 'LineWidth', 2)
    plot(xMat, yMat, 'b*', Xn(ii), Yn(ii), 'ro', [Xn(ii) a], [Yn(ii) b], 'k', 'LineWidth', 2)
    
    set(gca, 'xlim', [-4 4], 'ylim', [-4 4])

    if ii == length(scanTime)
        dt = scanTime(ii) - scanTime(ii - 1);
    else
        dt = scanTime(ii + 1) - scanTime(ii);
    end
    pause(dt)
end

%% Evaluate minimum distance

MinDist = zeros(length(scanTime), 1);
% XVn = interp1(odomTime(1:end-1),xVel,scanTime);
% YVn = interp1(odomTime(1:end-1),yVel,scanTime);

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

%% Check accuracy of estimated /cmd_vel
%
% create a bag with estimated /cmd_vel only inside
% passare direttamente da pubblicazione?
% 
