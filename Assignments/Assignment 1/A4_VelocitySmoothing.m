clear
clc
close all

%%
load('Analysis.mat')

k = 6;

xVel = zeros(length(odomTime), 1);
yVel = zeros(length(odomTime), 1);
thVel = zeros(length(odomTime), 1);

for ii = 1 : length(odomTime) - 1
    dt = odomTime(ii + 1) - odomTime(ii);
    xVel(ii) = (odomX(ii + 1) - odomX(ii))/dt;
    yVel(ii) = (odomY(ii + 1) - odomY(ii))/dt;
    thVel(ii) = (odomAngle(ii + 1) - odomAngle(ii))/dt;
end

xVelN = movmean(xVel, k);
yVelN = movmean(yVel, k);
thVelN = movmean(thVel, k);

figure
subplot(3,1,1)
plot(odomTime, xVel, 'r', odomTime, xVelN, 'b')
grid on
legend('Original', 'Smoothed')

subplot(3,1,2)
plot(odomTime, yVel, 'r', odomTime, yVelN, 'b')
grid on
legend('Original', 'Smoothed')

subplot(3,1,3)
plot(odomTime, thVel, 'r', odomTime, thVelN, 'b')
grid on
legend('Original', 'Smoothed')