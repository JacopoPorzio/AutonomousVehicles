clear
clc
close all

%% Load of file

load('FirstRun.mat')

time = out.tout;
gPos = out.GatePosition;
vPos = out.VehiclePosition;


%% Plot

figure
sgtitle('\boldmath$Parking\ gate\ and\ vehicle\ position$', 'FontSize', 20, 'Interpreter', 'LaTex')

ax1 = subplot(2,1,1);
plot(time, gPos*180/pi, 'k', 'LineWidth', 1.5)
xlim([time(1) time(end)])
ylabel('$Gate\ angular\ position\ [deg]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax1.TickLabelInterpreter = 'LaTex';
ax1.FontSize = 16;
grid on

ax2 = subplot(2,1,2);
plot(time, vPos, 'b', 'LineWidth', 1.5)
xlim([time(1) time(end)])
ylabel('$Vehicle\ position\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
xlabel('$t\ [s]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax2.TickLabelInterpreter = 'LaTex';
ax2.FontSize = 16;
grid on

set(gcf,'color','w');

