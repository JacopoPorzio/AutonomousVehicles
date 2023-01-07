clear
clc
close all

%% Load

% % % The two I'd consider to be published
% % % Only the second one has to be

% load('PathToPub2.mat')
load('PathToPubTest3.mat') % let's see tomorrow if the consecutive waypoints are a problem

map_rgb = imread('veryFineMap2.pgm');
BW = imbinarize(map_rgb,0.60);

%% PlotMap
% 
% plot_map_mod(BW)
% xlim([0 size(BW, 1)])
% ylim([0 size(BW, 2)])

%% Creation of local metric path
res = 0.05;             % [m/pix]
XYpathLoc = res*XYpath(2:end, :);

%% Global path definition

lx = 10;
ly = res*size(BW, 2) - 10;

XYpathGlob = zeros(length(XYpathLoc), 2);

XYpathGlob(:, 1) = - lx + XYpathLoc(:, 1);
XYpathGlob(:, 2) =   ly - XYpathLoc(:, 2);

figure
scatter(XYpathGlob(:, 1), XYpathGlob(:, 2))
grid on
axis equal

%% Selection of a subset of waypoints, automatic way
figure
subplot(2, 1, 1)
plot(XYpathGlob(:, 1), '*r', 'LineWidth', 1.5)
grid on
subplot(2, 1, 2)
plot(XYpathGlob(:, 2), '*b', 'LineWidth', 1.5)
grid on
sgtitle('Pos')

MPH = 1;

path = XYpathGlob;

dXp = diff(path(:, 2))./diff(path(:, 1));
dYp = 1./dXp;

figure
subplot(2, 1, 1)
plot(dXp, '*r', 'LineWidth', 1.5)
grid on
subplot(2, 1, 2)
plot(dYp, '*b', 'LineWidth', 1.5)
grid on
sgtitle('d')

ddXp = diff(dXp)./diff(path(1:end-1, 1));
ddYp = diff(dYp)./diff(path(1:end-1, 2));

figure
subplot(2, 1, 1)
plot(ddXp, '*r', 'LineWidth', 1.5)
grid on
subplot(2, 1, 2)
plot(ddYp, '*b', 'LineWidth', 1.5)
grid on
sgtitle('d^{2}')

ddXp(~isfinite(ddXp)) = 20*sign(ddXp(~isfinite(ddXp)));
ddYp(~isfinite(ddYp)) = 20*sign(ddXp(~isfinite(ddYp)));

[~, xL] = findpeaks(abs(ddXp), 'MinPeakHeight', MPH);
[~, yL] = findpeaks(abs(ddYp), 'MinPeakHeight', MPH);

msV = sort(1 + [xL.', yL.']);

msV = msV([diff(msV) ~= 0, true]);

selVec = [1, msV, length(path)];

% selVec = [selVec(1) selVec(3:end)]; % ONLY for Pub1

path = path(selVec, :); % what we want to publish

figure
ax1 = axes;
plot(XYpathGlob(:, 1), XYpathGlob(:, 2), '*r', 'LineWidth', 1.5)
hold on
plot(path(:, 1), path(:, 2), '*b', 'LineWidth', 1.5)
grid on
xlabel('$x\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ylabel('$y\ [m]$', 'FontSize', 16, 'Interpreter', 'LaTex')
ax1.TickLabelInterpreter = 'LaTex';
ax1.FontSize = 16;
set(gcf,'color','w');
sgtitle('\boldmath$Path\ comparison$', 'FontSize', 16, 'Interpreter', 'LaTex')
grid on
legend('Complete path', 'Used waypoints for feedback', 'interpreter', 'latex')

% save('PathGlobal2.mat', 'XYpathGlob')
% save('PathGlobalTest3.mat', 'XYpathGlob')


%% Angles addition

IND = 1:length(path) - 1;

thV = angle((path(IND + 1, 1) - path(IND, 1)) + 1i*(path(IND + 1, 2) - path(IND, 2)));
thV(end + 1) = thV(end);

pathAng = [path thV];
