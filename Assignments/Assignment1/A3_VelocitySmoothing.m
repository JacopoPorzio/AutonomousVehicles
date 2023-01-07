clear
clc
close all

%% Load no averaged velocity

% load('PublishingNEW.mat')


%% Load 
% 
load('PublishingSmooth.mat')

cmdTime = odomTime(1:end-1);
CVL = CmdVelLon;
CVA = CmdVelAng;

%% Plot of noisy/smoothed velocity

figure
plot(cmdTime, CVL, 'b')
xlim([cmdTime(1) cmdTime(end)])

CVLbis = CVL;

hold on
plot(cmdTime, CVLbis, 'r')
grid on

%% Correction of CVL

% first, calculate first derivative
CVL = CVLbis;
accT = cmdTime(1:end);
accL = zeros(length(accT), 1);

for ii = 1:length(accT)-1
    dt = accT(ii + 1) - accT(ii);
    accL(ii) = (CVL(ii + 1) - CVL(ii))/dt;
end

figure
plot(accT, accL)
xlim([accT(1) accT(end)])

accLlow = lowpass(accL, 0.0001, 30);

figure
plot(accLlow)
hold on

[PosP, PosL] = findpeaks(accLlow, 'MinPeakHeight', 0.011);
[NegP, NegL] = findpeaks(-accLlow, 'MinPeakHeight', 0.011);
NegP = - NegP;

peaks = [PosP; NegP];
peaksLoc = [PosL; NegL];

[peaksLoc, indL] = sort(peaksLoc);

peaks = peaks(indL);

plot(peaksLoc, peaks, '*r')

CVLtris = CVLbis;

for kk = 1:length(peaksLoc)-1
   
    a = peaksLoc(kk) + 2; b = peaksLoc(kk + 1) - 2;

    CVLtris(a:b) = mean(CVLtris(a:b))*ones(length(CVLtris(a:b)), 1);

end

figure
plot(cmdTime, CVLtris, 'r')
hold on

%% Plot of noisy/smoothed angular velocity

figure
plot(cmdTime, CVA, 'b')
xlim([cmdTime(1) cmdTime(end)])

CVAbis = CVA;

hold on
plot(cmdTime, CVAbis, 'r')
grid on

%% Correction of CVA

% first, calculate first derivative
CVA = CVAbis;
accT = cmdTime(1:end);
accA = zeros(length(accT), 1);

for ii = 1:length(accT)-1
    dt = accT(ii + 1) - accT(ii);
    accA(ii) = (CVA(ii + 1) - CVA(ii))/dt;
end

figure
plot(accT, accA)
xlim([accT(1) accT(end)])

accLlow = lowpass(accA, 0.0001, 30);

figure
plot(accLlow)
hold on

[PosP, PosL] = findpeaks(accLlow, 'MinPeakHeight', 0.09);
[NegP, NegL] = findpeaks(-accLlow, 'MinPeakHeight', 0.09);
NegP = - NegP;

peaks = [PosP; NegP];
peaksLoc = [PosL; NegL];

[peaksLoc, indL] = sort(peaksLoc);

peaks = peaks(indL);

plot(peaksLoc, peaks, '*r')

CVAtris = CVAbis;

for kk = 1:length(peaksLoc)-1
    
    a = peaksLoc(kk) + 2; b = peaksLoc(kk + 1) - 2;

    CVAtris(a:b) = mean(CVAtris(a:b))*ones(length(CVAtris(a:b)), 1);

end

figure
plot(cmdTime, CVAtris, 'r', 'LineWidth', 2)
hold on


%% Definition of velocities to publish
tO = odomTime(1:end-1);
dt = 1/10;
tN = (tO(1):dt:tO(end)).' - tO(1);

VLN = CVLtris(1:3:length(CVLtris));
VAN = CVAtris(1:3:length(CVAtris));

figure
subplot(2,1,1)
plot(cmdTime - cmdTime(1), CVL, 'r')
hold on
plot(tN, VLN, 'b')

subplot(2,1,2)
plot(cmdTime - cmdTime(1), CVA, 'r')
hold on
plot(tN, VAN, 'b')


