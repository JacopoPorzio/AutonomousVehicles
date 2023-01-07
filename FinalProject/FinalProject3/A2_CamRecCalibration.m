% the present script is used to calibrate the camera,
% in order to measure area


clear
clc
close all

l1 = 0.5;
l2 = 0.8;
l3 = 1.8;
l4 = 2.8;

d = 0.4;

%% Reference is image seen from 0.5 (m)
lr = l1;
D1 = l1/lr*d;
A1 = pi*(D1/2)^2;
D2 = l2/lr*d;
A2 = pi*(D2/2)^2;
D3 = l3/lr*d;
A3 = pi*(D3/2)^2;
D4 = l4/lr*d;
A4 = pi*(D4/2)^2;

%% Reference is image seen from 0.8 (m)
lr = l2;
D1 = l1/lr*d;
A1 = pi*(D1/2)^2;
D2 = l2/lr*d;
A2 = pi*(D2/2)^2;
D3 = l3/lr*d;
A3 = pi*(D3/2)^2;
D4 = l4/lr*d;
A4 = pi*(D4/2)^2;

%% Reference is image seen from 1.8 (m)
lr = l3;
D1 = l1/lr*d;
A1 = pi*(D1/2)^2;
D2 = l2/lr*d;
A2 = pi*(D2/2)^2;
D3 = l3/lr*d;
A3 = pi*(D3/2)^2;
D4 = l4/lr*d;
A4 = pi*(D4/2)^2;

%% Reference is image seen from 2.8 (m)
lr = l4;
D1 = l1/lr*d;
A1 = pi*(D1/2)^2;
D2 = l2/lr*d;
A2 = pi*(D2/2)^2;
D3 = l3/lr*d;
A3 = pi*(D3/2)^2;
D4 = l4/lr*d;
A4 = pi*(D4/2)^2;

