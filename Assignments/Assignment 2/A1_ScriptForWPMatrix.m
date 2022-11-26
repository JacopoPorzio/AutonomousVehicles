clear
clc
close all

%%
WaypointList = [ 5,  0, pi/2;
                 5,  5, pi*5/4;
                -5, -5, pi/2;
                -5,  5, 0;
                 0,  0, 0;
                 3,  3, pi*3/4;
                -3,  0, pi*3/2;
                 0, -3, pi/4;
                 3,  0, pi/2;
                 0,  0, pi*3/2];

% save('WList.mat', 'WaypointList')

% following if:
%              - quat2eul w/o mods;
%              - Gr = angle(z) always.
% WaypointList = [ 5,  0, pi/2;
%                  5,  5, pi*5/4 - 2*pi;
%                 -5, -5, pi/2;
%                 -5,  5, 0;
%                  0,  0, 0;
%                  3,  3, pi*3/4;
%                 -3,  0, pi*3/2 - 2*pi;
%                  0, -3, pi/4;
%                  3,  0, pi/2;
%                  0,  0, pi*3/2 - 2*pi];

%% Considerations
% in order to debug some strange behaviours (heading changing in a strange
% way et cetera), let's record a bag and study /odom and /cmd_vel
%
% a problem of the orientation could be due to excessive approaching velocity 
% remember the extra point!!!