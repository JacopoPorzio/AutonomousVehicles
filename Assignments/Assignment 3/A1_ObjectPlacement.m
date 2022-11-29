clear
clc
close all

%% Initialization
% rosinit
gazebo = ExampleHelperGazeboCommunicator;

%% Color conversion

RBallVec = [200 48 48];
RBallVec = RBallVec./255;

PBallVec = [200 0 103];
PBallVec = PBallVec./255;

%% add objects in GAZEBO

RBall = ExampleHelperGazeboModel("Ball")
sphereLink = addLink(RBall,"sphere",0.1,"color",[RBallVec 1]) %type, rad, color
spawnModel(gazebo,RBall,[0 6 0.1])

PBall = ExampleHelperGazeboModel("Ball2")
sphereLink = addLink(PBall,"sphere",0.2,"color",[PBallVec 1]) %type, rad, color
spawnModel(gazebo,PBall,[6 0 0.1])

% BBall = ExampleHelperGazeboModel("Ball3")
% sphereLink = addLink(BBall,"sphere",0.1,"color",[0 0 1 1]) %type, rad, color
% spawnModel(gazebo,BBall,[-3 3 0.1])