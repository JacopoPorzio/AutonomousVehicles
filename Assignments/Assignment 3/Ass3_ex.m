%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %
%   ADD objects in GAZEBO                                           %
%                                            Arrigoni 11.2022       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% ADD Objects

% connect to GAZEBO

%%
% rosinit % IPadress if VMachine
gazebo = ExampleHelperGazeboCommunicator; % create communication object

%% add objects in GAZEBO

cil = ExampleHelperGazeboModel("Cil")
cilLink = addLink(cil,"cylinder",[1 0.2],"color",[1 0 0 1]) %type, rad, color
spawnModel(gazebo,cil,[6 2 1])


ball1 = ExampleHelperGazeboModel("Ball")
sphereLink = addLink(ball1,"sphere",0.1,"color",[0 0 1 1]) %type, rad, color
spawnModel(gazebo,ball1,[5 0 0.1])

ball2 = ExampleHelperGazeboModel("Ball2")
sphereLink = addLink(ball2,"sphere",0.1,"color",[0 1 1 1]) %type, rad, color
spawnModel(gazebo,ball2,[6 -1 0.1])

ball3 = ExampleHelperGazeboModel("Ball3")
sphereLink = addLink(ball3,"sphere",0.1,"color",[0 0.4 0.4 1]) %type, rad, color
spawnModel(gazebo,ball3,[6 -2 0.1])

ball4 = ExampleHelperGazeboModel("Ball4")
sphereLink = addLink(ball4,"sphere",0.1,"color",[0 0 0.2 1]) %type, rad, color
spawnModel(gazebo,ball4,[6 -3 0.1])

ball5 = ExampleHelperGazeboModel("Ball5")
sphereLink = addLink(ball5,"sphere",0.1,"color",[0.2 0 0.3 1]) %type, rad, color
spawnModel(gazebo,ball5,[6 1 0.1])

ball6 = ExampleHelperGazeboModel("Ball6")
sphereLink = addLink(ball6,"sphere",0.1,"color",[0 1 0 1]) %type, rad, color
spawnModel(gazebo,ball6,[6 3 0.1])

%% Detect IMAGE

%%% simulink (from TurtleBotObjectTrackingExample.mlx)

blueMax = 120;  % Maximum permissible deviation from pure blue
darkMin = 30;   % Minimum acceptable darkness value

