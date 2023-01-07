clear
clc
close all

%% Waypoints' list definition

WList = [0, 0, 0;
          6, 0, 3/4*pi;
          3, 6, 1/4*pi;
          9, 9, 7/4*pi;
          12, 0, 1/4*pi;
          15, 3, 0;
          18, 3, 0];

IND = 1:length(WList) - 1;

thV = angle((WList(IND + 1, 1) - WList(IND, 1)) + 1i*(WList(IND + 1, 2) - WList(IND, 2)));
thV(end + 1) = thV(end);
thV(4) = thV(4) + 2*pi;

WList(:, 3) = thV;
save('WList.mat', 'WList')

%% Balls' list definition

BList = zeros(size(WList, 1) - 1, size(WList, 3));

for ii = 1:(size(WList, 1) - 1)
    BList(ii, 1) = (WList(ii, 1) + WList(ii + 1, 1))/2;
    BList(ii, 2) = (WList(ii, 2) + WList(ii + 1, 2))/2;
end

BList(:, 3) = 0.2;

%% Initialize Gazebo

%%%%%%%% rosinit
gazebo = ExampleHelperGazeboCommunicator;

%% Balls' color

RBallCol = [200 48 48];
RBallCol = RBallCol./255;

GBallCol = [48 200 48];
GBallCol = GBallCol./255;

%% Spawn the balls in Gazebo

RBList = BList([1 4 6], :);
GBList = BList([2 3 5], :);

for ii = 1: size(RBList, 1)

    strg = ['RBall' num2str(ii)];
    RBall = ExampleHelperGazeboModel(strg);
    sphereLink = addLink(RBall, "sphere", 0.2, "color", [RBallCol 1]); %type, rad, color
    spawnModel(gazebo, RBall, RBList(ii, :))

end

for ii = 1: size(GBList, 1)

    strg = ['GBall' num2str(ii)];
    GBall = ExampleHelperGazeboModel(strg);
    sphereLink = addLink(GBall, "sphere", 0.2, "color", [GBallCol 1]); %type, rad, color
    spawnModel(gazebo, GBall, GBList(ii, :))

end
