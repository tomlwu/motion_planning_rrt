clc
clear
close all

%% A demonstration of motion planning of a robot manipulator using RRT
% dependencies: Robotics System Toolbox

% load robot
robot = loadrobot("rethinkSawyer","DataFormat","row");
robot.removeBody('head'); % remove robot head screen b/c we don't need

% build an obstacle ball
env = {collisionSphere(0.3)};
env{1}.Pose(1:3, end) = [0.5 0.2 0.8];

% define start and end configs
startConfig = robot.homeConfiguration;
goalConfig =  [1.85 -1.65 0.02 1.04 0.49 0.04 0];

% plot robot start and end config with only collision bodies
% figure
% show(robot,startConfig,"Collisions","on","Visuals","off");
% hold on
% show(env{1})
% show(robot,goalConfig,"Collisions","on","Visuals","off");
% hold off
% title("start and goal configurations")

% build RRT
rrt = manipulatorRRT(robot,env);
% rrt.IgnoreSelfCollision = true;
% rrt.SkippedSelfCollisions = "parent";
rrt.MaxConnectionDistance = 0.02;

% plan motion
path = plan(rrt,startConfig,goalConfig);

% Interpolate and plot the trajectory
interpPath = interpolate(rrt,path);
% figure
% for i = 1:20:size(interpPath,1)
%     show(robot,interpPath(i,:));
%     hold on
% end
% show(env{1})
% hold off

figure;
plot(1:size(interpPath,1),interpPath)