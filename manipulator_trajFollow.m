clc
close all
clear

%% Control a robot manipulator to follow a trajectory
% dependencies: Robotics System Toolbox
% load robot
robot = importrobot('sawyer.urdf');
robot.removeBody('head');
robot.DataFormat = 'column';
robot.Gravity = [0, 0, -9.80665];

% extract robot joint limits - needed for the control
revoluteJointsIndex = [5:9,11,13];
jointLimits = zeros(7,2);
r = 1;
for i = revoluteJointsIndex(1:end)
    jointLimits(r,:) = robot.Bodies{i}.Joint.PositionLimits;
    r=r+1;
end

% load an example trajectory
traj = load("example_trajectories\traj_sawyer_short_obs_1.mat","path").path';
env = load("example_trajectories\traj_sawyer_short_obs_1.mat","env").env;

% initialize the configuration
q0 = traj(:,1); % Position
dq0 = zeros(size(q0)); % Velocity
ddq0 = zeros(size(q0)); % Acceleration

% run Simulink model
simResult = sim('manipulatorTrajectoryFollow.slx','StopTime','25');
path_time_series = simResult.yout{1}.Values;

%% visualize the motion

figure
for i = 1:size(path_time_series.Data,1)
    axis([-1 2 -1.3 1.7 -0.7 2.3])
    show(env{1})
    hold on
    show(robot,path_time_series.Data(i,:)',"Frames","off");
    hold off
    axis([-1 2 -1.3 1.7 -0.7 2.3])
    view([0 1 0.5])
    title("t="+num2str(path_time_series.Time(i))+"s")
    pause(0.01);
end

figure
subplot(2,1,1)
plot(path_time_series)
subplot(2,1,2)
plot(1:size(traj,2),traj)
title('Planned Trajectory')
xlabel('step')
ylabel('q')