clc
close all
clear

%% Generate data for the task of Controling a robot manipulator to follow trajectories
% the trajectories are pre-generated
% dependencies: Robotics System Toolbox

% specify waypoint data to be loaded
% fname = "example_trajectories\traj_sawyer_2.mat";
fname = "example_trajectories\traj_sawyer_100pts.mat";

% some settings
posTgtThreshold = 0.015; % a distance threshold to specify stopping criteria

% load robot
robot = importrobot('sawyer.urdf');
robot.removeBody('head');
robot.DataFormat = 'column';
robot.Gravity = [0, 0, -9.80665];

% extract robot joint limits - needed for the control
revoluteJointsIndex = [5:9,11,13];
jointLimits = zeros(7,2);
jointLimitsTol = 1e-5;
r = 1;
for i = revoluteJointsIndex(1:end)
    jointLimits(r,:) = robot.Bodies{i}.Joint.PositionLimits;
    r=r+1;
end
jointLimits = jointLimits+jointLimitsTol.*repmat([-1,1],[size(jointLimits,1),1]);


% load trajectory
load(fname);
path_time_series_set = cell(n_traj,1);

for i_traj = 1:n_traj
    traj = path_set{i_traj,:}';
    % initialize the configuration
    q0 = traj(:,1); % Position
    dq0 = zeros(size(q0)); % Velocity
    ddq0 = zeros(size(q0)); % Acceleration

    % run Simulink model
    simResult = sim('manipulatorTrajectoryFollowDataGen.slx','StopTime','100');
    path_time_series_set{i_traj,:} = [simResult.yout{1}.Values.Data,simResult.yout{2}.Values.Data];
end

fpath = '..\trainingData';
fname = strcat('timeseries_sawyer_',string(datetime('now','Format',"yyyy-MM-dd-HH-mm-ss")),'.mat');
save(string(fpath)+'\'+fname,"path_time_series_set","start_set","start_config_set","path_set","n_traj")

%% inspect a generated path
i_look = 591; % index of path to look at

figure
for i = 1:size(path_time_series_set{i_look,:},1)
    axis([-1 2 -1.3 1.7 -0.7 2.3])
    show(robot,path_time_series_set{i_look,:}(i,1:7)',"Frames","off");
    view([0 1 0.5])
    title("t="+num2str(0.1*(i-1))+"s")
    movieFrames(i)=getframe(gcf);
    pause(0.01);
end
% create the video writer with 1 fps
writerObj = VideoWriter('myVideo.avi');
writerObj.FrameRate = 10;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for iFrame=1:length(movieFrames)
    % convert the image to a frame
    frame = movieFrames(iFrame) ;
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);

figure
subplot(2,1,1)
plot(path_time_series_set{i_look,:}(:,1:14))
title('Planned Time Series')
xlabel('seconds')
ylabel('states')
subplot(2,1,2)
plot(1:size(path_set{i_look,:}',2),path_set{i_look,:}')
title('Planned Waypoints')
xlabel('step')
ylabel('q')