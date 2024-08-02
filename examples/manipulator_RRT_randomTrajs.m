clc
clear
close all

%% Perform motion planning of a robot manipulator using RRT for multiple random goals
% dependencies: Robotics System Toolbox

% settings

flag_obs = false; % include obstacles
n_traj = 100; % number of trajectories to be generated
startSpace = [-1 1; -1 1; -0.3 1.2];

%% initialize
% load robot
robot = loadrobot("rethinkSawyer","DataFormat","row");
robot.removeBody('head'); % remove robot head screen b/c we don't need

if flag_obs
    % build an obstacle ball
    env = {collisionSphere(0.3)};
    env{1}.Pose(1:3, end) = [0.5 0.2 0.8];
else
    env = {};
end

% define start and end configs
goalConfig = robot.homeConfiguration;
% find end config by solving a inverse kinematics problem
gik = generalizedInverseKinematics;
gik.RigidBodyTree = robot;
gik.ConstraintInputs = {'position'};
start_set = zeros(3,n_traj);
start_config_set = [];
n_config = 0;
while n_config<n_traj
    % generate random target positions
    T_start = [(startSpace(1,2)-startSpace(1,1))*rand+startSpace(1,1);...
        (startSpace(2,2)-startSpace(2,1))*rand+startSpace(2,1);...
        (startSpace(3,2)-startSpace(3,1))*rand+startSpace(3,1);];
    posTarget = constraintPositionTarget('right_wrist');
    posTarget.TargetPosition = T_start';
    [startConfig,solInfo] = gik(goalConfig,posTarget); % use home config as an init guess
    if solInfo.ExitFlag == 1 &&...discard the config if the IK solver fails
            ~robot.checkCollision(startConfig) % discard also if self collision
        start_set(:,n_config+1) = T_start;
        start_config_set = [start_config_set;startConfig];
        n_config = n_config+1;
    end
end


% build RRT
rrt = manipulatorRRT(robot,env);
% rrt.IgnoreSelfCollision = true;
% rrt.SkippedSelfCollisions = "parent";

% plan motion
path_set = cell(n_traj,1);
for i_path = 1:n_traj
    path_i = plan(rrt,start_config_set(i_path,:),goalConfig);
    if isempty(path_i)
        while isempty(path_i) % regenerate if motion planning fails
            flag_iksuccess = false;
            while ~flag_iksuccess
                T_start = [(startSpace(1,2)-startSpace(1,1))*rand+startSpace(1,1);...
                    (startSpace(2,2)-startSpace(2,1))*rand+startSpace(2,1);...
                    (startSpace(3,2)-startSpace(3,1))*rand+startSpace(3,1);];
                posTarget = constraintPositionTarget('right_wrist');
                posTarget.TargetPosition = T_start';
                [startConfig,solInfo] = gik(goalConfig,posTarget);
                flag_iksuccess = solInfo.ExitFlag == 1 && ~robot.checkCollision(startConfig);
            end
            path_i = plan(rrt,startConfig,goalConfig);
        end
        start_set(:,i_path) = T_start;
        start_config_set(i_path,:) = startConfig;
    end
    path_set{i_path,:} = path_i;
end

fpath = 'example_trajectories';
fname = strcat('traj_sawyer_',string(datetime('now','Format',"yyyy-MM-dd-HH-mm-ss")),'.mat');
save(string(fpath)+'\'+fname,"start_set","start_config_set","path_set","n_traj")