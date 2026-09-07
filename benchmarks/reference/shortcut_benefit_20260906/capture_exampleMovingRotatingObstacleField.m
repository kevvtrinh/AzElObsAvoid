function request = capture_exampleMovingRotatingObstacleField()
exampleOverrides = struct();
exampleOverrides.PlotOutputs = false;
exampleOverrides.Verbose     = false;
%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "earliestArrival", "MaximumTimeLayerCount", 25, "SampleTime_s", 0.05, "WrapX", false, "Title", "Moving rotating obstacle with three static obstacles"), [4 4]);

%% Section 2: Create Obstacles

missionEndTime_s         = 32;
staticTime_s             = [0; missionEndTime_s];
staticCenter_units         = [-5 0; 0 0; 5 0];
staticHalfSize_units       = [0.75 1.20];
staticBoundaryOffset_units = [ ...
    -1 -1; 1 -1; 1 1; -1 1] .* staticHalfSize_units;
safetyMargin_units = 0.12;
obstacleItems    = cell(4, 1);
% Process each obstacle included in this benchmark measurement.
for obstacleIndex = 1:size(staticCenter_units, 1)
    boundary_units = staticCenter_units(obstacleIndex, :) + staticBoundaryOffset_units;
    obstacleItems{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle("Static obstacle " + obstacleIndex, staticTime_s, boundary_units(:, 1), boundary_units(:, 2), safetyMargin_units);
end

movingTime_s              = (0:8:missionEndTime_s).';
movingCenter_units          = [2.5 3.8; 3.0 2.2; 3.4 0; 3.0 -2.2; 2.5 -3.8];
movingAngle_rad           = deg2rad([-40; -10; 30; 65; 100]);
movingBase_units            = [-1.25 -0.45; 1.25 -0.45; 1.25 0.45; -1.25 0.45];
movingXByTime_units   = cell(numel(movingTime_s), 1);
movingYByTime_units = cell(numel(movingTime_s), 1);
% Process each sample included in this benchmark measurement.
for sampleIndex = 1:numel(movingTime_s)
    angle_rad = movingAngle_rad(sampleIndex);
    rotation  = [cos(angle_rad), -sin(angle_rad); ...
        sin(angle_rad), cos(angle_rad)];
    boundary_units = movingBase_units * rotation.' + movingCenter_units(sampleIndex, :);
    movingXByTime_units{sampleIndex} = boundary_units(:, 1);
    movingYByTime_units{sampleIndex} = boundary_units(:, 2);
end
obstacleItems{4} = obstacleAvoidance.obstacles.createObstacle("Moving rotating obstacle", movingTime_s, movingXByTime_units, movingYByTime_units, safetyMargin_units);
obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleItems);

%% Section 3: Create Planner Inputs

initialState = struct();
initialState.time_s       = 0;
initialState.position_units = [-10 0];
goalState = struct("time_s", missionEndTime_s, "position_units", [10 0]);
limits    = struct("xInterval_units", [-12 12], ...
    "yInterval_units", [-6 6], ...
    "maxVelocity_units_s", [3 3], ...
    "maxAcceleration_units_s2", [1.5 1.5], ...
    "maxJerk_units_s3", displayOptions.MaxJerk_units_s3);


request=struct("obstacles",obstacles,"initialState",initialState,"goalState",goalState,"limits",limits,"options",options);
end
