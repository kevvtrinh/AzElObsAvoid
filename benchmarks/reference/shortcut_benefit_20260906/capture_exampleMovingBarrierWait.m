function request = capture_exampleMovingBarrierWait()
exampleOverrides = struct();
exampleOverrides.PlotOutputs = false;
exampleOverrides.Verbose     = false;
%% Section 1: Resolve Example Controls

% Keep fixed-arrival timing so waiting can be part of the solution.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "earliestArrival"), [2 2]);

%% Section 2: Create Obstacles

% The barrier crosses the useful route and then moves away. A valid planner must
% represent time, not only position. It can wait in free space and cross later.

obstacleTime_s             = [0; 6; 6.5; 12];
barrierCenterY_units = [0; 0; 8; 8];
sourcePosition_units         = [-0.2 -3; 0.2 -3; 0.2 3; -0.2 3];
xBySlice_units         = cell(numel(obstacleTime_s), 1);
yBySlice_units       = cell(numel(obstacleTime_s), 1);

% Move the same barrier through its sampled ys. Each cell stores the
% complete boundary at one time.
for sampleIndex = 1:numel(obstacleTime_s)
    translatedPosition_units = sourcePosition_units + [0 barrierCenterY_units(sampleIndex)];
    xBySlice_units{sampleIndex} = translatedPosition_units(:, 1);
    yBySlice_units{sampleIndex} = translatedPosition_units(:, 2);
end
safetyMargin_units = 0.1;
obstacles        = obstacleAvoidance.obstacles.createObstacle("translating barrier", obstacleTime_s, xBySlice_units, yBySlice_units, safetyMargin_units);

%% Section 3: Create Planner Inputs

% The direct geometric line becomes safe only after the barrier moves. The time
% window includes enough time to wait and then finish the motion.

initialState = struct();
initialState.time_s       = 0;
initialState.position_units = [-5 0];
goalState = struct();
goalState.time_s       = 12;
goalState.position_units = [5 0];
limits = struct("maxVelocity_units_s", [2 2], ...
    "maxAcceleration_units_s2", [1 1], ...
    "maxJerk_units_s3", displayOptions.MaxJerk_units_s3, "xInterval_units", [-6 6], "yInterval_units", [-3 3]);


request=struct("obstacles",obstacles,"initialState",initialState,"goalState",goalState,"limits",limits,"options",options);
end
