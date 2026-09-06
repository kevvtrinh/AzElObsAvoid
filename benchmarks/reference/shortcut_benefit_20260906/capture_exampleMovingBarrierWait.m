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
barrierCenterElevation_deg = [0; 0; 8; 8];
sourcePosition_deg         = [-0.2 -3; 0.2 -3; 0.2 3; -0.2 3];
azimuthBySlice_deg         = cell(numel(obstacleTime_s), 1);
elevationBySlice_deg       = cell(numel(obstacleTime_s), 1);

% Move the same barrier through its sampled elevations. Each cell stores the
% complete boundary at one time.
for sampleIndex = 1:numel(obstacleTime_s)
    translatedPosition_deg = sourcePosition_deg + [0 barrierCenterElevation_deg(sampleIndex)];
    azimuthBySlice_deg{sampleIndex} = translatedPosition_deg(:, 1);
    elevationBySlice_deg{sampleIndex} = translatedPosition_deg(:, 2);
end
safetyMargin_deg = 0.1;
obstacles        = obstacleAvoidance.obstacles.createObstacle("translating barrier", obstacleTime_s, azimuthBySlice_deg, elevationBySlice_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

% The direct geometric line becomes safe only after the barrier moves. The time
% window includes enough time to wait and then finish the motion.

initialState = struct();
initialState.time_s       = 0;
initialState.position_deg = [-5 0];
goalState = struct();
goalState.time_s       = 12;
goalState.position_deg = [5 0];
limits = struct("maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3, "azimuthInterval_deg", [-6 6], "elevationInterval_deg", [-3 3]);


request=struct("obstacles",obstacles,"initialState",initialState,"goalState",goalState,"limits",limits,"options",options);
end
