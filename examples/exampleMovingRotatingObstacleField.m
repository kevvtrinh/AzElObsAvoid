function result = exampleMovingRotatingObstacleField(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingRotatingObstacleField()
%   result = exampleMovingRotatingObstacleField(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Plan past three static obstacles while a fourth obstacle translates
%     and rotates across the available routes.
%   - Exercise mixed static and time-varying obstacle histories through the
%     maintained public planner without waypoints or preferred corridors.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3. Rotation angles are radians.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct( ...
    "GoalTimeMode", "earliestArrival", "MaximumSeedCount", 4, ...
    "MaximumTimeLayerCount", 25, "SampleTime_s", 0.05, ...
    "AllowAzimuthWrapping", false, ...
    "Title", "Moving rotating obstacle with three static obstacles"), ...
    [4 4]);

%% Section 2: Create Obstacles

missionEndTime_s = 32;
staticTime_s = [0; missionEndTime_s];
staticCenter_deg = [-5 0; 0 0; 5 0];
staticHalfSize_deg = [0.75 1.20];
staticBoundaryOffset_deg = [ ...
    -1 -1; 1 -1; 1 1; -1 1] .* staticHalfSize_deg;
safetyMargin_deg = 0.12;
obstacleItems = cell(4, 1);
for obstacleIndex = 1:size(staticCenter_deg, 1)
    boundary_deg = staticCenter_deg(obstacleIndex, :) + ...
        staticBoundaryOffset_deg;
    obstacleItems{obstacleIndex} = ...
        obstacleAvoidance.obstacles.createObstacle( ...
        "Static obstacle " + obstacleIndex, staticTime_s, ...
        boundary_deg(:, 1), boundary_deg(:, 2), safetyMargin_deg);
end

movingTime_s = (0:8:missionEndTime_s).';
movingCenter_deg = [2.5 3.8; 3.0 2.2; 3.4 0; 3.0 -2.2; 2.5 -3.8];
movingAngle_rad = deg2rad([-40; -10; 30; 65; 100]);
movingBase_deg = [-1.25 -0.45; 1.25 -0.45; 1.25 0.45; -1.25 0.45];
movingAzimuthByTime_deg = cell(numel(movingTime_s), 1);
movingElevationByTime_deg = cell(numel(movingTime_s), 1);
for sampleIndex = 1:numel(movingTime_s)
    angle_rad = movingAngle_rad(sampleIndex);
    rotation = [cos(angle_rad), -sin(angle_rad); ...
        sin(angle_rad), cos(angle_rad)];
    boundary_deg = movingBase_deg * rotation.' + ...
        movingCenter_deg(sampleIndex, :);
    movingAzimuthByTime_deg{sampleIndex} = boundary_deg(:, 1);
    movingElevationByTime_deg{sampleIndex} = boundary_deg(:, 2);
end
obstacleItems{4} = obstacleAvoidance.obstacles.createObstacle( ...
    "Moving rotating obstacle", movingTime_s, ...
    movingAzimuthByTime_deg, movingElevationByTime_deg, safetyMargin_deg);
obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleItems);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-10 0]);
goalState = struct("time_s", missionEndTime_s, "position_deg", [10 0]);
limits = struct( ...
    "azimuthInterval_deg", [-12 12], ...
    "elevationInterval_deg", [-6 6], ...
    "maxVelocity_deg_s", [3 3], ...
    "maxAcceleration_deg_s2", [1.5 1.5], ...
    "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

exampleValidation = validateExampleResult( ...
    result, "mixed moving and static obstacle field", ...
    struct("RequireDirectBlocked", true));
centerTravel_deg = sum(vecnorm(diff(movingCenter_deg, 1, 1), 2, 2));
rotationTravel_rad = sum(abs(diff(movingAngle_rad)));
motionDefinitionPassed = centerTravel_deg > 0 && rotationTravel_rad > 0;
if ~motionDefinitionPassed
    exampleValidation.Passed = false;
    exampleValidation.Message = ...
        "The fourth obstacle must both translate and rotate.";
end
if ~exampleValidation.Passed
    warning("exampleMovingRotatingObstacleField:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
