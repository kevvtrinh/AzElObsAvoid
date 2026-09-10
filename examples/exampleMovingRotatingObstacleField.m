function [result, diagnosis] = exampleMovingRotatingObstacleField(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingRotatingObstacleField()
%   result = exampleMovingRotatingObstacleField(exampleOverrides)
%
% PURPOSE
%   - Plan past three static obstacles while a fourth obstacle translates
%     and rotates across the available routes.
%   - Exercise mixed static and time-varying obstacle histories through the
%     maintained public planner without waypoints or preferred corridors.
%
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result.
%   - diagnosis (optional second output): search attempts and solver details.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3. Rotation angles are radians.

%% Section 1: Resolve Example Controls
if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct( ...
    "GoalTimeMode", "earliestArrival", "SampleTime_s", 0.05, "WrapX", false, ...
    "Title", "Moving rotating obstacle with three static obstacles"), [4 4]);

%% Section 2: Create Obstacles
missionEndTime_s = 32;
staticTime_s = [0; missionEndTime_s];
staticCenter_units = [-5 0; 0 0; 5 0];
staticHalfSize_units = [0.75 1.20];
staticBoundaryOffset_units = [-1 -1; 1 -1; 1 1; -1 1] .* staticHalfSize_units;
safetyMargin_units = 0.12;
obstacleItems = cell(4, 1);
for obstacleIndex = 1:size(staticCenter_units, 1)
    boundary_units = staticCenter_units(obstacleIndex, :) + staticBoundaryOffset_units;
    obstacleItems{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
        "Static obstacle " + obstacleIndex, staticTime_s, ...
        boundary_units(:, 1), boundary_units(:, 2), safetyMargin_units);
end

movingTime_s = (0:8:missionEndTime_s).';
movingCenter_units = [2.5 3.8; 3.0 2.2; 3.4 0; 3.0 -2.2; 2.5 -3.8];
movingAngle_rad = deg2rad([-40; -10; 30; 65; 100]);
movingBase_units = [-1.25 -0.45; 1.25 -0.45; 1.25 0.45; -1.25 0.45];
movingXByTime_units = cell(numel(movingTime_s), 1);
movingYByTime_units = cell(numel(movingTime_s), 1);
for sampleIndex = 1:numel(movingTime_s)
    angle_rad = movingAngle_rad(sampleIndex);
    rotation = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)];
    boundary_units = movingBase_units * rotation.' + movingCenter_units(sampleIndex, :);
    movingXByTime_units{sampleIndex} = boundary_units(:, 1);
    movingYByTime_units{sampleIndex} = boundary_units(:, 2);
end
obstacleItems{4} = obstacleAvoidance.obstacles.createObstacle( ...
    "Moving rotating obstacle", movingTime_s, movingXByTime_units, ...
    movingYByTime_units, safetyMargin_units);
obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleItems);

%% Section 3: Create Planner Inputs
initialState = struct('time_s', 0, 'position_units', [-10 0]);
goalState = struct('time_s', missionEndTime_s, 'position_units', [10 0]);
limits = struct('xInterval_units', [-12 12], 'yInterval_units', [-6 6], ...
    'maxVelocity_units_s', [3 3], 'maxAcceleration_units_s2', [1.5 1.5], ...
    'maxJerk_units_s3', displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner
[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result
exampleValidation = validateExampleResult(result, ...
    "mixed moving and static obstacle field", struct('RequireDirectBlocked', true), diagnosis);
centerTravel_units = sum(vecnorm(diff(movingCenter_units, 1, 1), 2, 2));
rotationTravel_rad = sum(abs(diff(movingAngle_rad)));
if centerTravel_units <= 0 || rotationTravel_rad <= 0
    exampleValidation.Passed = false;
    exampleValidation.Message = "The fourth obstacle must both translate and rotate.";
end
if ~exampleValidation.Passed
    warning('exampleMovingRotatingObstacleField:ValidationFailed', '%s', exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion
if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end
end
