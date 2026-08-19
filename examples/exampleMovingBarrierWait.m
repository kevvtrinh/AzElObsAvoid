function result = exampleMovingBarrierWait(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingBarrierWait()
%   result = exampleMovingBarrierWait(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate that HS3 can select useful waiting for a translating barrier.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Planner result, independent validation, plots, and example metrics.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveAzElExampleOptions( ...
    exampleOverrides, struct( ...
    "GoalTimeMode", "fixedArrival", "MaximumSeedCount", 5, ...
    "CollocationSegmentCount", 7, "MaximumPlanningTime_s", 40, ...
    "AzimuthInterval_deg", [-6 6], ...
    "ElevationInterval_deg", [-3 3]));

%% Section 2: Create Obstacles

obstacleTime_s = [0; 6; 6.5; 12];
barrierCenterElevation_deg = [0; 0; 8; 8];
sourcePosition_deg = [-0.2 -3; 0.2 -3; 0.2 3; -0.2 3];
azimuthBySlice_deg = cell(numel(obstacleTime_s), 1);
elevationBySlice_deg = cell(numel(obstacleTime_s), 1);
for sampleIndex = 1:numel(obstacleTime_s)
    translatedPosition_deg = sourcePosition_deg + ...
        [0 barrierCenterElevation_deg(sampleIndex)];
    azimuthBySlice_deg{sampleIndex} = translatedPosition_deg(:, 1);
    elevationBySlice_deg{sampleIndex} = translatedPosition_deg(:, 2);
end
safetyMargin_deg = 0.1;
obstacles = makeAzElObstacleData( ...
    "translating barrier", obstacleTime_s, ...
    azimuthBySlice_deg, elevationBySlice_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-5 0]);
goalState = struct("time_s", 12, "position_deg", [5 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2]);

%% Section 4: Run Planner

result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

result.ExampleValidation = validateAzElTrajectory(result);
waitSeedSelected = result.Success && result.SelectedSeedIndex > 0 && ...
    result.Seeds(result.SelectedSeedIndex).Source == "directWait";
result.ExampleValidation.WaitSeedSelected = waitSeedSelected;
result.ExampleValidation.Passed = ...
    result.ExampleValidation.Passed && waitSeedSelected;
if ~waitSeedSelected
    result.ExampleValidation.Message = ...
        result.ExampleValidation.Message + ...
        " The planner did not select the direct waiting seed.";
end

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if displayOptions.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, displayOptions.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleName = "exampleMovingBarrierWait";
result.ExampleMetrics = computeAzElExampleMetrics(result);
result.ExampleControls = displayOptions;
result.ExampleGeometry = struct( ...
    "obstacleTime_s", obstacleTime_s, ...
    "barrierCenterElevation_deg", barrierCenterElevation_deg, ...
    "sourcePosition_deg", sourcePosition_deg, ...
    "safetyMargin_deg", safetyMargin_deg);
end
