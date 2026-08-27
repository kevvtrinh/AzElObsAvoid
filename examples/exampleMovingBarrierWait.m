function result = exampleMovingBarrierWait(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingBarrierWait()
%   result = exampleMovingBarrierWait(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate useful continuous waiting for a translating barrier.
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
%     and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct( "GoalTimeMode", "earliestArrival", "MaximumSeedCount", 5), [2 2]);

%% Section 2: Create Obstacles

obstacleTime_s = [0; 6; 6.5; 12];
barrierCenterElevation_deg = [0; 0; 8; 8];
sourcePosition_deg = [-0.2 -3; 0.2 -3; 0.2 3; -0.2 3];
azimuthBySlice_deg = cell(numel(obstacleTime_s), 1);
elevationBySlice_deg = cell(numel(obstacleTime_s), 1);

% Translate the rigid barrier through its sampled elevations to define the timed geometry.
for sampleIndex = 1:numel(obstacleTime_s)
    translatedPosition_deg = sourcePosition_deg + [0 barrierCenterElevation_deg(sampleIndex)];
    azimuthBySlice_deg{sampleIndex} = translatedPosition_deg(:, 1);
    elevationBySlice_deg{sampleIndex} = translatedPosition_deg(:, 2);
end
safetyMargin_deg = 0.1;
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "translating barrier", obstacleTime_s, azimuthBySlice_deg, elevationBySlice_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-5 0]);
goalState = struct("time_s", 12, "position_deg", [5 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3, "azimuthInterval_deg", [-6 6], "elevationInterval_deg", [-3 3]);

%% Section 4: Run Planner

% These expected interior-point conditioning warnings are extremely repetitive
% for the intentional wait seed. Validation below still rejects bad motion.
warningState = warning;
warning("off", "MATLAB:nearlySingularMatrix");
warning("off", "MATLAB:singularMatrix");
warningCleanup = onCleanup(@() warning(warningState));
result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);
clear warningCleanup;

%% Section 5: Validate Result

exampleValidation = obstacleAvoidance.validateTrajectory(result);
waitSeedSelected = result.Success && result.SelectedSeedIndex > 0 && ...
    result.Seeds(result.SelectedSeedIndex).Source == "directWait";
exampleValidation.WaitSeedSelected = waitSeedSelected;
exampleValidation.Passed = exampleValidation.Passed && waitSeedSelected;
if ~waitSeedSelected
    exampleValidation.Message = exampleValidation.Message + ...
        " The planner did not select the direct waiting seed.";
end
if ~exampleValidation.Passed
    warning("exampleMovingBarrierWait:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
