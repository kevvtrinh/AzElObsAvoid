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

% Keep fixed-arrival timing so waiting can be part of the solution.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct("GoalTimeMode", "earliestArrival"), [2 2]);

%% Section 2: Create Obstacles

% The barrier crosses the useful route and then moves away. A valid planner must
% represent time, not only position. It can wait in free space and cross later.

obstacleTime_s = [0; 6; 6.5; 12];
barrierCenterElevation_deg = [0; 0; 8; 8];
sourcePosition_deg = [-0.2 -3; 0.2 -3; 0.2 3; -0.2 3];
azimuthBySlice_deg = cell(numel(obstacleTime_s), 1);
elevationBySlice_deg = cell(numel(obstacleTime_s), 1);

% Move the same barrier through its sampled elevations. Each cell stores the
% complete boundary at one time.
for sampleIndex = 1:numel(obstacleTime_s)
    translatedPosition_deg = sourcePosition_deg + [0 barrierCenterElevation_deg(sampleIndex)];
    azimuthBySlice_deg{sampleIndex} = translatedPosition_deg(:, 1);
    elevationBySlice_deg{sampleIndex} = translatedPosition_deg(:, 2);
end
safetyMargin_deg = 0.1;
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "translating barrier", obstacleTime_s, azimuthBySlice_deg, elevationBySlice_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

% The direct geometric line becomes safe only after the barrier moves. The time
% window includes enough time to wait and then finish the motion.

initialState = struct("time_s", 0, "position_deg", [-5 0]);
goalState = struct("time_s", 12, "position_deg", [5 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3, "azimuthInterval_deg", [-6 6], "elevationInterval_deg", [-3 3]);

%% Section 4: Run Planner

% The wait seed can produce repeated solver conditioning warnings. Hide only
% these expected warnings. Independent validation still rejects invalid motion.
warningState = warning;
warning("off", "MATLAB:nearlySingularMatrix");
warning("off", "MATLAB:singularMatrix");
warningCleanup = onCleanup(@() warning(warningState));
result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);
clear warningCleanup;

%% Section 5: Validate Result

% Check collision freedom over time. This check detects a route that crosses the
% barrier too early even if its geometric path looks correct.

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

% Animation is important for this case because a static plot cannot show why
% the waiting segment is necessary.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
