function result = exampleUSOutlineExtremeVisibility(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleUSOutlineExtremeVisibility()
%   result = exampleUSOutlineExtremeVisibility(options)
%**************************************************************************
% PURPOSE
%   - Plan sequential routes around the dense static outlines of Hawaii,
%     Croatia, and the Philippines using bounded extreme visibility
%     candidates and full protected collision geometry.
%**************************************************************************
% INPUTS
%   - options (scalar struct, optional; default struct())
%       Planner/display overrides plus the finite MaxJerk_deg_s3 limit.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result for the final region.
%**************************************************************************
% UNITS
%   - Position is degrees, time is seconds, velocity is degrees per second,
%     acceleration is degrees per second squared, and jerk is degrees per
%     second cubed.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(options)
    options = struct();
end
[options, jerkConfiguration] = resolveExampleOptions( ...
    options, struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "MaximumSeedCount", 5, ...
    "MaximumDisplayedSlicesPerObstacle", 1, ...
    "ShowSweptSurfaces", false, ...
    "FigureVisible", "on", "Title", "Extreme geographic-region visibility sequence"), [12 12]);

%% Section 2: Create Obstacles

missionEndTime_s = 120;
regionNames = ["Hawaii" "Croatia" "Philippines"];
regionCount = numel(regionNames);
obstacles = cell(regionCount, 1);
obstacleHistories = cell(regionCount, 1);
regionScenarios = cell(regionCount, 1);

% The private geometry helper is retained because source-shapefile
% selection, clipping, and thousands of coastline vertices would obscure
% the visible scenario flow.
for regionIndex = 1:regionCount
    [obstacles{regionIndex}, obstacleHistories{regionIndex}, ...
        regionScenarios{regionIndex}] = createGeographicRegionObstacle( ...
        regionNames(regionIndex), [0; missionEndTime_s], 0.15, struct("Verbose", options.Verbose));
end

%% Section 3: Create Planner Inputs

limits = struct( ...
    "maxVelocity_deg_s", [8 8], "maxAcceleration_deg_s2", [3 3], "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);

%% Section 4: Run Planner

regionResults = cell(regionCount, 1);

% Plan each named geographic region independently under the same physical limits.
for regionIndex = 1:regionCount
    scenario = regionScenarios{regionIndex};
    initialState = struct( "time_s", 0, "position_deg", scenario.initialPosition_deg);
    goalState = struct( "time_s", missionEndTime_s, "position_deg", scenario.goalPosition_deg);
    regionOptions = options;
    regionResults{regionIndex} = obstacleAvoidance.planTrajectory( ...
        obstacles{regionIndex}, initialState, goalState, limits, regionOptions);
end

%% Section 5: Validate Result

regionPassed = false(regionCount, 1);

% Validate every regional result and collect comparable geometry-size evidence.
for regionIndex = 1:regionCount
    resultForRegion = regionResults{regionIndex};
    exampleValidation = validateExampleResult( ...
        resultForRegion, ...
        "static " + lower(regionNames(regionIndex)) + " outline", struct("RequireDirectBlocked", true));
    regionPassed(regionIndex) = exampleValidation.Passed;
    if ~exampleValidation.Passed
        warning("exampleUSOutlineExtremeVisibility:ValidationFailed", ...
            "%s: %s", regionNames(regionIndex), exampleValidation.Message);
    end
end

%% Section 6: Plot Diagnostics And Motion

if jerkConfiguration.PlotOutputs

    % Plot each regional result in its own figure with a region-specific title.
    for regionIndex = 1:regionCount
        plotOptions = jerkConfiguration.PlotOptions;
        plotOptions.Title = "Extreme visibility: " + regionNames(regionIndex);
        obstacleAvoidance.plotting.plotTrajectory( ...
            regionResults{regionIndex}, plotOptions);
    end
end

result = regionResults{end};
if ~all(regionPassed)
    warning("exampleUSOutlineExtremeVisibility:SequenceValidationFailed", ...
        "One or more regional planning results failed independent validation.");
end
end
