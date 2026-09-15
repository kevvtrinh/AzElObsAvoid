function [result, regionResults] = exampleUSOutlineExtremeVisibility(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleUSOutlineExtremeVisibility()
%   result = exampleUSOutlineExtremeVisibility(options)
%   [result, regionResults] = exampleUSOutlineExtremeVisibility(options)
%**************************************************************************
% PURPOSE
%   - Plan sequential routes around the dense static outlines of Hawaii,
%     Croatia, and the Philippines using full protected collision geometry.
%**************************************************************************
% INPUTS
%   - options (scalar struct, optional; default struct())
%       Planner/display overrides plus the finite MaxJerk_units_s3 limit.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result for the final region.
%   - regionResults (N-by-1 cell array)
%       Every unmodified regional planner result. Ordinary planning failure
%       returns Success = false; invalid input throws.
%**************************************************************************
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

% Resolve one set of limits and display controls for all geographic regions.

if nargin < 1 || isempty(options)
    options = struct();
end
scenarioDefaults = struct( ...
    "GoalTimeMode",  "earliestArrival", ...
    "FigureVisible", "on", ...
    "Title",         "Extreme geographic-region visibility sequence");
[options, displayOptions] = resolveExampleOptions(options, scenarioDefaults, [12 12]);

%% Section 2: Create Obstacles

% Load each region at full available resolution. Each dense outline becomes an
% independent static planning case.

missionEndTime_s  = 120;
regionNames       = ["Hawaii" "Croatia" "Philippines"];
regionCount       = numel(regionNames);
obstacles         = cell(regionCount, 1);
obstacleHistories = cell(regionCount, 1);
regionScenarios   = cell(regionCount, 1);

% The private helper selects and clips source map data. It also processes many
% coastline vertices. Keeping this work separate makes the scenario flow clear.
for regionIndex = 1:regionCount
    [obstacles{regionIndex}, obstacleHistories{regionIndex}, ...
        regionScenarios{regionIndex}] = createGeographicRegionObstacle( ...
        regionNames(regionIndex), [0; missionEndTime_s], 0.15, ...
        struct("Verbose", displayOptions.Verbose));
end

%% Section 3: Create Planner Inputs

% Use equal physical limits for each region. The helper derives endpoints from
% occupancy tests and does not store a preferred detour.

limits = struct( ...
    "maxVelocity_units_s",      [8 8], ...
    "maxAcceleration_units_s2", [3 3], ...
    "maxJerk_units_s3",         displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner

regionResults = cell(regionCount, 1);

% Plan each geographic region independently. Use the same physical limits.
for regionIndex = 1:regionCount
    scenario      = regionScenarios{regionIndex};
    initialState  = struct("time_s", 0, "position_units", scenario.initialPosition_units);
    goalState     = struct("time_s", missionEndTime_s, "position_units", scenario.goalPosition_units);
    regionOptions = options;
    regionResults{regionIndex} = planner(obstacles{regionIndex}, initialState, goalState, limits, regionOptions);
end

%% Section 5: Validate Result

regionPassed = false(regionCount, 1);

% Validate every region. Record geometry size for a fair comparison.
for regionIndex = 1:regionCount
    resultForRegion   = regionResults{regionIndex};
    exampleValidation = validateExampleResult( ...
        resultForRegion, "static " + lower(regionNames(regionIndex)) + ...
        " outline", struct("RequireDirectBlocked", true));
    regionPassed(regionIndex) = exampleValidation.Passed;
    if ~exampleValidation.Passed
        warning("exampleUSOutlineExtremeVisibility:ValidationFailed", ...
            "%s: %s", regionNames(regionIndex), exampleValidation.Message);
    end
end

%% Section 6: Plot Diagnostics And Motion

if displayOptions.PlotOutputs

    % Plot each region in a separate figure. Put the region name in the title.
    for regionIndex = 1:regionCount
        plotOptions       = displayOptions.PlotOptions;
        plotOptions.Title = "Extreme visibility: " + regionNames(regionIndex);
        obstacleAvoidance.plotting.plotTrajectory(regionResults{regionIndex}, plotOptions);
    end
end

% Return the final regional result and report aggregate validation after any
% requested plots have been created.
result = regionResults{end};
if ~all(regionPassed)
    warning("exampleUSOutlineExtremeVisibility:SequenceValidationFailed", ...
        "One or more regional planning results failed independent validation.");
end
end
