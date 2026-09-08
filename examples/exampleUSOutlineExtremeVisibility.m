function [result, diagnosis, sequence] = exampleUSOutlineExtremeVisibility(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleUSOutlineExtremeVisibility()
%   result = exampleUSOutlineExtremeVisibility(options)
%
% PURPOSE
%   - Plan sequential routes around the dense static outlines of Hawaii,
%     Croatia, and the Philippines using bounded extreme visibility
%     candidates and full protected collision geometry.
%
% INPUTS
%   - options (scalar struct, optional; default struct())
%       Planner/display overrides plus the finite MaxJerk_units_s3 limit.
%
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result for the final region.
%   - sequence (optional scalar struct)
%       Names, unmodified results, and diagnoses for every geographic case.
%
% UNITS
%   - Position is coordinate units, time is seconds, velocity is coordinate units per second,
%     acceleration is coordinate units per second squared, and jerk is coordinate units per
%     second cubed.
%

%% Section 1: Resolve Example Controls

% Resolve one set of limits and display controls for all geographic regions.

if nargin < 1 || isempty(options)
    options = struct();
end
[options, jerkConfiguration] = resolveExampleOptions(options, struct("GoalTimeMode", "earliestArrival", "MaximumDisplayedSlicesPerObstacle", 1, "ShowSweptSurfaces", false, "FigureVisible", "on", "Title", "Extreme geographic-region visibility sequence"), [12 12]);

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
        regionScenarios{regionIndex}] = createGeographicRegionObstacle(regionNames(regionIndex), [0; missionEndTime_s], 0.15, struct("Verbose", jerkConfiguration.Verbose));
end

%% Section 3: Create Planner Inputs

% Use equal physical limits for each region. The helper derives endpoints from
% occupancy tests and does not store a preferred detour.

limits = struct("maxVelocity_units_s", [8 8], "maxAcceleration_units_s2", [3 3], "maxJerk_units_s3", jerkConfiguration.MaxJerk_units_s3);

%% Section 4: Run Planner

regionResults   = cell(regionCount, 1);
regionDiagnoses = cell(regionCount, 1);

% Plan each geographic region independently. Use the same physical limits.
for regionIndex = 1:regionCount
    scenario      = regionScenarios{regionIndex};
    initialState  = struct("time_s", 0, "position_units", scenario.initialPosition_units);
    goalState     = struct("time_s", missionEndTime_s, "position_units", scenario.goalPosition_units);
    regionOptions = options;
    [regionResults{regionIndex}, regionDiagnoses{regionIndex}] = planner(obstacles{regionIndex}, initialState, goalState, limits, regionOptions);
end

%% Section 5: Validate Result

regionPassed = false(regionCount, 1);

% Validate every region. Record geometry size for a fair comparison.
for regionIndex = 1:regionCount
    resultForRegion   = regionResults{regionIndex};
    exampleValidation = validateExampleResult(resultForRegion, "static " + lower(regionNames(regionIndex)) + " outline", struct("RequireDirectBlocked", true), regionDiagnoses{regionIndex});
    regionPassed(regionIndex) = exampleValidation.Passed;
    if ~exampleValidation.Passed
        warning("exampleUSOutlineExtremeVisibility:ValidationFailed", "%s: %s", regionNames(regionIndex), exampleValidation.Message);
    end
end

%% Section 6: Plot Diagnostics And Motion

if jerkConfiguration.PlotOutputs

    % Plot each region in a separate figure. Put the region name in the title.
    for regionIndex = 1:regionCount
        plotOptions = jerkConfiguration.PlotOptions;
        plotOptions.Title = "Extreme visibility: " + regionNames(regionIndex);
        obstacleAvoidance.plotting.plotTrajectory(regionResults{regionIndex}, plotOptions, regionDiagnoses{regionIndex});
    end
end

result    = regionResults{end};
diagnosis = regionDiagnoses{end};
sequence = struct('Names', regionNames, 'Results', {regionResults}, 'Diagnoses', {regionDiagnoses});
if ~all(regionPassed)
    warning("exampleUSOutlineExtremeVisibility:SequenceValidationFailed", "One or more regional planning results failed independent validation.");
end
end
