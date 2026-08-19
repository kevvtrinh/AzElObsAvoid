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
%       Final-region planner result plus independent ExampleValidation,
%       RegionSequenceResults, RegionSequenceSummary, ObstacleHistory, and
%       ExampleConfiguration metadata for the complete sequence.
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
[options, jerkConfiguration] = resolveAzElExampleOptions( ...
    options, struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "MaximumSeedCount", 5, ...
    "MaximumPlanningTime_s", 120, ...
    "MaximumDisplayedSlicesPerObstacle", 1, ...
    "ShowSweptSurfaces", false, ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", "Extreme geographic-region visibility sequence"), ...
    [12 12]);

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
        regionNames(regionIndex), [0; missionEndTime_s], 0.15, ...
        struct("Verbose", options.Verbose));
end

%% Section 3: Create Planner Inputs

limits = struct( ...
    "maxVelocity_deg_s", [8 8], ...
    "maxAcceleration_deg_s2", [3 3], ...
    "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);

%% Section 4: Run Planner

regionResults = cell(regionCount, 1);
for regionIndex = 1:regionCount
    scenario = regionScenarios{regionIndex};
    initialState = struct( ...
        "time_s", 0, ...
        "position_deg", scenario.initialPosition_deg);
    goalState = struct( ...
        "time_s", missionEndTime_s, ...
        "position_deg", scenario.goalPosition_deg);
    regionOptions = options;
    regionResults{regionIndex} = planAzElMotion( ...
        obstacles{regionIndex}, initialState, goalState, ...
        limits, regionOptions);
end

%% Section 5: Validate Result

regionPassed = false(regionCount, 1);
regionArrivalTime_s = nan(regionCount, 1);
regionRouteLength_deg = nan(regionCount, 1);
regionNativeVertexCount = zeros(regionCount, 1);
regionTestVertexCount = zeros(regionCount, 1);
for regionIndex = 1:regionCount
    resultForRegion = regionResults{regionIndex};
    exampleValidation = validateAzElExampleResult( ...
        resultForRegion, ...
        "static " + lower(regionNames(regionIndex)) + " outline", ...
        struct("RequireDirectBlocked", true));
    resultForRegion.ExampleValidation = exampleValidation;
    resultForRegion.ObstacleHistory = ...
        obstacleHistories{regionIndex};
    resultForRegion.ExampleConfiguration = jerkConfiguration;
    resultForRegion.ExampleConfiguration.RegionName = ...
        regionNames(regionIndex);
    resultForRegion.ExampleConfiguration.RegionScenario = ...
        regionScenarios{regionIndex};
    regionResults{regionIndex} = resultForRegion;
    regionPassed(regionIndex) = exampleValidation.Passed;
    regionNativeVertexCount(regionIndex) = ...
        obstacleHistories{regionIndex}.nativeSourceVertexCount;
    regionTestVertexCount(regionIndex) = ...
        obstacleHistories{regionIndex}.sourceVertexCount;
    if resultForRegion.Success
        regionArrivalTime_s(regionIndex) = ...
            resultForRegion.time_s(end);
        regionRouteLength_deg(regionIndex) = ...
            sum(vecnorm(diff( ...
            resultForRegion.SelectedSeed_deg, 1, 1), 2, 2));
    end
end

%% Section 6: Plot Diagnostics And Motion

if jerkConfiguration.PlotOutputs
    for regionIndex = 1:regionCount
        plotOptions = jerkConfiguration.PlotOptions;
        plotOptions.Title = "Extreme visibility: " + ...
            regionNames(regionIndex);
        regionResults{regionIndex}.PlotHandles = plotAzElMotion( ...
            regionResults{regionIndex}, plotOptions);
    end
end

%% Section 7: Return Example Metadata

result = regionResults{end};
regionSequencePassed = all(regionPassed);
result.RegionSequenceResults = regionResults;
result.RegionSequenceSummary = table( ...
    regionNames.', regionPassed, regionArrivalTime_s, ...
    regionRouteLength_deg, regionNativeVertexCount, ...
    regionTestVertexCount, ...
    'VariableNames', {'Region', 'Passed', 'ArrivalTime_s', ...
    'RouteLength_deg', 'NativeVertexCount', 'TestVertexCount'});
result.RegionSequencePassed = regionSequencePassed;
result.ExampleValidation.RegionSequencePassed = ...
    regionSequencePassed;
result.ExampleValidation.Passed = ...
    result.ExampleValidation.Passed && regionSequencePassed;
result.ExampleConfiguration.RegionSequence = regionNames;
result.ExampleMetrics = computeAzElExampleMetrics(result);
end
