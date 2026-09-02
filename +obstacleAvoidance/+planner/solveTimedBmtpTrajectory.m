function [candidate, diagnostics] = solveTimedBmtpTrajectory( ...
        seed, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.solveTimedBmtpTrajectory( ...
%       seed, obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Adapt one timed multi-waypoint seed to the smooth BMTP engine.
%   - Conservatively bind each moving-obstacle time cell to the polynomial
%     spans that overlap it, without constraining interior waypoints to rest.
%**************************************************************************
% INPUTS
%   - seed (scalar struct)
%       position_deg is N-by-2, tau increases zero to one, and
%       EstimatedDuration_s is the search-derived arrival estimate.
%   - obstacles (canonical or prepared obstacle struct array)
%       Static and time-varying protected geometry over the request horizon.
%   - initialState, goalState, limits, options (resolved scalar structs)
%       Normalized planner request and fully resolved planner options.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Smooth motion or stable expected-failure record for public validation.
%   - diagnostics (scalar struct)
%       Engine evidence plus every bounded fixed-arrival time-cell trial.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3. Histories and polygon vertices are N-by-2.
%**************************************************************************

%% Section 1: Resolve Bounded Fixed-Arrival Trials

if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);
startTime_s = initialState.time_s;
horizonTime_s = goalState.time_s;
estimatedTime_s = startTime_s + double(seed.EstimatedDuration_s);
if options.GoalTimeMode == "fixedArrival"
    trialTime_s = horizonTime_s;
else
    estimatedTime_s = min(horizonTime_s, estimatedTime_s);
    trialTime_s = estimatedTime_s;
    if ~isfinite(estimatedTime_s) || estimatedTime_s <= startTime_s
        trialTime_s = horizonTime_s;
    elseif estimatedTime_s < horizonTime_s - options.ArrivalTimeTolerance_s
        trialTime_s = [estimatedTime_s; horizonTime_s];
    end
end
trialTime_s = unique(double(trialTime_s(:)), "stable");
maximumTimedSegmentCount = seed.MaximumTimedSegmentCount;
if maximumTimedSegmentCount < 1
    error("solveTimedBmtpTrajectory:MissingTimedSegmentBound", ...
        "A timed visibility seed must retain its positive search-clock bound.");
end
timedSegmentCount = createTimedSegmentCount(seed, maximumTimedSegmentCount);
trialTemplate = struct( ...
    "FinalTime_s", NaN, "Coverage", struct(), ...
    "Success", false, "TerminationReason", "notRun", ...
    "ElapsedTime_s", 0);
trials = repmat(trialTemplate, numel(trialTime_s), 1);
candidate = bmtpEngine.createMotionRecord( ...
    struct(), initialState, [], [], options.SampleTime_s, seed.Source);
diagnostics = struct( ...
    "Identifier", "bmtpTimedCell", "Attempted", true, ...
    "Accepted", false, "TrialCount", numel(trialTime_s), ...
    "Trials", trials, "ElapsedTime_s", 0, ...
    "TerminationReason", "noTimedCellTrajectory");
totalTimer = tic;

%% Section 2: Solve Each Conservative Time-Cell Representation

fixedOptions = options;
fixedOptions.GoalTimeMode = "fixedArrival";
for trialIndex = 1:numel(trialTime_s)
    fixedGoalState = createFixedGoalState(goalState, trialTime_s(trialIndex));
    [regions_deg, coverage] = createTimeCellRegions( ...
        obstacles, startTime_s, trialTime_s(trialIndex), timedSegmentCount);
    trialTimer = tic;
    [trialCandidate, trialDiagnostics] = bmtpEngine.solve( ...
        seed, regions_deg, coverage, initialState, fixedGoalState, limits, ...
        fixedOptions);
    trials(trialIndex).FinalTime_s = trialTime_s(trialIndex);
    trials(trialIndex).Coverage = coverage;
    trials(trialIndex).Success = trialCandidate.Success;
    trials(trialIndex).TerminationReason = trialCandidate.TerminationReason;
    trials(trialIndex).ElapsedTime_s = toc(trialTimer);
    candidate = trialCandidate;
    diagnostics = trialDiagnostics;
    diagnostics.Identifier = "bmtpTimedCell";
    diagnostics.TimeCellTrials = trials;
    diagnostics.TimedCellTrialCount = trialIndex;
    diagnostics.DynamicObstacleRepresentation = ...
        "perIntervalProtectedGeometryConvexHull";
    if trialCandidate.Success
        diagnostics.Accepted = true;
        diagnostics.TerminationReason = "goalReached";
        break;
    end
end
diagnostics.TimeCellTrials = trials;
diagnostics.TimedCellTrialCount = find([trials.ElapsedTime_s] > 0, 1, "last");
if isempty(diagnostics.TimedCellTrialCount)
    diagnostics.TimedCellTrialCount = 0;
end
diagnostics.ElapsedTime_s = toc(totalTimer);
candidate.SolverDiagnostics = diagnostics;
end

%% Section 3: Local Functions

function fixedGoalState = createFixedGoalState(goalState, finalTime_s)
% Freeze the requested endpoint at one physical trial time.
fixedGoalState = goalState;
fixedGoalState.time_s = finalTime_s;
fixedGoalState.position_deg = ...
    obstacleAvoidance.input.goalPositionAtTime(goalState, finalTime_s);
metadataFields = intersect(fieldnames(fixedGoalState), ...
    {'targetTime_s', 'targetPosition_deg', 'InterpolationMethod'});
if ~isempty(metadataFields)
    fixedGoalState = rmfield(fixedGoalState, metadataFields);
end
end

function [regions_deg, coverage] = createTimeCellRegions( ...
        obstacles, startTime_s, finishTime_s, timedSegmentCount)
% Cover static shapes exactly and movers by interval-wide convex supersets.
regions_deg = cell(0, 1);
activeTauInterval = zeros(0, 2);
sourceObstacleIndex = zeros(0, 1);
sourceCellIndex = zeros(0, 1);
timeCellCount = timedSegmentCount;
baseEdges_s = linspace(startTime_s, finishTime_s, timeCellCount + 1).';
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    [isStatic, staticShape] = ...
        obstacleAvoidance.obstacles.queryStaticHorizon( ...
        obstacle, startTime_s, finishTime_s);
    if isStatic
        exactRegions = ...
            obstacleAvoidance.geometry.convexPolygonRegions(staticShape);
        for regionIndex = 1:numel(exactRegions)
            vertices_deg = finiteVertices(exactRegions(regionIndex).Vertices);
            if size(vertices_deg, 1) >= 3
                regions_deg{end + 1, 1} = vertices_deg; %#ok<AGROW>
                activeTauInterval(end + 1, :) = [0 1]; %#ok<AGROW>
                sourceObstacleIndex(end + 1, 1) = obstacleIndex; %#ok<AGROW>
                sourceCellIndex(end + 1, 1) = 0; %#ok<AGROW>
            end
        end
        continue;
    end
    obstacleTimes_s = double(obstacle.time_s(:));
    internalEdges_s = obstacleTimes_s(obstacleTimes_s > startTime_s & ...
        obstacleTimes_s < finishTime_s);
    cellEdges_s = snapCellEdgesToObstacleTimes( ...
        [baseEdges_s; internalEdges_s], obstacleTimes_s);
    for cellIndex = 1:numel(cellEdges_s) - 1
        cellStart_s = cellEdges_s(cellIndex);
        cellFinish_s = cellEdges_s(cellIndex + 1);
        queryTime_s = [cellStart_s; ...
            0.5 * (cellStart_s + cellFinish_s); cellFinish_s];
        vertices_deg = zeros(0, 2);
        for queryIndex = 1:numel(queryTime_s)
            shape = obstacleAvoidance.obstacles.shapeAtTime( ...
                obstacle, queryTime_s(queryIndex));
            vertices_deg = [vertices_deg; ...
                finiteVertices(shape.Vertices)]; %#ok<AGROW>
        end
        vertices_deg = unique(vertices_deg, "rows", "stable");
        if size(vertices_deg, 1) < 3
            continue;
        end
        hullIndex = convhull(vertices_deg(:, 1), vertices_deg(:, 2));
        regions_deg{end + 1, 1} = ...
            vertices_deg(hullIndex(1:end - 1), :); %#ok<AGROW>
        activeTauInterval(end + 1, :) = ...
            ([cellStart_s cellFinish_s] - startTime_s) / ...
            (finishTime_s - startTime_s); %#ok<AGROW>
        sourceObstacleIndex(end + 1, 1) = obstacleIndex; %#ok<AGROW>
        sourceCellIndex(end + 1, 1) = cellIndex; %#ok<AGROW>
    end
end
coverage = struct( ...
    "Passed", true, "ObstacleCount", numel(obstacles), ...
    "RegionCount", numel(regions_deg), ...
    "ExactRegionCount", numel(regions_deg), ...
    "SolverRegionCount", numel(regions_deg), ...
    "RegionActiveTauInterval", activeTauInterval, ...
    "RegionSourceObstacleIndex", sourceObstacleIndex, ...
    "RegionSourceCellIndex", sourceCellIndex, ...
    "BaseTimeCellCount", timeCellCount, ...
    "TimedSegmentCount", timedSegmentCount, ...
    "TimeCellContainmentBasis", ...
    "convexHullOfProtectedIntervalEndpointAndMidpointGeometry", ...
    "AuthoritativeCoverageCheck", "publicDynamicValidation");
end

function cellEdges_s = snapCellEdgesToObstacleTimes( ...
        candidateEdges_s, obstacleTimes_s)
% Coalesce roundoff-equivalent solver-grid and obstacle-event times.
timeScale_s = max([1; abs(candidateEdges_s); abs(obstacleTimes_s)]);
timeTolerance_s = 4096 * eps(timeScale_s);
for eventIndex = 1:numel(obstacleTimes_s)
    nearEvent = abs(candidateEdges_s - obstacleTimes_s(eventIndex)) <= ...
        timeTolerance_s;
    candidateEdges_s(nearEvent) = obstacleTimes_s(eventIndex);
end
cellEdges_s = unique(candidateEdges_s, "sorted");
end

function segmentCount = createTimedSegmentCount(seed, maximumSegmentCount)
% Recover the search clock without exceeding the layer budget that authored it.
tau = double(seed.tau(:));
minimumInterval = min(diff(tau));
segmentCount = max(1, round(1 / minimumInterval));
clockResidual = max(abs(tau * segmentCount - round(tau * segmentCount)));
if clockResidual > 1e-8 || segmentCount > maximumSegmentCount
    segmentCount = maximumSegmentCount;
end
end

function vertices_deg = finiteVertices(vertices_deg)
% Remove polyshape ring separators before exact decomposition or hulling.
vertices_deg = double(vertices_deg);
vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
end
