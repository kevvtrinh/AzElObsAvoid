function [result, diagnosis] = planTrajectory( ...
        obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.planTrajectory()
%   result = obstacleAvoidance.planTrajectory( ...
%       obstacles, initialState, goalState, limits)
%   result = obstacleAvoidance.planTrajectory( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%   [result, diagnosis] = obstacleAvoidance.planTrajectory( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%
% PURPOSE
%   - Plan collision-free Az/El motion through one public entry point.
%   - Minimize arrival time, breaking ties by path length, or minimize travel
%     at a specified arrival time.
%
% INPUTS
%   - obstacles (canonical protected obstacle array, nested cells, or [])
%       Use obstacleAvoidance.obstacles.createObstacle to add each safety
%       margin one time.
%   - initialState (scalar struct)
%       Initial time, position, and supported derivatives.
%   - goalState (scalar struct)
%       Fixed or moving-goal state accepted by the obstacle planner.
%   - limits (scalar struct)
%       Physical and workspace limits with units in field names.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial planner options. Empty fields use their documented defaults.
%
% OUTPUTS
%   - result (scalar struct)
%       Status, selected route, motion, plotting inputs, and validation data.
%   - diagnosis (optional scalar struct)
%       Timing, candidate attempts, search evidence, and flat solver details.
%   - options (scalar struct, zero-input call)
%       Fully resolved planner defaults.
%
% UNITS
%   - Position is in degrees. Time is in seconds.
%   - Derivatives use deg/s, deg/s^2, and deg/s^3.
%   - Histories are N-by-2 [azimuth elevation] arrays.
%

%% Section 1: Resolve Defaults Requests

% Return planner defaults when called without inputs.
if nargin == 0
    result = obstacleAvoidance.input.resolvePlannerOptions();
    diagnosis = struct();
    return;
end

%% Section 2: Resolve The Planner Request

% Require obstacles, initial state, goal state, and limits.
if nargin < 4
    error("planTrajectory:MissingInputs", ...
        "obstacles, initialState, goalState, and limits are required.");
end
% Use defaults when options are omitted or empty.
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end

%% Section 3: Normalize The Request And Prepare The Scene

planningTimer = tic;

% Normalize the planning inputs.
options = obstacleAvoidance.input.resolvePlannerOptions(optionOverrides);

[obstacles, initialState, goalState, limits] = obstacleAvoidance.input.normalizePlannerRequest( ...
    obstacles, initialState, goalState, limits, options);

[result, summaryTemplate] = obstacleAvoidance.planner.createEmptyResult(obstacles, initialState, goalState, limits, options, ...
    obstacleAvoidance.validateTrajectory());

% Prepare shared obstacle geometry once for search and validation.
scene = obstacleAvoidance.obstacles.preparePlanningScene( ...
    obstacles, initialState, goalState);

preparedObstacles = scene.preparedObstacles;
useStaticSolver = scene.obstaclesRemainStatic;
stageTiming = result.SearchDiagnostics.StageTiming;
exactMotionSet = obstacleAvoidance.planner.solveExactCandidates();
result.SearchDiagnostics.DirectAttempt = exactMotionSet.DirectAttempt;
result.SearchDiagnostics.FixedClockExcursion = ...
    exactMotionSet.ExcursionDiagnostics;
result.SearchDiagnostics.SelectionPolicy = struct( ...
    "GoalTimeMode", options.GoalTimeMode, ...
    "JerkRole", "hardConstraintOnly", ...
    "UtilizationTieBreak", ...
    "mean normalized peak velocity, acceleration, and jerk");
%% Section 4: Check Physical Endpoints

[endpointFeasible, result.Message, result.TerminationReason] = ...
    obstacleAvoidance.input.validatePlannerEndpoints( ...
    preparedObstacles, initialState, goalState, limits, options);
if ~endpointFeasible

    result = obstacleAvoidance.planner.stageTiming(result, planningTimer, stageTiming);
    [result, diagnosis] = obstacleAvoidance.planner.createPublicOutputs(result, nargout > 1);
    return;
end

%% Section 5: Try Exact Physical-Time Motions

% Try validated direct and fixed-clock motions before building the graph.
exactMotionSet = obstacleAvoidance.planner.solveExactCandidates( ...
    initialState, goalState, limits, options, ...
    scene, stageTiming);
stageTiming = exactMotionSet.StageTiming;
result.SearchDiagnostics.DirectAttempt = exactMotionSet.DirectAttempt;
result.SearchDiagnostics.FixedClockExcursion = ...
    exactMotionSet.ExcursionDiagnostics;
if exactMotionSet.FastPath.Available
    fastPath = exactMotionSet.FastPath;
    result = finishFastPath(result, fastPath.Candidate, ...
        fastPath.Validation, fastPath.AttemptDetails, ...
        fastPath.ElapsedTime_s, fastPath.Seed, summaryTemplate, ...
        fastPath.Message, planningTimer, stageTiming);
    [result, diagnosis] = obstacleAvoidance.planner.createPublicOutputs(result, nargout > 1);
    return;
end

%% Section 6: Create Proposal Geometry And Search Routes

topologyTimer = tic;

proposal = struct();
visibilityGraph = struct();
routeSet = struct();
corridorBoundary_deg = zeros(0, 2);
needsRouteSearch = options.MaximumSeedCount > 1 && ...
    ~isempty(preparedObstacles);
if needsRouteSearch
    % Build proposal geometry for route search; final validation uses the original obstacles.
    proposal = obstacleAvoidance.search.createRouteSearchGeometry( ...
        initialState, goalState, options, ...
        scene);

    % Build the visibility graph and record its attempts.
    visibilityGraph = obstacleAvoidance.search.createVisibilityGraph( ...
        limits, ...
        proposal);

    % Search timed routes and distinct spatial routes.
    routeSet = obstacleAvoidance.search.searchRoutes( ...
        initialState, goalState, limits, options, ...
        scene, proposal, visibilityGraph);

    corridorBoundary_deg = proposal.shape.Vertices;
end
% Seed the general solver with a direct guess, then any searched detours.
seeds = obstacleAvoidance.search.createSeeds( ...
    initialState, goalState, limits, options, routeSet, corridorBoundary_deg);

stageTiming.TopologyElapsedTime_s = toc(topologyTimer);
seedSolveContext = struct( ...
    "UseStaticSolver", useStaticSolver, ...
    "SummaryTemplate", summaryTemplate);
% Try the first two ordinary seeds before failure recovery.
primarySeedCount = min(2, numel(seeds));
primarySeeds = seeds(1:primarySeedCount);
primarySummaries = repmat(summaryTemplate, primarySeedCount, 1);
primaryCandidates = cell(primarySeedCount, 1);
checkTemplate = obstacleAvoidance.validateTrajectory();
primaryChecks = repmat(checkTemplate, primarySeedCount, 1);
firstValidatedMotionTime_s = NaN;
for seedIndex = 1:primarySeedCount
    [primaryCandidates{seedIndex}, primarySummaries(seedIndex), ...
        primaryChecks(seedIndex), stageTiming] = ...
        obstacleAvoidance.planner.solveOneSeed( ...
            preparedObstacles, initialState, goalState, limits, options, ...
            primarySeeds(seedIndex), seedSolveContext, stageTiming);
    if primaryChecks(seedIndex).Passed && ...
            isnan(firstValidatedMotionTime_s)
        firstValidatedMotionTime_s = toc(planningTimer);
    end
end
candidateSet = struct( ...
    "Seeds", primarySeeds, ...
    "Candidates", {primaryCandidates}, ...
    "Summaries", primarySummaries, ...
    "CheckResults", primaryChecks, ...
    "FirstValidatedMotionTime_s", firstValidatedMotionTime_s, ...
    "StageTiming", stageTiming);

% Try additional seeds after failure, up to MaximumSeedCount.
recoveryContext = struct( ...
    "Scene", scene, ...
    "Proposal", proposal, ...
    "VisibilityGraph", visibilityGraph, ...
    "SeedSolveContext", seedSolveContext, ...
    "HasValidatedExactMotion", exactMotionSet.ExcursionIsValidated, ...
    "PlanningTimer", planningTimer);
[candidateSet, routeSet, generatedSeeds] = ...
    obstacleAvoidance.planner.recoverAdditionalSeeds( ...
        initialState, goalState, limits, options, ...
        candidateSet, routeSet, seeds, recoveryContext);

% Assemble diagnostics after recovery has added its routes and seeds.
gridDiagnostics = obstacleAvoidance.search.createSearchDiagnostics( ...
    proposal, visibilityGraph, routeSet, generatedSeeds);
gridDiagnostics.ElapsedTime_s = ...
    candidateSet.StageTiming.TopologyElapsedTime_s;
result.SearchDiagnostics.Grid = gridDiagnostics;

seeds = candidateSet.Seeds;
candidates = candidateSet.Candidates;
seedSummaries = candidateSet.Summaries;
firstValidatedMotionTime_s = ...
    candidateSet.FirstValidatedMotionTime_s;
stageTiming = candidateSet.StageTiming;

% Compare validated fixed-arrival motions by travel length.
if exactMotionSet.ExcursionIsValidated
    excursionCandidate = exactMotionSet.ExcursionCandidate;
    excursionDiagnostics = exactMotionSet.ExcursionDiagnostics;
    excursionElapsedTime_s = exactMotionSet.ExcursionElapsedTime_s;
    excursionSeed = exactMotionSet.ExcursionSeed;
    excursionCandidate.SeedIndex = numel(seeds) + 1;
    excursionSeed.Index = excursionCandidate.SeedIndex;
    seeds(end + 1) = excursionSeed;
    candidates{end + 1, 1} = excursionCandidate;
    seedSummaries(end + 1, 1) = ...
        obstacleAvoidance.planner.createCandidateSummary( ...
        excursionCandidate, excursionCandidate.Validation, ...
        excursionDiagnostics, excursionElapsedTime_s, summaryTemplate, ...
        limits);
end

%% Section 7: Select A Valid Motion Or Return Evidence

% Select only validated motions; keep a partial attempt for failure diagnostics.
selection = obstacleAvoidance.planner.selectValidatedCandidate( ...
    seedSummaries, options);

% Attach diagnostics and the selected validated motion, if any.
result.Seeds = seeds;
result.SeedSummaries = seedSummaries;

result.SearchDiagnostics.AttemptedSeedCount = numel(seeds);

result.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
result.SearchDiagnostics.ValidatedCandidateCount = ...
    selection.ValidatedCandidateCount;
result.SearchDiagnostics.BestPartialSeedIndex = ...
    selection.BestPartialSeedIndex;
result.Message = selection.Message;
result.TerminationReason = selection.TerminationReason;
if selection.Success
    selectedIndex = selection.SelectedCandidateIndex;
    result.Success = true;
    result.SelectedSeedIndex = selectedIndex;
    result.SelectedSeed_deg = seeds(selectedIndex).position_deg;
    result = copyMotion(result, candidates{selectedIndex});
end

result = obstacleAvoidance.planner.stageTiming( ...
    result, planningTimer, stageTiming);
[result, diagnosis] = obstacleAvoidance.planner.createPublicOutputs(result, nargout > 1);
end

%% Section 8: Local Functions

function result = finishFastPath(result, candidate, validation, diagnostics, ...
        elapsedTime_s, seed, summaryTemplate, message, timer, stageTiming)
% Assemble the validated fast-path result.
summary = obstacleAvoidance.planner.createCandidateSummary( ...
    candidate, validation, diagnostics, ...
    elapsedTime_s, summaryTemplate, result.Inputs.limits);
result.Success = true;
result.Message = message;
result.TerminationReason = "goalReached";
result.Seeds = seed;
result.SeedSummaries = summary;
result.SelectedSeedIndex = seed.Index;
result.SelectedSeed_deg = seed.position_deg;
result = copyMotion(result, candidate);
result.FirstValidatedMotionTime_s = toc(timer);

result.SearchDiagnostics.AttemptedSeedCount = 1;
result.SearchDiagnostics.ValidatedCandidateCount = 1;

result.SearchDiagnostics.BestPartialSeedIndex = seed.Index;

result = obstacleAvoidance.planner.stageTiming(result, timer, stageTiming);
end

function result = copyMotion(result, candidate)
% Copy the selected motion and arrival fields.
for name = ["time_s", "position_deg", "velocity_deg_s", ...
        "acceleration_deg_s2", "jerk_deg_s3", "Polynomial", ...
        "SeedCorridorBoundary_deg", "SeedCorridor", ...
        "PlaneCertificate", "Validation"]
    result.(name) = candidate.(name);
end
result.ArrivalTime_s = candidate.FinalTime_s;
result.TrajectoryDuration_s = candidate.MotionDuration_s;
end
