function [result, diagnosis] = planTrajectory(obstacles, initialState, goalState, limits, optionOverrides)
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
%       Failure retains rejected motion when available; Success remains false.
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
    result    = obstacleAvoidance.input.resolvePlannerOptions();
    diagnosis = struct();
    return;
end

%% Section 2: Resolve The Planner Request

% Require obstacles, initial state, goal state, and limits.
if nargin < 4
    error("planTrajectory:MissingInputs", "obstacles, initialState, goalState, and limits are required.");
end
% Use defaults when options are omitted or empty.
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end

%% Section 3: Normalize The Request And Prepare The Scene

planningTimer = tic;

% Normalize the planning inputs.
options = obstacleAvoidance.input.resolvePlannerOptions(optionOverrides);

[obstacles, initialState, goalState, limits] = obstacleAvoidance.input.normalizePlannerRequest(obstacles, initialState, goalState, limits, options);

[result, summaryTemplate] = obstacleAvoidance.planner.createPlanningRecord(obstacles, initialState, goalState, limits, options, obstacleAvoidance.validateTrajectory());

% Prepare shared obstacle geometry once for search and validation.
scene = obstacleAvoidance.obstacles.preparePlanningScene(obstacles, initialState, goalState);

preparedObstacles = scene.preparedObstacles;
useStaticSolver   = scene.obstaclesRemainStatic;
stageTiming       = result.SearchDiagnostics.StageTiming;
result.SearchDiagnostics.SelectionPolicy = struct("GoalTimeMode", options.GoalTimeMode, "JerkRole", "hardConstraintOnly");
%% Section 4: Check Physical Endpoints

[endpointFeasible, result.Message, result.TerminationReason] = obstacleAvoidance.input.validatePlannerEndpoints(preparedObstacles, initialState, goalState, limits, options);
% Reject the request before route search when endpoint states violate motion limits.
if ~endpointFeasible
    emptyMotions = obstacleAvoidance.planner.tryDirectAndFixedTimeMotions();
    result.SearchDiagnostics.DirectAttempt       = emptyMotions.DirectAttempt;
    result.SearchDiagnostics.FixedClockExcursion = emptyMotions.ExcursionDiagnostics;

    result = finalizePlanningTiming(result, planningTimer, stageTiming);
    [result, diagnosis] = obstacleAvoidance.planner.assemblePlannerOutputs(result, nargout > 1);
    return;
end

%% Section 5: Try Exact Physical-Time Motions

% Try validated direct and fixed-clock motions before building the graph.
exactMotionSet             = obstacleAvoidance.planner.tryDirectAndFixedTimeMotions(initialState, goalState, limits, options, scene, stageTiming);
stageTiming                = exactMotionSet.StageTiming;
firstValidatedMotionTime_s = NaN;
if exactMotionSet.ExcursionIsValidated
    firstValidatedMotionTime_s = toc(planningTimer);
end
result.SearchDiagnostics.DirectAttempt       = exactMotionSet.DirectAttempt;
result.SearchDiagnostics.FixedClockExcursion = exactMotionSet.ExcursionDiagnostics;
if exactMotionSet.FastPath.Available
    fastPath = exactMotionSet.FastPath;
    result   = finishFastPath(result, fastPath.Candidate, fastPath.Validation, fastPath.AttemptDetails, fastPath.ElapsedTime_s, fastPath.Seed, summaryTemplate, fastPath.Message, planningTimer, stageTiming);
    [result, diagnosis] = obstacleAvoidance.planner.assemblePlannerOutputs(result, nargout > 1);
    return;
end

%% Section 6: Create Proposal Geometry And Search Routes

routeSearchTimer = tic;

proposal             = struct();
visibilityGraph      = struct();
routeSet             = struct();
obstacleEnvelope_deg = zeros(0, 2);
needsRouteSearch     = options.MaximumSeedCount > 1 && ~isempty(preparedObstacles);
if needsRouteSearch
    % Build proposal geometry for route search; final validation uses the original obstacles.
    proposal = obstacleAvoidance.search.createRouteSearchGeometry(initialState, goalState, options, scene);

    % Build the visibility graph and record its attempts.
    visibilityGraph = obstacleAvoidance.search.createVisibilityGraph(limits, proposal);

    % Search timed routes and distinct spatial routes.
    routeSet = obstacleAvoidance.search.searchRoutes(initialState, goalState, limits, options, scene, proposal, visibilityGraph);

    obstacleEnvelope_deg = proposal.shape.Vertices;
end
% Seed the general solver with a direct guess, then any searched detours.
seeds = obstacleAvoidance.search.createPathGuesses(initialState, goalState, limits, options, routeSet, obstacleEnvelope_deg);

stageTiming.RouteSearchElapsedTime_s = toc(routeSearchTimer);
endpointDerivative = [initialState.velocity_deg_s, initialState.acceleration_deg_s2, goalState.velocity_deg_s, goalState.acceleration_deg_s2];
useStateToStateSolver = any(abs(endpointDerivative) > options.ConstraintTolerance);
seedSolveContext = struct("UseStaticSolver", useStaticSolver, ...
    "UseStateToStateSolver", useStateToStateSolver, ...
    "SummaryTemplate", summaryTemplate, ...
    "StaticGeometry", struct(), "EnclosureGeometry", struct(), "Enclosure", struct());
% Try the first two ordinary seeds before failure recovery.
primarySeedCount  = min(2, numel(seeds));
primarySeeds      = seeds(1:primarySeedCount);
primarySummaries  = repmat(summaryTemplate, primarySeedCount, 1);
primaryCandidates = cell(primarySeedCount, 1);
% Evaluate each seed before retaining the best admissible candidate.
for seedIndex = 1:primarySeedCount
    [primaryCandidates{seedIndex}, primarySummaries(seedIndex), ...
        stageTiming, seedSolveContext] = obstacleAvoidance.planner.solvePathGuess(preparedObstacles, initialState, goalState, limits, options, primarySeeds(seedIndex), seedSolveContext, stageTiming);
    % Record the first validation time once; later successful candidates must not overwrite that milestone.
    if primarySummaries(seedIndex).ValidationPassed && isnan(firstValidatedMotionTime_s)
        firstValidatedMotionTime_s = toc(planningTimer);
    end
end
candidateSet = struct("Seeds", primarySeeds, ...
    "Candidates", {primaryCandidates}, ...
    "Summaries", primarySummaries, ...
    "FirstValidatedMotionTime_s", firstValidatedMotionTime_s, ...
    "StageTiming", stageTiming);

% Try additional seeds after failure, up to MaximumSeedCount.
recoveryContext = struct("Scene", scene, ...
    "Proposal", proposal, ...
    "VisibilityGraph", visibilityGraph, ...
    "SeedSolveContext", seedSolveContext, ...
    "HasValidatedExactMotion", exactMotionSet.ExcursionIsValidated, ...
    "PlanningTimer", planningTimer);
[candidateSet, routeSet, generatedSeeds] = obstacleAvoidance.planner.tryAdditionalPathGuesses(initialState, goalState, limits, options, candidateSet, routeSet, seeds, recoveryContext);

% Assemble diagnostics after recovery has added its routes and seeds.
searchDiagnostics = obstacleAvoidance.search.createSearchDiagnostics(proposal, visibilityGraph, routeSet, generatedSeeds);
searchDiagnostics.ElapsedTime_s = candidateSet.StageTiming.RouteSearchElapsedTime_s;
result.SearchDiagnostics.GraphSearch = searchDiagnostics;

seeds                      = candidateSet.Seeds;
candidates                 = candidateSet.Candidates;
seedSummaries              = candidateSet.Summaries;
firstValidatedMotionTime_s = candidateSet.FirstValidatedMotionTime_s;
stageTiming                = candidateSet.StageTiming;

% Compare validated fixed-arrival motions by travel length.
if exactMotionSet.ExcursionIsValidated
    excursionCandidate     = exactMotionSet.ExcursionCandidate;
    excursionDiagnostics   = exactMotionSet.ExcursionDiagnostics;
    excursionElapsedTime_s = exactMotionSet.ExcursionElapsedTime_s;
    excursionSeed          = exactMotionSet.ExcursionSeed;
    excursionCandidate.SeedIndex = numel(seeds) + 1;
    excursionSeed.Index = excursionCandidate.SeedIndex;
    seeds(end + 1) = excursionSeed;
    candidates{end + 1, 1} = excursionCandidate;
    seedSummaries(end + 1, 1) = obstacleAvoidance.planner.createCandidateSummary(excursionCandidate, excursionCandidate.Validation, excursionDiagnostics, excursionElapsedTime_s, summaryTemplate, limits);
end

%% Section 7: Select A Valid Motion Or Return Evidence

% Select only validated motions; keep a partial attempt for failure diagnostics.
selection = obstacleAvoidance.planner.selectValidatedCandidate(seedSummaries, options);
result.SearchDiagnostics.SelectionPolicy          = selection.Ranking;
result.SearchDiagnostics.SelectionPolicy.JerkRole = "hardConstraintOnly";

% Attach diagnostics and the selected validated motion, if any.
result.Seeds         = seeds;
result.SeedSummaries = seedSummaries;

result.SearchDiagnostics.AttemptedSeedCount = numel(seeds);

result.FirstValidatedMotionTime_s                = firstValidatedMotionTime_s;
result.SearchDiagnostics.ValidatedCandidateCount = selection.ValidatedCandidateCount;
result.SearchDiagnostics.BestPartialSeedIndex    = selection.BestPartialSeedIndex;
result.Message                                   = selection.Message;
result.TerminationReason                         = selection.TerminationReason;
% Publish the selected validated motion on success; failure branches retain partial-route diagnostics instead.
if selection.Success
    selectedIndex = selection.SelectedCandidateIndex;
    result.Success           = true;
    result.SelectedSeedIndex = selectedIndex;
    result.SelectedSeed_deg  = seeds(selectedIndex).position_deg;
    result = copyMotion(result, candidates{selectedIndex});
% Expose the best partial seed only when no complete candidate succeeded.
elseif selection.BestPartialSeedIndex > 0
    partialCandidate = candidates{selection.BestPartialSeedIndex};
    if ~isempty(partialCandidate.time_s)
        % Keep rejected motion and its failed validation available for inspection.
        result = copyMotion(result, partialCandidate);
    end
end

result = finalizePlanningTiming(result, planningTimer, stageTiming);
[result, diagnosis] = obstacleAvoidance.planner.assemblePlannerOutputs(result, nargout > 1);
end

%% Section 8: Local Functions

function result = finishFastPath(result, candidate, validation, diagnostics, elapsedTime_s, seed, summaryTemplate, message, timer, stageTiming)
    % Assemble the validated fast-path result.
    summary   = obstacleAvoidance.planner.createCandidateSummary(candidate, validation, diagnostics, elapsedTime_s, summaryTemplate, result.Inputs.limits);
    selection = obstacleAvoidance.planner.selectValidatedCandidate(summary, result.Options);
    result.SearchDiagnostics.SelectionPolicy          = selection.Ranking;
    result.SearchDiagnostics.SelectionPolicy.JerkRole = "hardConstraintOnly";
    result.Success                                    = true;
    result.Message                                    = message;
    result.TerminationReason                          = "goalReached";
    result.Seeds                                      = seed;
    result.SeedSummaries                              = summary;
    result.SelectedSeedIndex                          = seed.Index;
    result.SelectedSeed_deg                           = seed.position_deg;
    result = copyMotion(result, candidate);
    result.FirstValidatedMotionTime_s = toc(timer);

    result.SearchDiagnostics.AttemptedSeedCount      = 1;
    result.SearchDiagnostics.ValidatedCandidateCount = 1;

    result.SearchDiagnostics.BestPartialSeedIndex = seed.Index;

    result = finalizePlanningTiming(result, timer, stageTiming);
end

function result = copyMotion(result, candidate)
    % Copy the selected motion and arrival fields.
    for name = ["time_s", "position_deg", "velocity_deg_s", ...
            "acceleration_deg_s2", "jerk_deg_s3", "Polynomial", ...
            "SeedCorridorBoundary_deg", "SeedCorridor", ...
            "PlaneCertificate", "Validation"]
        result.(name) = candidate.(name);
    end
    result.ArrivalTime_s        = candidate.ArrivalTime_s;
    result.TrajectoryDuration_s = candidate.TrajectoryDuration_s;
end

function result = finalizePlanningTiming(result, planningTimer, timing)
    % Attach reconciled stage times to the planning record.
    result.SearchDiagnostics.StageTiming = obstacleAvoidance.planner.reconcileStageTiming(timing, toc(planningTimer));
    result.ElapsedPlanningTime_s         = result.SearchDiagnostics.StageTiming.TotalElapsedTime_s;
end
