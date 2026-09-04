function result = planTrajectory( ...
        obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.planTrajectory()
%   result = obstacleAvoidance.planTrajectory( ...
%       obstacles, initialState, goalState, limits)
%   result = obstacleAvoidance.planTrajectory( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plan collision-free Az/El motion through one public entry point.
%   - Honor the resolved goal-time policy: balanced arrival applies an explicit
%     travel-saved-per-second exchange rate, fixed arrival minimizes travel at
%     the mission time, and earliest arrival minimizes time.
%**************************************************************************
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
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       The result contains success or failure data, motion, and diagnostics.
%   - options (scalar struct, zero-input call)
%       Fully resolved planner defaults.
%**************************************************************************
% UNITS
%   - Position is in degrees. Time is in seconds.
%   - Derivatives use deg/s, deg/s^2, and deg/s^3.
%   - Histories are N-by-2 [azimuth elevation] arrays.
%**************************************************************************

%% Section 1: Resolve Defaults Requests

% A call with no inputs requests the planner defaults. Resolve them from the same single source
% used by normal planning requests. This keeps the reported defaults equal to the values that
% the planner uses for a normal request.
if nargin == 0
    result = obstacleAvoidance.input.resolvePlannerOptions();
    return;
end

% One input does not define a planning problem. Report this case here. The
% caller then gets a direct input error before input normalization starts.
if nargin == 1
    error("planTrajectory:MissingInputs", ...
        "Planning requires obstacles, initialState, goalState, and limits.");
end

%% Section 2: Resolve The Planner Request

% Keep the four physical inputs in one fixed order. They describe the
% environment, initial motion, required final motion, and physical limits.
if nargin < 4
    error("planTrajectory:MissingInputs", ...
        "obstacles, initialState, goalState, and limits are required.");
end
% An omitted or empty option structure selects all default values. The internal
% planner merges partial options and validates each value.
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("planTrajectory:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end

%% Section 3: Create The Request And Prepare The Scene

% Search, motion construction, and final validation require one normalized
% request and one prepared obstacle history. Establish those representations
% before endpoint checks and fast paths so every later stage reads the same
% physical inputs and geometry.

planningTimer = tic;

% Search, motion construction, and final validation require the same resolved
% options and normalized physical inputs. Create one request here so later
% stages cannot interpret the caller's raw structures differently.
request = obstacleAvoidance.input.createPlanningRequest( ...
    obstacles, initialState, goalState, limits, optionOverrides);
obstacles = request.obstacles;
initialState = request.initialState;
goalState = request.goalState;
limits = request.limits;
options = request.options;
useRuckigWaypoint = options.TrajectoryMethod == "ruckigWaypoint";
[result, summaryTemplate] = obstacleAvoidance.planner.createEmptyResult( ...
    obstacles, initialState, goalState, limits, options, ...
    obstacleAvoidance.validateTrajectory());

% Graph construction and motion checks query obstacle histories many times.
% Prepare their shared shapes and horizon decision once, before any stage can
% take an early return or create an alternative representation.
scene = obstacleAvoidance.obstacles.preparePlanningScene(request);
preparedObstacles = scene.preparedObstacles;
useStaticKernel = scene.isStaticHorizon;
stageTiming = result.SearchDiagnostics.StageTiming;
result.SearchDiagnostics.StageOutputs.Scene = scene;
exactMotionSet = obstacleAvoidance.planner.solveExactCandidates();
result.SearchDiagnostics.DirectAttempt = exactMotionSet.DirectAttempt;
result.SearchDiagnostics.FixedClockExcursion = ...
    exactMotionSet.ExcursionDiagnostics;
result.SearchDiagnostics.SelectionPolicy = struct( ...
    "GoalTimeMode", options.GoalTimeMode, ...
    "MinimumTravelSavingsRate_deg_s", ...
    options.MinimumTravelSavingsRate_deg_s, ...
    "BalancedCost", "travel_deg + rate_deg_s * elapsed_s", ...
    "JerkRole", "hardConstraintOnly", ...
    "UtilizationTieBreak", ...
    "mean normalized peak velocity, acceleration, and jerk");
%% Section 4: Check Physical Endpoints

[endpointFeasible, result.Message, result.TerminationReason] = ...
    obstacleAvoidance.input.validatePlannerEndpoints( ...
    preparedObstacles, initialState, goalState, limits, options);
if ~endpointFeasible
    result.SearchDiagnostics.TerminationReason = result.TerminationReason;
    result = obstacleAvoidance.planner.stageTiming( ...
        result, planningTimer, stageTiming);
    return;
end

%% Section 5: Try Exact Physical-Time Motions

% Exact analytic motions can avoid graph construction entirely, but only after
% the same complete trajectory check used by all other candidates. Try the
% direct profile and enabled fixed-clock excursion before creating proposal
% geometry; retain every attempt even when route search must continue.
exactMotionSet = obstacleAvoidance.planner.solveExactCandidates( ...
    request, scene, stageTiming);
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
    return;
end

%% Section 6: Create Proposal Geometry And Search Routes

topologyTimer = tic;

proposal = struct();
visibilityGraph = struct();
routeSet = struct();
needsRouteSearch = options.MaximumSeedCount > 1 && ...
    ~isempty(preparedObstacles);
if needsRouteSearch
    % Obstacle histories are too detailed for affordable spatial graph work.
    % Create one explicit proposal representation, retaining whether it is a
    % sampled union or conservative dense envelope; this can suggest routes
    % but cannot approve any final motion.
    proposal = obstacleAvoidance.search.createProposalGeometry( ...
        scene, request);

    % Route search needs discrete nodes and collision-checked connections.
    % Preserve every offset attempt so rejected nodes, edges, and connectivity
    % recovery remain inspectable before any route is selected.
    visibilityGraph = obstacleAvoidance.search.createVisibilityGraph( ...
        proposal, request);
    obstacleAvoidance.input.throwIfCancellationRequested(options);

    % Moving histories may admit timed routes that a static envelope hides,
    % while distinct spatial routes preserve different obstacle-passing
    % choices. Search both forms before converting either into motion seeds.
    routeSet = obstacleAvoidance.search.searchRoutes( ...
        scene, request, proposal, visibilityGraph);

    % Visibility routes are only starting suggestions. Convert timed and
    % spatial routes into one deterministic seed order with explicit source,
    % duration, and reduced-geometry provenance for the motion solvers.
    seeds = obstacleAvoidance.search.createSeeds( ...
        routeSet, proposal, request);
else
    % A direct-only or obstacle-free request still uses the same seed creator
    % so source labels, timing estimates, and indices keep one owner.
    seeds = obstacleAvoidance.search.createSeeds([], [], request);
end

stageTiming.TopologyElapsedTime_s = toc(topologyTimer);
seedSolveContext = struct( ...
    "UseRuckigWaypoint", useRuckigWaypoint, ...
    "UseStaticKernel", useStaticKernel, ...
    "PreparedObstacles", preparedObstacles, ...
    "InitialState", initialState, ...
    "GoalState", goalState, ...
    "Limits", limits, ...
    "Options", options, ...
    "SummaryTemplate", summaryTemplate);
% Solve at most the first two ordinary seeds on the common path. A larger
% MaximumSeedCount enables failure-only recovery without making successful
% requests pay for additional motion solves.
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

% All work after the first two seeds is isolated here. Setting
% MaximumSeedCount to 2 disables ordinary later-seed recovery; values through
% 5 enable it only after the initial candidates fail full validation.
recoveryContext = struct( ...
    "Scene", scene, ...
    "Request", request, ...
    "Proposal", proposal, ...
    "VisibilityGraph", visibilityGraph, ...
    "SeedSolveContext", seedSolveContext, ...
    "HasValidatedExactMotion", exactMotionSet.ExcursionIsValidated, ...
    "PlanningTimer", planningTimer);
[candidateSet, routeSet, generatedSeeds] = ...
    obstacleAvoidance.planner.recoverAdditionalSeeds( ...
    candidateSet, routeSet, seeds, recoveryContext);

% Assemble diagnostics once, after optional recovery has returned every route
% and seed it generated. CandidateSet separately records which seeds ran.
gridDiagnostics = obstacleAvoidance.search.createSearchDiagnostics( ...
    proposal, visibilityGraph, routeSet, generatedSeeds);
gridDiagnostics.ElapsedTime_s = ...
    candidateSet.StageTiming.TopologyElapsedTime_s;
result.SearchDiagnostics.Grid = gridDiagnostics;
result.SearchDiagnostics.StageOutputs.Proposal = proposal;
result.SearchDiagnostics.StageOutputs.VisibilityGraph = visibilityGraph;
result.SearchDiagnostics.StageOutputs.RouteSet = routeSet;
result.SearchDiagnostics.StageOutputs.SeedSet = generatedSeeds;
result.SearchDiagnostics.SeedGenerationElapsedTime_s = ...
    gridDiagnostics.ElapsedTime_s;
seeds = candidateSet.Seeds;
candidates = candidateSet.Candidates;
seedSummaries = candidateSet.Summaries;
firstValidatedMotionTime_s = ...
    candidateSet.FirstValidatedMotionTime_s;
stageTiming = candidateSet.StageTiming;
result.SearchDiagnostics.StageOutputs.CandidateSet = candidateSet;

% Balanced and fixed policies compare every validated special motion against
% the topology candidates; their physical arrival lower bounds are not travel
% optimality certificates.
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
        limits, options, initialState.time_s);
end

%% Section 7: Select A Valid Motion Or Return Evidence

% Solver and proposal status cannot approve a motion. Restrict ranking to
% summaries whose full trajectory check passed, retaining a best partial
% attempt only as failure evidence when no candidate qualifies.
selection = obstacleAvoidance.planner.selectValidatedCandidate( ...
    seedSummaries, options);

% The final stage preserves one result shape on success and expected failure.
% It copies trajectory fields only from the selected fully checked candidate;
% plotting and callers therefore consume returned work without rerunning it.
result = obstacleAvoidance.planner.createPlannerResult( ...
    result, seeds, candidates, seedSummaries, selection, ...
    firstValidatedMotionTime_s, planningTimer, stageTiming);
end

%% Section 8: Local Functions

function result = finishFastPath(result, candidate, validation, diagnostics, ...
        elapsedTime_s, seed, summaryTemplate, message, timer, stageTiming)
% Assemble each independently accepted fast-path through one owner.
summary = obstacleAvoidance.planner.createCandidateSummary( ...
    candidate, validation, diagnostics, ...
    elapsedTime_s, summaryTemplate, result.Inputs.limits, result.Options, ...
    result.Inputs.initialState.time_s);
result.Success = true;
result.Message = message;
result.TerminationReason = "goalReached";
result.Seeds = seed;
result.SeedSummaries = summary;
result.SelectedSeedIndex = seed.Index;
result.SelectedSeed_deg = seed.position_deg;
result = copyMotion(result, candidate);
result.FirstValidatedMotionTime_s = toc(timer);
result.SearchDiagnostics.SeedSummaries = summary;
result.SearchDiagnostics.AttemptedSeedCount = 1;
result.SearchDiagnostics.ValidatedCandidateCount = 1;
result.SearchDiagnostics.FirstValidatedMotionTime_s = ...
    result.FirstValidatedMotionTime_s;
result.SearchDiagnostics.BestPartialSeedIndex = seed.Index;
result.SearchDiagnostics.TerminationReason = result.TerminationReason;
result = obstacleAvoidance.planner.stageTiming(result, timer, stageTiming);
end

function result = copyMotion(result, candidate)
% Copy the stable public motion payload and authoritative arrival fields.
for name = ["time_s", "position_deg", "velocity_deg_s", ...
        "acceleration_deg_s2", "jerk_deg_s3", "Polynomial", ...
        "SeedCorridorBoundary_deg", "SeedCorridor", ...
        "PlaneCertificate", "Validation"]
    result.(name) = candidate.(name);
end
result.ArrivalTime_s = candidate.FinalTime_s;
result.TrajectoryDuration_s = candidate.MotionDuration_s;
end
