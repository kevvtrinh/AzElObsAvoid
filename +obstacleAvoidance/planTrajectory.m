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
result.SearchDiagnostics.DirectAttempt = directAttemptTemplate();
[~, result.SearchDiagnostics.FixedClockExcursion] = ...
    obstacleAvoidance.planner.createFixedClockLateralExcursion();
result.SearchDiagnostics.SelectionPolicy = struct( ...
    "GoalTimeMode", options.GoalTimeMode, ...
    "MinimumTravelSavingsRate_deg_s", ...
    options.MinimumTravelSavingsRate_deg_s, ...
    "BalancedCost", "travel_deg + rate_deg_s * elapsed_s", ...
    "JerkRole", "hardConstraintOnly", ...
    "UtilizationTieBreak", ...
    "mean normalized peak velocity, acceleration, and jerk");
result.SearchDiagnostics.SeedEarlyExit = struct( ...
    "Applied", false, ...
    "Reason", "notApplicableExactPath", ...
    "PhysicalArrivalLowerBound_s", NaN, ...
    "ReachedBySeedIndex", 0);
excursionIsValidated = false;

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

obstacleAvoidance.input.throwIfCancellationRequested(options);
motionTimer = tic;
if useRuckigWaypoint
    directSeed = createDirectSeed(initialState, goalState, ...
        goalState.time_s - initialState.time_s);
    directSeed.Source = "ruckigDirect";
    [directCandidate, ~] = ...
        obstacleAvoidance.planner.createRuckigWaypointMotion( ...
        directSeed, initialState, goalState, limits, options);
else
    directCandidate = bmtpEngine.createDirectMotion( ...
        initialState, goalState, limits, options);
end
directElapsedTime_s = toc(motionTimer);
stageTiming.MotionSolvingElapsedTime_s = ...
    stageTiming.MotionSolvingElapsedTime_s + directElapsedTime_s;
[directCandidate, directValidation, directValidationTime_s, stageTiming] = ...
    obstacleAvoidance.planner.checkCandidateMotion( ...
    directCandidate, preparedObstacles, initialState, ...
    goalState, limits, options, stageTiming, "");
obstacleAvoidance.input.throwIfCancellationRequested(options);
directAttempt = recordDirectAttempt(directCandidate, directValidation, ...
    directElapsedTime_s, directValidationTime_s);
result.SearchDiagnostics.DirectAttempt = directAttempt;
if directValidation.Passed
    result = finishFastPath(result, directCandidate, directValidation, ...
        directAttempt, directElapsedTime_s, ...
        createDirectSeed(initialState, goalState, ...
        directCandidate.MotionDuration_s), summaryTemplate, ...
        "An exact direct rest-to-rest motion passed independent validation.", ...
        planningTimer, stageTiming);
    return;
end
result.SearchDiagnostics.DirectAttempt.FallbackContinued = true;

if ~useRuckigWaypoint
    obstacleAvoidance.input.throwIfCancellationRequested(options);
    motionTimer = tic;
    [excursionCandidate, excursionDiagnostics] = ...
        obstacleAvoidance.planner.createFixedClockLateralExcursion( ...
        directCandidate, preparedObstacles, initialState, goalState, ...
        limits, options, directValidation);
    excursionElapsedTime_s = toc(motionTimer);
    obstacleAvoidance.input.throwIfCancellationRequested(options);
    stageTiming = accountConstructorValidation( ...
        stageTiming, excursionElapsedTime_s, excursionDiagnostics);
    result.SearchDiagnostics.FixedClockExcursion = excursionDiagnostics;
    if excursionDiagnostics.Success && excursionCandidate.Validation.Passed
        excursionIsValidated = true;
        excursionSeed = createMotionSeed( ...
            excursionCandidate, "fixedClockLateralExcursion");
        if options.GoalTimeMode == "earliestArrival"
            result = finishFastPath(result, excursionCandidate, ...
                excursionCandidate.Validation, excursionDiagnostics, ...
                excursionElapsedTime_s, excursionSeed, summaryTemplate, ...
                "A fixed-clock lateral excursion attained the physical time floor.", ...
                planningTimer, stageTiming);
            return;
        end
    end
end

%% Section 6: Search And Solve Deterministic Seeds

topologyTimer = tic;
[seeds, gridDiagnostics, searchStages] = ...
    obstacleAvoidance.search.createRouteCandidates( ...
    scene, request);
gridDiagnostics.ElapsedTime_s = toc(topologyTimer);
stageTiming.TopologyElapsedTime_s = gridDiagnostics.ElapsedTime_s;
result.SearchDiagnostics.Grid = gridDiagnostics;
result.SearchDiagnostics.StageOutputs.Proposal = searchStages.Proposal;
result.SearchDiagnostics.StageOutputs.VisibilityGraph = ...
    searchStages.VisibilityGraph;
result.SearchDiagnostics.StageOutputs.RouteSet = searchStages.RouteSet;
result.SearchDiagnostics.StageOutputs.SeedSet = seeds;
result.SearchDiagnostics.SeedGenerationElapsedTime_s = ...
    gridDiagnostics.ElapsedTime_s;
seedSolveContext = struct( ...
    "UseRuckigWaypoint", useRuckigWaypoint, ...
    "UseStaticKernel", useStaticKernel, ...
    "PreparedObstacles", preparedObstacles, ...
    "InitialState", initialState, ...
    "GoalState", goalState, ...
    "Limits", limits, ...
    "Options", options, ...
    "SummaryTemplate", summaryTemplate);
hasFixedGoal = ~isfield(goalState, "targetTime_s") || ...
    isempty(goalState.targetTime_s);
physicalArrivalLowerBound_s = NaN;
if hasFixedGoal && isfinite(directCandidate.MotionDuration_s)
    physicalArrivalLowerBound_s = initialState.time_s + ...
        directCandidate.MotionDuration_s;
end
% Every geometric seed must become a timed candidate and pass the full motion
% check before selection. Solve them as one inspectable stage, retaining the
% existing proven earliest-arrival exit without hiding unattempted seeds.
candidateSet = obstacleAvoidance.planner.solveSeeds( ...
    seeds, seedSolveContext, stageTiming, ...
    physicalArrivalLowerBound_s, planningTimer);
seeds = candidateSet.Seeds;
candidates = candidateSet.Candidates;
seedSummaries = candidateSet.Summaries;
firstValidatedMotionTime_s = ...
    candidateSet.FirstValidatedMotionTime_s;
stageTiming = candidateSet.StageTiming;
result.SearchDiagnostics.SeedEarlyExit = candidateSet.SeedEarlyExit;
result.SearchDiagnostics.StageOutputs.CandidateSet = candidateSet;

% Balanced and fixed policies compare every validated special motion against
% the topology candidates; their physical arrival lower bounds are not travel
% optimality certificates.
if excursionIsValidated
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

function record = directAttemptTemplate()
% Define stable exact-direct evidence before the attempt runs.
record = struct("Identifier", "analyticRestToRest", ...
    "Attempted", false, "ProfileCreated", false, ...
    "ValidationAttempted", false, "ValidationPassed", false, ...
    "CollisionFree", false, "CollisionResolved", false, ...
    "FallbackContinued", false, "KernelTerminationReason", "notRun", ...
    "TerminationReason", "notRun", ...
    "Message", "The exact direct motion was not attempted.", ...
    "ElapsedTime_s", 0, "ValidationElapsedTime_s", 0, ...
    "MotionDuration_s", NaN, "MotionLength_deg", NaN, ...
    "MinimumAxisDuration_s", [NaN NaN], ...
    "StraightProgressMinimumDuration_s", NaN, ...
    "UsedStraightProgress", false);
end

function record = recordDirectAttempt(candidate, validation, elapsedTime_s, ...
        validationElapsedTime_s)
% Fill exact-direct kernel and authoritative-validation evidence.
record = directAttemptTemplate();
if isfield(candidate, "SolverDiagnostics") && ...
        isfield(candidate.SolverDiagnostics, "Identifier")
    record.Identifier = candidate.SolverDiagnostics.Identifier;
end
record.Attempted = true;
record.ProfileCreated = ~isempty(candidate.time_s);
record.ValidationAttempted = record.ProfileCreated;
record.ValidationPassed = validation.Passed;
record.CollisionFree = validation.CollisionFree;
record.CollisionResolved = validation.CollisionResolved;
record.KernelTerminationReason = candidate.TerminationReason;
record.TerminationReason = candidate.TerminationReason;
record.Message = candidate.Message;
record.ElapsedTime_s = elapsedTime_s;
record.ValidationElapsedTime_s = validationElapsedTime_s;
for name = ["MotionDuration_s", "MotionLength_deg", ...
        "MinimumAxisDuration_s", "StraightProgressMinimumDuration_s", ...
        "UsedStraightProgress"]
    record.(name) = candidate.(name);
end
if record.ValidationAttempted && ~validation.Passed
    record.TerminationReason = "directValidationFailed";
    record.Message = strtrim(candidate.Message + " " + validation.Message);
elseif validation.Passed
    record.TerminationReason = "goalReached";
    record.Message = validation.Message;
end
end

function seed = createDirectSeed(initialState, goalState, duration_s, wait_s)
% Create the ordinary endpoint seed, adding a truthful waiting vertex if used.
position_deg = [initialState.position_deg; goalState.position_deg];
tau = [0; 1];
if nargin >= 4 && wait_s > 0
    position_deg = [initialState.position_deg; position_deg];
    tau = [0; wait_s / duration_s; 1];
end
seed = obstacleAvoidance.search.createSeed();
seed.Index = 1;
seed.Source = "directRestToRest";
[seed.position_deg, seed.tau] = deal(position_deg, tau);
seed.EstimatedDuration_s = duration_s;
seed.Length_deg = norm(diff(position_deg, 1, 1));
end

function seed = createMotionSeed(candidate, source)
% Preserve an accepted curved motion instead of labeling its blocked chord.
time_s = double(candidate.time_s(:));
duration_s = candidate.MotionDuration_s;
tau = (time_s - time_s(1)) / duration_s;
seed = obstacleAvoidance.search.createSeed();
seed.Index = 1;
seed.Source = string(source);
[seed.position_deg, seed.tau] = deal(candidate.position_deg, tau);
seed.EstimatedDuration_s = duration_s;
seed.Length_deg = candidate.MotionLength_deg;
end

function stageTiming = accountConstructorValidation( ...
        stageTiming, constructorElapsedTime_s, diagnostics)
% Move nested authoritative validation from constructor work into its stages.
validationElapsedTime_s = 0;
collisionElapsedTime_s = 0;
if isfield(diagnostics, "ValidationElapsedTime_s")
    validationElapsedTime_s = double(diagnostics.ValidationElapsedTime_s);
end
if isfield(diagnostics, "CollisionCheckingElapsedTime_s")
    collisionElapsedTime_s = double(diagnostics.CollisionCheckingElapsedTime_s);
end
tolerance_s = 256 * eps(max(1, constructorElapsedTime_s));
if validationElapsedTime_s > constructorElapsedTime_s + tolerance_s || ...
        collisionElapsedTime_s > validationElapsedTime_s + tolerance_s
    error("planCorridorQuintic:InvalidConstructorTiming", ...
        "Nested validation timing exceeds its constructor or validation total.");
end
validationElapsedTime_s = min(validationElapsedTime_s, constructorElapsedTime_s);
collisionElapsedTime_s = min(collisionElapsedTime_s, validationElapsedTime_s);
stageTiming.MotionSolvingElapsedTime_s = ...
    stageTiming.MotionSolvingElapsedTime_s + ...
    constructorElapsedTime_s - validationElapsedTime_s;
stageTiming.CollisionCheckingElapsedTime_s = ...
    stageTiming.CollisionCheckingElapsedTime_s + collisionElapsedTime_s;
stageTiming.FinalValidationElapsedTime_s = ...
    stageTiming.FinalValidationElapsedTime_s + ...
    validationElapsedTime_s - collisionElapsedTime_s;
end

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
