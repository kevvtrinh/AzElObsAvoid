function result = planCorridorQuintic( ...
        obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.planner.planCorridorQuintic()
%   result = obstacleAvoidance.planner.planCorridorQuintic( ...
%       obstacles, initialState, goalState, limits)
%   result = obstacleAvoidance.planner.planCorridorQuintic( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Try exact direct and fixed-clock motions before topology work.
%   - Exhaust deterministic topology seeds through the static BMTP fallback.
%   - Promote only motions that pass independent public validation.
%**************************************************************************
% INPUTS
%   - obstacles (supported obstacle input or [])
%       Static or time-varying protected geometry.
%   - initialState, goalState (scalar structs)
%       Normalized endpoint motion states and goal-time policy.
%   - limits (scalar struct)
%       Two-axis physical and workspace limits with units in field names.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial public options; omitted and empty fields use defaults.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable success-or-failure record. Invalid inputs throw; expected
%       planning failures return Success=false with retained search evidence.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Histories are N-by-2
%     [azimuth elevation]; derivative units are deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Normalize The Request

if nargin == 0
    result = obstacleAvoidance.input.resolvePlannerOptions();
    return;
end
if nargin < 4
    error("planCorridorQuintic:MissingInputs", ...
        "obstacles, initialState, goalState, and limits are required.");
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
planningTimer = tic;
options = obstacleAvoidance.input.resolvePlannerOptions(optionOverrides);
obstacleAvoidance.input.throwIfCancellationRequested(options);
useRuckigWaypoint = options.TrajectoryMethod == "ruckigWaypoint";
[obstacles, initialState, goalState, limits] = ...
    obstacleAvoidance.input.normalizePlannerRequest( ...
    obstacles, initialState, goalState, limits, options);
[preparedObstacles, publicObstacles] = prepareRequestObstacles(obstacles);
[result, summaryTemplate] = obstacleAvoidance.planner.createEmptyResult( ...
    publicObstacles, initialState, goalState, limits, options, ...
    obstacleAvoidance.validateTrajectory());
useStaticKernel = obstacleAvoidance.obstacles.queryStaticHorizon( ...
    preparedObstacles, initialState.time_s, goalState.time_s);
stageTiming = result.SearchDiagnostics.StageTiming;
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
firstValidatedMotionTime_s = NaN;
excursionIsValidated = false;

%% Section 2: Reject Physically Invalid Endpoints

[endpointFeasible, result.Message, result.TerminationReason] = ...
    obstacleAvoidance.input.validatePlannerEndpoints( ...
    preparedObstacles, initialState, goalState, limits, options);
if ~endpointFeasible
    result.SearchDiagnostics.TerminationReason = result.TerminationReason;
    result = obstacleAvoidance.planner.stageTiming( ...
        result, planningTimer, stageTiming);
    return;
end

%% Section 3: Try Exact Physical-Time Motions

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
    validateCandidate(directCandidate, preparedObstacles, initialState, ...
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

%% Section 4: Exhaust Deterministic Topology Seeds

topologyTimer = tic;
[seeds, gridDiagnostics] = ...
    obstacleAvoidance.search.createRouteCandidates( ...
    preparedObstacles, initialState, goalState, limits, options);
gridDiagnostics.ElapsedTime_s = toc(topologyTimer);
stageTiming.TopologyElapsedTime_s = gridDiagnostics.ElapsedTime_s;
result.SearchDiagnostics.Grid = gridDiagnostics;
result.SearchDiagnostics.SeedGenerationElapsedTime_s = ...
    gridDiagnostics.ElapsedTime_s;
seedCount = numel(seeds);
seedSummaries = repmat(summaryTemplate, seedCount, 1);
candidates = cell(seedCount, 1);
sweptSeedIsEligible = false(seedCount, 1);
staticBmtpRepresentation = struct();
sweptBmtpRepresentation = struct();
projection = struct();
if useStaticKernel && seedCount > 0
    staticBmtpRepresentation = ...
        obstacleAvoidance.planner.createBmtpStaticRepresentation( ...
        preparedObstacles, initialState.time_s, goalState.time_s);
    result.SearchDiagnostics.StaticRepresentationCreationCount = 1;
elseif ~useRuckigWaypoint
    for seedIndex = 1:seedCount
        sweptSeedIsEligible(seedIndex) = ...
            string(seeds(seedIndex).Source) ~= "directWait" && ...
            size(seeds(seedIndex).position_deg, 1) > 2;
    end
    if any(sweptSeedIsEligible)
        [planningObstacles, projection] = ...
            obstacleAvoidance.obstacles.createStaticPlanningProjection( ...
            preparedObstacles, initialState.time_s, goalState.time_s);
        sweptBmtpRepresentation = ...
            obstacleAvoidance.planner.createBmtpStaticRepresentation( ...
            planningObstacles, initialState.time_s, goalState.time_s);
        result.SearchDiagnostics.StaticProjectionCreationCount = 1;
        result.SearchDiagnostics.StaticRepresentationCreationCount = 1;
    end
end
for seedIndex = 1:seedCount
    obstacleAvoidance.input.throwIfCancellationRequested(options);
    motionTimer = tic;
    candidateWasPrevalidated = false;
    prevalidationElapsedTime_s = 0;
    validation = obstacleAvoidance.validateTrajectory();
    if useRuckigWaypoint
        [candidate, solverDiagnostics] = ...
            obstacleAvoidance.planner.createRuckigWaypointMotion( ...
            seeds(seedIndex), initialState, goalState, limits, options);
    elseif useStaticKernel
        kernelGoalState = createFixedKernelGoalState(goalState, options);
        [candidate, solverDiagnostics] = ...
            obstacleAvoidance.planner.solveBmtpTrajectory( ...
            seeds(seedIndex), staticBmtpRepresentation, initialState, ...
            kernelGoalState, limits, options);
    else
        trySweptProjection = sweptSeedIsEligible(seedIndex);
        sweptAttempt = struct();
        timedBmtpAttempt = struct();
        if trySweptProjection
            kernelGoalState = createFixedKernelGoalState(goalState, options);
            [sweptCandidate, sweptDiagnostics] = ...
                obstacleAvoidance.planner.solveBmtpTrajectory( ...
                seeds(seedIndex), sweptBmtpRepresentation, initialState, ...
                kernelGoalState, limits, options);
            [sweptCandidate, sweptValidation, sweptValidationTime_s, ...
                stageTiming] = validateCandidate( ...
                sweptCandidate, preparedObstacles, initialState, ...
                goalState, limits, options, stageTiming, ...
                "The swept-projection BMTP kernel returned no trajectory.");
            prevalidationElapsedTime_s = prevalidationElapsedTime_s + ...
                sweptValidationTime_s;
            sweptAttempt = createSweptProjectionRecord( ...
                sweptDiagnostics, sweptValidation, projection);
            if sweptValidation.Passed
                candidate = sweptCandidate;
                validation = sweptValidation;
                solverDiagnostics = sweptDiagnostics;
                solverDiagnostics.SweptProjection = sweptAttempt;
                solverDiagnostics.DynamicObstacleRepresentation = ...
                    "conservativeStaticProtectedHistoryConvexHull";
                candidate.SolverDiagnostics = solverDiagnostics;
                candidateWasPrevalidated = true;
            end
        end
        tryTimedBmtp = trySweptProjection && ~candidateWasPrevalidated && ...
            string(seeds(seedIndex).Source) == "timeExpandedVisibilityGraph";
        if tryTimedBmtp
            [timedCandidate, timedBmtpDiagnostics] = ...
                obstacleAvoidance.planner.solveTimedBmtpTrajectory( ...
                seeds(seedIndex), preparedObstacles, initialState, ...
                goalState, limits, options);
            [timedCandidate, timedValidation, timedValidationTime_s, ...
                stageTiming] = validateCandidate( ...
                timedCandidate, preparedObstacles, initialState, ...
                goalState, limits, options, stageTiming, ...
                "The timed-cell BMTP kernel returned no trajectory.");
            prevalidationElapsedTime_s = prevalidationElapsedTime_s + ...
                timedValidationTime_s;
            timedBmtpAttempt = struct( ...
                "Attempted", true, ...
                "SolverDiagnostics", timedBmtpDiagnostics, ...
                "FullObstacleValidation", timedValidation, ...
                "Outcome", "rejectedByFullValidation");
            if timedValidation.Passed
                timedBmtpAttempt.Outcome = "acceptedAfterFullValidation";
                candidate = timedCandidate;
                validation = timedValidation;
                solverDiagnostics = timedBmtpDiagnostics;
                solverDiagnostics.SweptProjection = sweptAttempt;
                solverDiagnostics.TimedBmtp = timedBmtpAttempt;
                candidate.SolverDiagnostics = solverDiagnostics;
                candidateWasPrevalidated = true;
            end
        end
        if ~candidateWasPrevalidated
            [candidate, solverDiagnostics] = createTimedSeedCandidate( ...
                seeds(seedIndex), initialState, goalState, limits, ...
                options, [], []);
            if trySweptProjection
                solverDiagnostics.SweptProjection = sweptAttempt;
                solverDiagnostics.TimedBmtp = timedBmtpAttempt;
                candidate.SolverDiagnostics = solverDiagnostics;
            end
        end
        timedTerminationReason = string(candidate.TerminationReason);
        timedTopologyIsUnsupported = any(timedTerminationReason == ...
            ["unsupportedTimedMultiWaypointRoute", ...
            "invalidDirectWaitSeed"]);
        if timedTopologyIsUnsupported
            timedDiagnostics = solverDiagnostics;
            if options.UnsupportedTimedTopologyPolicy == ...
                    "ruckigStopAtWaypoints"
                [candidate, fallbackDiagnostics] = ...
                    obstacleAvoidance.planner.createRuckigWaypointMotion( ...
                    seeds(seedIndex), initialState, goalState, limits, ...
                    options);
                solverDiagnostics = combineFallbackDiagnostics( ...
                    timedDiagnostics, fallbackDiagnostics, ...
                    timedTerminationReason, true);
                if fallbackDiagnostics.Accepted
                    candidate.Message = candidate.Message + ...
                        " Every interior waypoint was constrained to rest " + ...
                        "by the explicitly enabled Ruckig fallback.";
                else
                    candidate.Message = ...
                        "The explicitly enabled Ruckig stop-at-waypoints " + ...
                        "fallback failed. " + candidate.Message;
                    candidate.TerminationReason = ...
                        "ruckigWaypointFallbackFailed";
                    solverDiagnostics.FallbackOutcome = ...
                        candidate.TerminationReason;
                end
                candidate.SolverDiagnostics = solverDiagnostics;
            else
                solverDiagnostics = combineFallbackDiagnostics( ...
                    timedDiagnostics, struct(), ...
                    timedTerminationReason, false);
                candidate.SolverDiagnostics = solverDiagnostics;
            end
        end
    end
    elapsedTime_s = toc(motionTimer) - prevalidationElapsedTime_s;
    obstacleAvoidance.input.throwIfCancellationRequested(options);
    stageTiming.MotionSolvingElapsedTime_s = ...
        stageTiming.MotionSolvingElapsedTime_s + elapsedTime_s;
    if ~candidateWasPrevalidated
        [candidate, validation, ~, stageTiming] = validateCandidate( ...
            candidate, preparedObstacles, initialState, goalState, limits, ...
            options, stageTiming, "The motion kernel returned no trajectory.");
        if validation.Passed && string(seeds(seedIndex).Source) == ...
                "directWait" && options.GoalTimeMode == "earliestArrival"
            [candidate, validation, solverDiagnostics, ...
                refinementElapsedTime_s, stageTiming] = refineDirectWait( ...
                seeds(seedIndex), candidate, validation, solverDiagnostics, ...
                preparedObstacles, initialState, goalState, limits, options, ...
                stageTiming);
            elapsedTime_s = elapsedTime_s + refinementElapsedTime_s;
            stageTiming.MotionSolvingElapsedTime_s = ...
                stageTiming.MotionSolvingElapsedTime_s + ...
                refinementElapsedTime_s;
        end
    end
    summary = summarizeCandidate(candidate, validation, ...
        solverDiagnostics, elapsedTime_s, summaryTemplate, ...
        limits, options, initialState.time_s);
    if ~seedSummaries(seedIndex).ValidationPassed || ...
            (validation.Passed && isBetterSummary( ...
            summary, seedSummaries(seedIndex), options))
        candidates{seedIndex} = candidate;
        seedSummaries(seedIndex) = summary;
    end
    if validation.Passed && isnan(firstValidatedMotionTime_s)
        firstValidatedMotionTime_s = toc(planningTimer);
    end
end

% Balanced and fixed policies compare every validated special motion against
% the topology candidates; their physical arrival lower bounds are not travel
% optimality certificates.
if excursionIsValidated
    excursionCandidate.SeedIndex = numel(seeds) + 1;
    excursionSeed.Index = excursionCandidate.SeedIndex;
    seeds(end + 1) = excursionSeed;
    candidates{end + 1, 1} = excursionCandidate;
    seedSummaries(end + 1, 1) = summarizeCandidate( ...
        excursionCandidate, excursionCandidate.Validation, ...
        excursionDiagnostics, excursionElapsedTime_s, summaryTemplate, ...
        limits, options, initialState.time_s);
end

result.Seeds = seeds;

%% Section 5: Select A Valid Motion Or Return Evidence

result.SeedSummaries = seedSummaries;
result.SearchDiagnostics.SeedSummaries = seedSummaries;
result.SearchDiagnostics.AttemptedSeedCount = numel(seeds);
result.SearchDiagnostics.FirstValidatedMotionTime_s = ...
    firstValidatedMotionTime_s;
result.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
validatedIndices = find([seedSummaries.ValidationPassed]).';
result.SearchDiagnostics.ValidatedCandidateCount = numel(validatedIndices);
if isempty(validatedIndices)
    bestIndex = bestPartialSeed(seedSummaries);
    result.SearchDiagnostics.BestPartialSeedIndex = bestIndex;
    result.Message = "No compact motion passed independent validation.";
    result.TerminationReason = "noValidatedSeed";
    if bestIndex > 0
        bestReason = string(seedSummaries(bestIndex).TerminationReason);
        if bestReason == "unsupportedTimedMultiWaypointRoute"
            result.TerminationReason = bestReason;
            result.Message = "A geometric route was found, but the smooth " + ...
                "timed-motion kernel does not yet support its multi-waypoint " + ...
                "topology. The stop-at-waypoint fallback was disabled by policy.";
        elseif strlength(seedSummaries(bestIndex).Message) > 0
            result.Message = result.Message + " Best attempt: " + ...
                seedSummaries(bestIndex).Message;
        end
    end
    result.SearchDiagnostics.TerminationReason = result.TerminationReason;
    result = obstacleAvoidance.planner.stageTiming( ...
        result, planningTimer, stageTiming);
    return;
end
selectedIndex = selectValidatedCandidate( ...
    seedSummaries, validatedIndices, options);
selectedCandidate = candidates{selectedIndex};
result.Success = true;
result.Message = "A validated motion was found.";
result.TerminationReason = "goalReached";
result.SelectedSeedIndex = selectedIndex;
result.SelectedSeed_deg = seeds(selectedIndex).position_deg;
result = copyMotion(result, selectedCandidate);
result.SearchDiagnostics.TerminationReason = result.TerminationReason;
result.SearchDiagnostics.BestPartialSeedIndex = selectedIndex;
result = obstacleAvoidance.planner.stageTiming( ...
    result, planningTimer, stageTiming);
end

%% Section 6: Local Functions

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

function [candidate, validation, elapsedTime_s, stageTiming] = ...
        validateCandidate(candidate, obstacles, initialState, goalState, ...
        limits, options, stageTiming, emptyMessage)
% Run and time the sole authoritative acceptance check for one candidate.
validation = obstacleAvoidance.validateTrajectory();
elapsedTime_s = 0;
if isempty(candidate.time_s)
    if strlength(emptyMessage) > 0
        validation.Message = emptyMessage;
    end
else
    validationTimer = tic;
    validation = obstacleAvoidance.validateTrajectory(candidate, obstacles, ...
        initialState, goalState, limits, options);
    elapsedTime_s = toc(validationTimer);
    stageTiming.CollisionCheckingElapsedTime_s = ...
        stageTiming.CollisionCheckingElapsedTime_s + ...
        validation.CollisionCheckingElapsedTime_s;
    stageTiming.FinalValidationElapsedTime_s = ...
        stageTiming.FinalValidationElapsedTime_s + max(0, elapsedTime_s - ...
        validation.CollisionCheckingElapsedTime_s);
end
candidate.Validation = validation;
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

function summary = summarizeCandidate(candidate, validation, diagnostics, ...
        elapsedTime_s, template, limits, options, initialTime_s)
% Copy solve and independent-validation evidence into the stable summary.
summary = template;
names = ["SeedIndex", "SeedSource", "OptimizerFeasible", ...
    "FinalTime_s", "MotionDuration_s", "MotionLength_deg", ...
    "IntegratedSquaredJerk_deg2_s5", "MaximumConstraintViolation", ...
    "TerminationReason"];
targets = ["SeedIndex", "SeedSource", "OptimizerFeasible", ...
    "ArrivalTime_s", "MotionDuration_s", "MotionLength_deg", ...
    "IntegratedSquaredJerk_deg2_s5", "MaximumConstraintViolation", ...
    "TerminationReason"];
for fieldIndex = 1:numel(names)
    summary.(targets(fieldIndex)) = candidate.(names(fieldIndex));
end

summary.ValidationPassed = validation.Passed;
summary.CollisionFree = validation.CollisionFree;
summary.CollisionResolved = validation.CollisionResolved;
summary.MinimumClearance_deg = validation.MinimumClearance_deg;
summary.CollisionIntervalCount = validation.CollisionIntervalCount;
summary.UnresolvedIntervalCount = validation.UnresolvedIntervalCount;
summary.SeedPlanningElapsedTime_s = elapsedTime_s;
summary.Message = strtrim(string(candidate.Message) + " " + validation.Message);
summary.SolverDiagnostics = diagnostics;
if validation.Passed
    normalizedPeaks = [validation.PeakVelocity_deg_s ./ ...
        limits.maxVelocity_deg_s, validation.PeakAcceleration_deg_s2 ./ ...
        limits.maxAcceleration_deg_s2, validation.PeakJerk_deg_s3 ./ ...
        limits.maxJerk_deg_s3];
    summary.KinematicUtilization = mean(normalizedPeaks);
    summary.TravelTimeTradeoffCost_deg = candidate.MotionLength_deg + ...
        options.MinimumTravelSavingsRate_deg_s * ...
        (candidate.FinalTime_s - initialTime_s);
end
if ~validation.Passed && ~isempty(candidate.time_s)
    summary.TerminationReason = "independentValidationFailed";
end
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
summary = summarizeCandidate(candidate, validation, diagnostics, ...
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

function kernelGoalState = createFixedKernelGoalState(goalState, options)
% Remove moving-goal evidence only after a fixed trial has frozen its endpoint.
kernelGoalState = goalState;
hasTargetHistory = isfield(goalState, "targetTime_s") && ...
    ~isempty(goalState.targetTime_s);
if ~hasTargetHistory || string(options.GoalTimeMode) ~= "fixedArrival"
    return;
end
targetPosition_deg = obstacleAvoidance.input.goalPositionAtTime( ...
    goalState, goalState.time_s);
coordinateScale_deg = bmtpEngine.createCoordinateTolerances( ...
    targetPosition_deg, goalState.position_deg);
if max(abs(targetPosition_deg - goalState.position_deg)) > ...
        256 * eps(coordinateScale_deg)
    return;
end
metadataFields = intersect(fieldnames(kernelGoalState), ...
    {'targetTime_s', 'targetPosition_deg', 'InterpolationMethod'});
kernelGoalState = rmfield(kernelGoalState, metadataFields);
end

function [candidate, diagnostics] = createTimedSeedCandidate( ...
        seed, initialState, goalState, limits, options, ...
        waitOverride_s, directMotionDuration_s)
% Realize a timed direct-wait seed without sending dynamics to static BMTP.
timer = tic;
    candidate = bmtpEngine.createMotionRecord( ...
    struct(), initialState, [], [], options.SampleTime_s, seed.Source);
candidate.SeedIndex = seed.Index;
diagnostics = struct("Accepted", false, "ElapsedTime_s", 0, ...
    "TerminationReason", "unsupportedTimedMultiWaypointRoute", ...
    "WaitTime_s", NaN, "InitialWaitTime_s", NaN, ...
    "FinalWaitTime_s", NaN, "RefinementCount", 0, ...
    "InfeasibleLowerWaitTime_s", NaN, ...
    "CoarseTrialCount", 0, "FeasibleWindowCount", 0, ...
    "EventWaitTime_s", zeros(0, 1), ...
    "TrialWaitTime_s", zeros(0, 1), "TrialPassed", false(0, 1), ...
    "WindowRecords", repmat(waitWindowTemplate(), 0, 1), ...
    "SelectedWindowIndex", 0, ...
    "HorizonProjectionKey", "candidateTimeRange_s", ...
    "EstimatedDuration_s", seed.EstimatedDuration_s, ...
    "SeedIndex", seed.Index, "SeedSource", string(seed.Source), ...
    "WaypointPosition_deg", seed.position_deg, "Tau", seed.tau, ...
    "ContainsWait", hasRepeatedWaypoint(seed.position_deg), ...
    "FirstUnsupportedTransitionIndex", 0, ...
    "FirstUnsupportedFeature", "", ...
    "OriginalTerminationReason", "", ...
    "FallbackPolicy", options.UnsupportedTimedTopologyPolicy, ...
    "FallbackAvailable", true, "FallbackAttempted", false, ...
    "FallbackMethod", "", "FallbackOutcome", "notApplicable", ...
    "AllInteriorWaypointsConstrainedToRest", false);
if string(seed.Source) ~= "directWait"
    diagnostics.FirstUnsupportedTransitionIndex = 1;
    diagnostics.FirstUnsupportedFeature = "multiWaypointTimedRoute";
    diagnostics.OriginalTerminationReason = diagnostics.TerminationReason;
    diagnostics.FallbackOutcome = "fallbackDisabledByPolicy";
    candidate.Message = ...
        "The compact dynamic kernel currently requires a direct-wait seed.";
    candidate.TerminationReason = diagnostics.TerminationReason;
    candidate.SolverDiagnostics = diagnostics;
    diagnostics.ElapsedTime_s = toc(timer);
    return;
end

coordinateScale_deg = bmtpEngine.createCoordinateTolerances( ...
    seed.position_deg, initialState.position_deg, goalState.position_deg);
positionTolerance_deg = 256 * eps(coordinateScale_deg);
isInitialPosition = vecnorm( ...
    seed.position_deg - initialState.position_deg, 2, 2) <= ...
    positionTolerance_deg;
firstMotionIndex = find(~isInitialPosition, 1, "first");
isDirectWait = ~isempty(firstMotionIndex) && firstMotionIndex > 1 && ...
    all(vecnorm(seed.position_deg(firstMotionIndex:end, :) - ...
    goalState.position_deg, 2, 2) <= positionTolerance_deg);
if ~isDirectWait
    diagnostics.TerminationReason = "invalidDirectWaitSeed";
    diagnostics.FirstUnsupportedTransitionIndex = firstMotionIndex;
    diagnostics.FirstUnsupportedFeature = "nonDirectMotionAfterWait";
    diagnostics.OriginalTerminationReason = diagnostics.TerminationReason;
    diagnostics.FallbackOutcome = "fallbackDisabledByPolicy";
    candidate.Message = "The timed seed is not a dwell followed by a direct edge.";
    candidate.TerminationReason = diagnostics.TerminationReason;
    candidate.SolverDiagnostics = diagnostics;
    diagnostics.ElapsedTime_s = toc(timer);
    return;
end

duration_s = double(seed.EstimatedDuration_s);
waitTime_s = duration_s * double(seed.tau(firstMotionIndex - 1));
if ~isempty(waitOverride_s)
    waitTime_s = waitOverride_s;
    duration_s = waitTime_s + directMotionDuration_s;
end
delayedInitialState = initialState;
delayedInitialState.time_s = initialState.time_s + waitTime_s;
delayedGoalState = goalState;
delayedGoalState.time_s = initialState.time_s + duration_s;
fixedOptions = options;
fixedOptions.GoalTimeMode = "fixedArrival";
direct = bmtpEngine.createDirectMotion( ...
    delayedInitialState, delayedGoalState, limits, fixedOptions);
if ~direct.Success
    candidate = direct;
    candidate.SeedIndex = seed.Index;
    candidate.SeedSource = string(seed.Source);
    diagnostics.TerminationReason = direct.TerminationReason;
    diagnostics.ElapsedTime_s = toc(timer);
    candidate.SolverDiagnostics = diagnostics;
    return;
end

directBreak_s = [direct.Polynomial.SegmentStartTime_s; ...
    direct.Polynomial.FinalTime_s] - delayedInitialState.time_s;
directJerk_deg_s3 = reshape(direct.Polynomial.jerkPower_deg_s3, ...
    direct.Polynomial.SegmentCount, numel(initialState.position_deg));
if waitTime_s > 0
    relativeBreak_s = [0; waitTime_s + directBreak_s];
    segmentJerk_deg_s3 = [zeros(1, numel(initialState.position_deg)); ...
        directJerk_deg_s3];
else
    relativeBreak_s = directBreak_s;
    segmentJerk_deg_s3 = directJerk_deg_s3;
end
candidate = bmtpEngine.createMotionRecord( ...
    direct, initialState, relativeBreak_s, segmentJerk_deg_s3, ...
    options.SampleTime_s, seed.Source);
candidate.SeedIndex = seed.Index;
candidate.Message = "An exact direct motion was realized after the timed dwell.";
[candidate.Success, candidate.OptimizerFeasible] = deal(true);
candidate.TerminationReason = "goalReached";
diagnostics.Accepted = true;
diagnostics.TerminationReason = candidate.TerminationReason;
diagnostics.WaitTime_s = waitTime_s;
diagnostics.ElapsedTime_s = toc(timer);
candidate.SolverDiagnostics = diagnostics;
end

function diagnostics = combineFallbackDiagnostics( ...
        timedDiagnostics, fallbackDiagnostics, originalReason, attempted)
% Preserve the earliest timed-kernel failure across an explicit recovery.
diagnostics = timedDiagnostics;
diagnostics.OriginalTerminationReason = originalReason;
diagnostics.FallbackAttempted = attempted;
diagnostics.FallbackMethod = "ruckigStopAtWaypoints";
if ~attempted
    diagnostics.FallbackOutcome = "fallbackDisabledByPolicy";
    return;
end
diagnostics.FallbackOutcome = string(fallbackDiagnostics.EngineTerminationReason);
diagnostics.FallbackDiagnostics = fallbackDiagnostics;
for fieldName = ["InteriorWaypointTime_s", ...
        "InteriorWaypointPosition_deg", ...
        "InteriorWaypointVelocity_deg_s", ...
        "InteriorWaypointAcceleration_deg_s2", ...
        "AllInteriorWaypointsConstrainedToRest"]
    if isfield(fallbackDiagnostics, fieldName)
        diagnostics.(fieldName) = fallbackDiagnostics.(fieldName);
    end
end
end

function hasWait = hasRepeatedWaypoint(position_deg)
% Treat a repeated consecutive guide point as an explicit spatial dwell.
if size(position_deg, 1) < 2
    hasWait = false;
    return;
end
coordinateScale_deg = bmtpEngine.createCoordinateTolerances(position_deg);
duplicateTolerance_deg = 256 * eps(coordinateScale_deg);
hasWait = any(vecnorm(diff(position_deg), 2, 2) <= ...
    duplicateTolerance_deg);
end

function [preparedObstacles, publicObstacles] = ...
        prepareRequestObstacles(obstacles)
% Reuse source-checked preparation without exposing internal caches in results.
preparedObstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);
publicObstacles = obstacles;
if isfield(publicObstacles, "InternalPreparation")
    publicObstacles = rmfield(publicObstacles, "InternalPreparation");
end
end

function record = createSweptProjectionRecord( ...
        diagnostics, validation, projection)
% Record conservative static mover geometry and authoritative validation.
record = struct( ...
    "Attempted", true, ...
    "Projection", projection, ...
    "SolverDiagnostics", diagnostics, ...
    "FullObstacleValidation", validation, ...
    "Outcome", "rejectedByFullValidation");
if validation.Passed
    record.Outcome = "acceptedAfterFullValidation";
end
end

function [candidate, validation, diagnostics, motionElapsedTime_s, ...
        stageTiming] = refineDirectWait(seed, candidate, validation, ...
        diagnostics, obstacles, initialState, goalState, limits, options, ...
        stageTiming)
% Bisect a measured infeasible/feasible dwell bracket through public validation.
initialWaitTime_s = diagnostics.WaitTime_s;
diagnostics.InitialWaitTime_s = initialWaitTime_s;
diagnostics.FinalWaitTime_s = initialWaitTime_s;
motionElapsedTime_s = 0;
if options.MaximumWaitRefinementIterations == 0 || initialWaitTime_s <= 0
    candidate.SolverDiagnostics = diagnostics;
    return;
end

directMotionDuration_s = candidate.MotionDuration_s - initialWaitTime_s;
eventWaitTime_s = collectDirectWaitEventTimes( ...
    obstacles, initialState.time_s, directMotionDuration_s, initialWaitTime_s);
coarseIntervalCount = 16;
trialWaitTime_s = obstacleAvoidance.planner.createEventAwareTrialTimes( ...
    0, initialWaitTime_s, eventWaitTime_s, coarseIntervalCount);
trialPassed = false(size(trialWaitTime_s));
trialCandidates = cell(size(trialWaitTime_s));
trialValidations = cell(size(trialWaitTime_s));
for trialIndex = 1:numel(trialWaitTime_s)
    if trialWaitTime_s(trialIndex) == initialWaitTime_s
        trialCandidate = candidate;
        trialValidation = validation;
    else
        motionTimer = tic;
        [trialCandidate, ~] = createTimedSeedCandidate( ...
            seed, initialState, goalState, limits, options, ...
            trialWaitTime_s(trialIndex), directMotionDuration_s);
        motionElapsedTime_s = motionElapsedTime_s + toc(motionTimer);
        [trialCandidate, trialValidation, ~, stageTiming] = ...
            validateCandidate( ...
            trialCandidate, obstacles, initialState, goalState, limits, ...
            options, stageTiming, ...
            "The refined direct-wait kernel returned no trajectory.");
    end
    trialPassed(trialIndex) = trialValidation.Passed;
    if trialPassed(trialIndex)
        trialCandidates{trialIndex} = trialCandidate;
        trialValidations{trialIndex} = trialValidation;
    end
end

windowStartIndex = find(trialPassed & [true; ~trialPassed(1:end - 1)]);
windowRecords = repmat(waitWindowTemplate(), numel(windowStartIndex), 1);
selectedWaitTime_s = NaN;
selectedWindowIndex = 0;
bestCandidate = candidate;
bestValidation = validation;
refinementCount = 0;
selectedLowerWaitTime_s = NaN;
for windowIndex = 1:numel(windowStartIndex)
    upperIndex = windowStartIndex(windowIndex);
    upperWaitTime_s = trialWaitTime_s(upperIndex);
    initialUpperWaitTime_s = upperWaitTime_s;
    bestWindowCandidate = trialCandidates{upperIndex};
    bestWindowValidation = trialValidations{upperIndex};
    windowRefinementCount = 0;
    if upperIndex == 1
        lowerWaitTime_s = NaN;
        coarseLowerWaitTime_s = NaN;
    else
        lowerWaitTime_s = trialWaitTime_s(upperIndex - 1);
        coarseLowerWaitTime_s = lowerWaitTime_s;
        while upperWaitTime_s - lowerWaitTime_s > ...
                options.ArrivalTimeTolerance_s && ...
                windowRefinementCount < ...
                options.MaximumWaitRefinementIterations
            queryWaitTime_s = 0.5 * ...
                (lowerWaitTime_s + upperWaitTime_s);
            motionTimer = tic;
            [trialCandidate, ~] = createTimedSeedCandidate( ...
                seed, initialState, goalState, limits, options, ...
                queryWaitTime_s, directMotionDuration_s);
            motionElapsedTime_s = motionElapsedTime_s + toc(motionTimer);
            [trialCandidate, trialValidation, ~, stageTiming] = ...
                validateCandidate( ...
                trialCandidate, obstacles, initialState, goalState, limits, ...
                options, stageTiming, ...
                "The refined direct-wait kernel returned no trajectory.");
            refinementCount = refinementCount + 1;
            windowRefinementCount = windowRefinementCount + 1;
            if trialValidation.Passed
                bestWindowCandidate = trialCandidate;
                bestWindowValidation = trialValidation;
                upperWaitTime_s = queryWaitTime_s;
            else
                lowerWaitTime_s = queryWaitTime_s;
            end
        end
    end
    windowRecords(windowIndex) = struct( ...
        "CoarseLowerWaitTime_s", coarseLowerWaitTime_s, ...
        "InitialUpperWaitTime_s", initialUpperWaitTime_s, ...
        "FinalLowerWaitTime_s", lowerWaitTime_s, ...
        "SelectedWaitTime_s", upperWaitTime_s, ...
        "RefinementCount", windowRefinementCount);
    if ~isfinite(selectedWaitTime_s) || upperWaitTime_s < selectedWaitTime_s
        selectedWaitTime_s = upperWaitTime_s;
        selectedLowerWaitTime_s = lowerWaitTime_s;
        selectedWindowIndex = windowIndex;
        bestCandidate = bestWindowCandidate;
        bestValidation = bestWindowValidation;
    end
end
candidate = bestCandidate;
validation = bestValidation;
diagnostics.RefinementCount = refinementCount;
diagnostics.CoarseTrialCount = numel(trialWaitTime_s);
diagnostics.FeasibleWindowCount = numel(windowRecords);
diagnostics.EventWaitTime_s = eventWaitTime_s;
diagnostics.TrialWaitTime_s = trialWaitTime_s;
diagnostics.TrialPassed = trialPassed;
diagnostics.WindowRecords = windowRecords;
diagnostics.SelectedWindowIndex = selectedWindowIndex;
diagnostics.InfeasibleLowerWaitTime_s = selectedLowerWaitTime_s;
diagnostics.WaitTime_s = selectedWaitTime_s;
diagnostics.FinalWaitTime_s = selectedWaitTime_s;
diagnostics.ElapsedTime_s = diagnostics.ElapsedTime_s + motionElapsedTime_s;
candidate.SolverDiagnostics = diagnostics;
end

function eventWaitTime_s = collectDirectWaitEventTimes( ...
        obstacles, initialTime_s, directMotionDuration_s, maximumWaitTime_s)
% Map source events to dwell times where motion starts or finishes at an event.
sourceEventTime_s = zeros(0, 1);
for obstacleIndex = 1:numel(obstacles)
    sourceEventTime_s = [sourceEventTime_s; ...
        double(obstacles(obstacleIndex).time_s(:))]; %#ok<AGROW>
end
eventWaitTime_s = [sourceEventTime_s - initialTime_s; ...
    sourceEventTime_s - initialTime_s - directMotionDuration_s];
eventWaitTime_s = unique(eventWaitTime_s( ...
    eventWaitTime_s >= 0 & eventWaitTime_s <= maximumWaitTime_s));
end

function record = waitWindowTemplate()
% Keep direct-wait feasible-window diagnostics stable across all candidates.
record = struct("CoarseLowerWaitTime_s", NaN, ...
    "InitialUpperWaitTime_s", NaN, "FinalLowerWaitTime_s", NaN, ...
    "SelectedWaitTime_s", NaN, "RefinementCount", 0);
end

function index = selectValidatedCandidate(summaries, indices, options)
% Rank valid motions by the declared objective, then utilization and travel.
length_deg = [summaries(indices).MotionLength_deg].';
utilization = [summaries(indices).KinematicUtilization].';
if options.GoalTimeMode == "fixedArrival"
    ranking = [length_deg, -utilization, indices];
elseif options.GoalTimeMode == "earliestArrival"
    ranking = [[summaries(indices).ArrivalTime_s].', ...
        -utilization, length_deg, indices];
else
    ranking = [[summaries(indices).TravelTimeTradeoffCost_deg].', ...
        [summaries(indices).ArrivalTime_s].', ...
        -utilization, length_deg, indices];
end
[~, order] = sortrows(ranking, 1:size(ranking, 2));
index = indices(order(1));
end

function isBetter = isBetterSummary(candidate, incumbent, options)
% Compare passing summaries using the public selection priority.
isBetter = selectValidatedCandidate([incumbent, candidate], [1; 2], ...
    options) == 2;
end

function index = bestPartialSeed(summaries)
% Prefer resolved collision evidence, small residual, and large clearance.
if isempty(summaries)
    index = 0;
    return;
end
violation = [summaries.MaximumConstraintViolation].';
violation(~isfinite(violation)) = Inf;
clearance_deg = [summaries.MinimumClearance_deg].';
clearance_deg(~isfinite(clearance_deg)) = -Inf;
collisionRank = 2 * ~[summaries.CollisionResolved].' + ...
    ~[summaries.CollisionFree].';
[~, order] = sortrows([collisionRank, violation, -clearance_deg, ...
    (1:numel(summaries)).']);
index = order(1);
end
