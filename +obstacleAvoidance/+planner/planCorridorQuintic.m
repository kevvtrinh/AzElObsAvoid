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
%   - Accept timed-opening or cavity motion early only when its request-wide
%     physical lower bound is within 0.1 microsecond of its validated upper.
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
[result, summaryTemplate] = obstacleAvoidance.planner.createEmptyResult( ...
    obstacles, initialState, goalState, limits, options, ...
    obstacleAvoidance.validateTrajectory());
preparedObstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);
useStaticKernel = obstacleAvoidance.obstacles.queryStaticHorizon( ...
    preparedObstacles, initialState.time_s, goalState.time_s);
stageTiming = result.SearchDiagnostics.StageTiming;
result.SearchDiagnostics.DirectAttempt = directAttemptTemplate();
[~, result.SearchDiagnostics.FixedClockExcursion] = ...
    obstacleAvoidance.planner.createFixedClockLateralExcursion();
[~, result.SearchDiagnostics.TimedOpening] = ...
    obstacleAvoidance.planner.createTimedOrthogonalOpeningMotion();
result.SearchDiagnostics.OrthogonalCavity = ...
    obstacleAvoidance.planner.evaluateArrivalCertificatePortfolio();
maximumInfimumGap_s = 1e-7;
firstValidatedMotionTime_s = NaN;
openingIsValidated = false;

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
        excursionSeed = createMotionSeed( ...
            excursionCandidate, "fixedClockLateralExcursion");
        result = finishFastPath(result, excursionCandidate, ...
            excursionCandidate.Validation, excursionDiagnostics, ...
            excursionElapsedTime_s, excursionSeed, summaryTemplate, ...
            "A fixed-clock lateral excursion attained the physical time floor.", ...
            planningTimer, stageTiming);
        return;
    end
end

%% Section 4: Try A Certified Timed Opening

if ~useRuckigWaypoint
    obstacleAvoidance.input.throwIfCancellationRequested(options);
    motionTimer = tic;
    [openingCandidate, openingDiagnostics] = ...
        obstacleAvoidance.planner.createTimedOrthogonalOpeningMotion( ...
        obstacles, initialState, goalState, limits, options);
    openingElapsedTime_s = toc(motionTimer);
    obstacleAvoidance.input.throwIfCancellationRequested(options);
    stageTiming = accountConstructorValidation( ...
        stageTiming, openingElapsedTime_s, openingDiagnostics);
    openingIsValidated = openingDiagnostics.Success && ...
        openingCandidate.Validation.Passed;
    if openingIsValidated
        firstValidatedMotionTime_s = toc(planningTimer);
    end
    if openingIsValidated && openingDiagnostics.AllRouteCertificatePassed
        if openingDiagnostics.InfimumGap_s < 0
            error("planCorridorQuintic:InconsistentOpeningBounds", ...
                "The opening upper is below its request-wide lower bound.");
        end
        openingDiagnostics.InfimumGapWithinPolicy = ...
            openingDiagnostics.InfimumGap_s <= maximumInfimumGap_s;
    end
    result.SearchDiagnostics.TimedOpening = openingDiagnostics;
    if openingIsValidated && openingDiagnostics.InfimumGapWithinPolicy
        openingSeed = createDirectSeed(initialState, goalState, ...
            openingCandidate.MotionDuration_s, openingDiagnostics.WaitTime_s);
        openingSeed.Source = "timedOrthogonalOpening";
        result = finishFastPath(result, openingCandidate, ...
            openingCandidate.Validation, openingDiagnostics, ...
            openingElapsedTime_s, openingSeed, summaryTemplate, ...
            "A timed opening motion met the physical infimum-gap policy.", ...
            planningTimer, stageTiming);
        return;
    end
end

%% Section 5: Exhaust Deterministic Topology Seeds

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
cavityAttempts = cell(seedCount, 1);
cavityCandidates = cell(seedCount, 1);
cavityLowerBounds_s = NaN(seedCount, 1);
cavityUpperDurations_s = NaN(seedCount, 1);
cavityMotionLengths_deg = NaN(seedCount, 1);
if ~useRuckigWaypoint
    for seedIndex = 1:seedCount
        obstacleAvoidance.input.throwIfCancellationRequested(options);
        motionTimer = tic;
        [candidate, cavityDiagnostics] = ...
            obstacleAvoidance.planner.createOrthogonalCavityMotion( ...
            seeds(seedIndex), preparedObstacles, initialState, goalState, ...
            limits, options);
        elapsedTime_s = toc(motionTimer);
        stageTiming.MotionSolvingElapsedTime_s = ...
            stageTiming.MotionSolvingElapsedTime_s + elapsedTime_s;
        [candidate, validation, ~, stageTiming] = validateCandidate( ...
            candidate, preparedObstacles, initialState, goalState, limits, ...
            options, stageTiming, "");
        obstacleAvoidance.input.throwIfCancellationRequested(options);
        candidate.SeedIndex = seedIndex;
        lowerCertificate = struct("Passed", false, ...
            "TerminationReason", "candidateNotConstructed", ...
            "LowerBound_s", NaN, "UpperGap_s", NaN);
        if ~isempty(candidate.time_s)
            lowerCertificate = ...
                obstacleAvoidance.planner.certifyOrthogonalCavityLowerBound( ...
                cavityDiagnostics, candidate, preparedObstacles, ...
                initialState, goalState, limits, options);
        end
        attempt = struct("Diagnostics", cavityDiagnostics, ...
            "AllRouteCertificate", lowerCertificate, ...
            "Validation", validation, "ElapsedTime_s", elapsedTime_s);
        cavityAttempts{seedIndex} = attempt;
        if validation.Passed
            candidate = acceptValidatedCandidate( ...
                candidate, initialState.time_s, ...
                "A cavity motion passed independent public validation.");
            cavityCandidates{seedIndex} = candidate;
            cavityUpperDurations_s(seedIndex) = candidate.MotionDuration_s;
            cavityMotionLengths_deg(seedIndex) = candidate.MotionLength_deg;
            if lowerCertificate.Passed
                cavityLowerBounds_s(seedIndex) = lowerCertificate.LowerBound_s;
            end
            candidates{seedIndex} = candidate;
            seedSummaries(seedIndex) = summarizeCandidate( ...
                candidate, validation, attempt, elapsedTime_s, ...
                summaryTemplate);
            if isnan(firstValidatedMotionTime_s)
                firstValidatedMotionTime_s = toc(planningTimer);
            end
        end
    end
    cavityPortfolio = ...
        obstacleAvoidance.planner.evaluateArrivalCertificatePortfolio( ...
        cavityUpperDurations_s, cavityMotionLengths_deg, ...
        cavityLowerBounds_s, maximumInfimumGap_s);
    cavityPortfolio.Attempts = cavityAttempts;
    result.SearchDiagnostics.OrthogonalCavity = cavityPortfolio;
    if cavityPortfolio.InfimumGapWithinPolicy
        selectedIndex = cavityPortfolio.SelectedSeedIndex;
        selectedCandidate = cavityCandidates{selectedIndex};
        selectedCandidate.SeedIndex = 1;
        selectedSeed = seeds(selectedIndex);
        selectedSeed.Index = 1;
        result = finishFastPath(result, selectedCandidate, ...
            selectedCandidate.Validation, cavityAttempts{selectedIndex}, ...
            cavityAttempts{selectedIndex}.ElapsedTime_s, selectedSeed, ...
            summaryTemplate, ...
            "A cavity motion met the fixed-goal physical infimum-gap policy.", ...
            planningTimer, stageTiming);
        return;
    end
end

for seedIndex = 1:seedCount
    obstacleAvoidance.input.throwIfCancellationRequested(options);
    seedOptions = applyIncumbentWorkBudget(options, seedSummaries);
    motionTimer = tic;
    candidateWasPrevalidated = false;
    prevalidationElapsedTime_s = 0;
    validation = obstacleAvoidance.validateTrajectory();
    if useRuckigWaypoint
        [candidate, solverDiagnostics] = ...
            obstacleAvoidance.planner.createRuckigWaypointMotion( ...
            seeds(seedIndex), initialState, goalState, limits, seedOptions);
    elseif useStaticKernel
        kernelGoalState = createFixedKernelGoalState(goalState, options);
        [candidate, solverDiagnostics] = ...
            obstacleAvoidance.planner.solveBmtpTrajectory( ...
            seeds(seedIndex), preparedObstacles, initialState, kernelGoalState, ...
            limits, seedOptions);
    else
        [staticObstacles, staticObstacleIndices, movingObstacleIndices] = ...
            partitionStaticObstacles( ...
            preparedObstacles, initialState.time_s, goalState.time_s);
        tryStaticProjection = string(seeds(seedIndex).Source) ~= ...
            "directWait" && size(seeds(seedIndex).position_deg, 1) > 2;
        staticAttempt = struct();
        sweptAttempt = struct();
        timedBmtpAttempt = struct();
        if tryStaticProjection
            kernelGoalState = createFixedKernelGoalState(goalState, options);
            [staticCandidate, staticDiagnostics] = ...
                obstacleAvoidance.planner.solveBmtpTrajectory( ...
                seeds(seedIndex), staticObstacles, initialState, ...
                kernelGoalState, limits, seedOptions);
            [staticCandidate, staticValidation, staticValidationTime_s, ...
                stageTiming] = validateCandidate( ...
                staticCandidate, preparedObstacles, initialState, ...
                goalState, limits, options, stageTiming, ...
                "The static-projection BMTP kernel returned no trajectory.");
            prevalidationElapsedTime_s = staticValidationTime_s;
            staticAttempt = createStaticProjectionRecord( ...
                staticDiagnostics, staticValidation, ...
                staticObstacleIndices, movingObstacleIndices);
            if staticValidation.Passed
                candidate = staticCandidate;
                validation = staticValidation;
                solverDiagnostics = staticDiagnostics;
                solverDiagnostics.StaticProjection = staticAttempt;
                solverDiagnostics.MovingObstacleRelevance = ...
                    "provenAbsentForReturnedTrajectory";
                candidate.SolverDiagnostics = solverDiagnostics;
                candidateWasPrevalidated = true;
            end
        end
        if tryStaticProjection && ~candidateWasPrevalidated
            [planningObstacles, projection] = ...
                obstacleAvoidance.obstacles.createStaticPlanningProjection( ...
                preparedObstacles, initialState.time_s, goalState.time_s);
            [sweptCandidate, sweptDiagnostics] = ...
                obstacleAvoidance.planner.solveBmtpTrajectory( ...
                seeds(seedIndex), planningObstacles, initialState, ...
                kernelGoalState, limits, seedOptions);
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
                solverDiagnostics.StaticProjection = staticAttempt;
                solverDiagnostics.SweptProjection = sweptAttempt;
                solverDiagnostics.DynamicObstacleRepresentation = ...
                    "conservativeStaticProtectedHistoryConvexHull";
                candidate.SolverDiagnostics = solverDiagnostics;
                candidateWasPrevalidated = true;
            end
        end
        tryTimedBmtp = tryStaticProjection && ~candidateWasPrevalidated && ...
            string(seeds(seedIndex).Source) == "timeExpandedVisibilityGraph";
        if tryTimedBmtp
            [timedCandidate, timedBmtpDiagnostics] = ...
                obstacleAvoidance.planner.solveTimedBmtpTrajectory( ...
                seeds(seedIndex), preparedObstacles, initialState, ...
                goalState, limits, seedOptions);
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
                solverDiagnostics.StaticProjection = staticAttempt;
                solverDiagnostics.SweptProjection = sweptAttempt;
                solverDiagnostics.TimedBmtp = timedBmtpAttempt;
                candidate.SolverDiagnostics = solverDiagnostics;
                candidateWasPrevalidated = true;
            end
        end
        if ~candidateWasPrevalidated
            [candidate, solverDiagnostics] = createTimedSeedCandidate( ...
                seeds(seedIndex), initialState, goalState, limits, ...
                seedOptions, [], []);
            if tryStaticProjection
                solverDiagnostics.StaticProjection = staticAttempt;
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
                    seedOptions);
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
    budgetExhausted = solverWorkBudgetExhausted(solverDiagnostics);
    if budgetExhausted
        validation = obstacleAvoidance.validateTrajectory();
        candidate.TerminationReason = "seedWorkBudgetExhausted";
        candidate.Message = "The seed motion solve exhausted its incumbent work budget.";
    elseif ~candidateWasPrevalidated
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
        solverDiagnostics, elapsedTime_s, summaryTemplate);
    if budgetExhausted
        summary.TerminationReason = "seedWorkBudgetExhausted";
        summary.Message = candidate.Message;
    end
    if ~seedSummaries(seedIndex).ValidationPassed || ...
            (validation.Passed && isBetterSummary( ...
            summary, seedSummaries(seedIndex), options.GoalTimeMode))
        candidates{seedIndex} = candidate;
        seedSummaries(seedIndex) = summary;
    end
    if validation.Passed && isnan(firstValidatedMotionTime_s)
        firstValidatedMotionTime_s = toc(planningTimer);
    end
end

% A route-class upper without a request-wide lower must not suppress BMTP.
if openingIsValidated
    openingCandidate.SeedIndex = seedCount + 1;
    openingSeed = createDirectSeed(initialState, goalState, ...
        openingCandidate.MotionDuration_s, openingDiagnostics.WaitTime_s);
    openingSeed.Index = openingCandidate.SeedIndex;
    openingSeed.Source = "timedOrthogonalOpening";
    seeds(end + 1) = openingSeed;
    candidates{end + 1, 1} = openingCandidate;
    seedSummaries(end + 1, 1) = summarizeCandidate(openingCandidate, ...
        openingCandidate.Validation, openingDiagnostics, ...
        openingElapsedTime_s, summaryTemplate);
end
result.Seeds = seeds;

%% Section 6: Select A Valid Motion Or Return Evidence

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
    seedSummaries, validatedIndices, options.GoalTimeMode);
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

%% Section 7: Local Functions

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

function candidate = acceptValidatedCandidate(candidate, initialTime_s, message)
% Normalize authoritative final-time fields after validation passes.
candidate.FinalTime_s = double(candidate.Polynomial.FinalTime_s);
candidate.ArrivalTime_s = candidate.FinalTime_s;
candidate.MotionDuration_s = candidate.FinalTime_s - initialTime_s;
candidate.Success = true;
candidate.TerminationReason = "goalReached";
candidate.Message = message;
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
        elapsedTime_s, template)
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
summary.UnresolvedIntervalCount = validation.UnresolvedIntervalCount;
summary.SeedPlanningElapsedTime_s = elapsedTime_s;
summary.Message = strtrim(string(candidate.Message) + " " + validation.Message);
summary.SolverDiagnostics = diagnostics;
if ~validation.Passed && ~isempty(candidate.time_s)
    summary.TerminationReason = "independentValidationFailed";
end
end

function options = applyIncumbentWorkBudget(options, seedSummaries)
% Limit only later work after public validation establishes an incumbent.
successful = [seedSummaries.ValidationPassed];
if ~any(successful)
    return;
end
successfulElapsed_s = [seedSummaries(successful).SeedPlanningElapsedTime_s];
fastestSuccessfulSolve_s = min(successfulElapsed_s);
if ~isfinite(fastestSuccessfulSolve_s) || fastestSuccessfulSolve_s <= 0
    return;
end
options.MaximumSolverTime_s = max(1, ...
    options.PerSeedWorkBudgetMultiplier * fastestSuccessfulSolve_s);
end

function exhausted = solverWorkBudgetExhausted(diagnostics)
% Translate the engine work-limit flag into the planner's stable seed reason.
exhausted = isstruct(diagnostics) && isscalar(diagnostics) && ...
    isfield(diagnostics, "WorkLimitReached") && ...
    isequal(diagnostics.WorkLimitReached, true);
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
    elapsedTime_s, summaryTemplate);
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

function [staticObstacles, staticIndices, movingIndices] = ...
        partitionStaticObstacles(obstacles, startTime_s, endTime_s)
% Partition by complete-horizon invariance without changing obstacle records.
isStatic = false(numel(obstacles), 1);
for obstacleIndex = 1:numel(obstacles)
    isStatic(obstacleIndex) = ...
        obstacleAvoidance.obstacles.queryStaticHorizon( ...
        obstacles(obstacleIndex), startTime_s, endTime_s);
end
staticIndices = find(isStatic);
movingIndices = find(~isStatic);
staticObstacles = obstacles(isStatic);
end

function record = createStaticProjectionRecord( ...
        diagnostics, validation, staticIndices, movingIndices)
% Record an optional BMTP solve whose full validation proves mover irrelevance.
record = struct( ...
    "Attempted", true, ...
    "StaticObstacleIndices", staticIndices, ...
    "MovingObstacleIndices", movingIndices, ...
    "SolverDiagnostics", diagnostics, ...
    "FullObstacleValidation", validation, ...
    "MovingObstacleInteractionProvenAbsent", validation.Passed, ...
    "Outcome", "rejectedByFullValidation");
if validation.Passed
    record.Outcome = "acceptedAfterFullValidation";
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
lowerWaitTime_s = 0;
upperWaitTime_s = initialWaitTime_s;
bestCandidate = candidate;
bestValidation = validation;
for refinementIndex = 1:options.MaximumWaitRefinementIterations
    if refinementIndex == 1
        trialWaitTime_s = lowerWaitTime_s;
    else
        trialWaitTime_s = 0.5 * (lowerWaitTime_s + upperWaitTime_s);
    end
    motionTimer = tic;
    [trialCandidate, ~] = createTimedSeedCandidate( ...
        seed, initialState, goalState, limits, options, ...
        trialWaitTime_s, directMotionDuration_s);
    motionElapsedTime_s = motionElapsedTime_s + toc(motionTimer);
    [trialCandidate, trialValidation, ~, stageTiming] = validateCandidate( ...
        trialCandidate, obstacles, initialState, goalState, limits, options, ...
        stageTiming, "The refined direct-wait kernel returned no trajectory.");
    diagnostics.RefinementCount = refinementIndex;
    if trialValidation.Passed
        bestCandidate = trialCandidate;
        bestValidation = trialValidation;
        upperWaitTime_s = trialWaitTime_s;
        if trialWaitTime_s == 0
            break;
        end
    else
        lowerWaitTime_s = trialWaitTime_s;
        diagnostics.InfeasibleLowerWaitTime_s = lowerWaitTime_s;
    end
end
candidate = bestCandidate;
validation = bestValidation;
diagnostics.WaitTime_s = upperWaitTime_s;
diagnostics.FinalWaitTime_s = upperWaitTime_s;
diagnostics.ElapsedTime_s = diagnostics.ElapsedTime_s + motionElapsedTime_s;
candidate.SolverDiagnostics = diagnostics;
end

function index = selectValidatedCandidate(summaries, indices, goalTimeMode)
% Rank fixed arrivals by length then jerk, and free arrivals by time first.
ranking = [[summaries(indices).MotionLength_deg].', ...
    [summaries(indices).IntegratedSquaredJerk_deg2_s5].', indices];
if goalTimeMode ~= "fixedArrival"
    ranking = [[summaries(indices).ArrivalTime_s].', ranking];
end
[~, order] = sortrows(ranking, 1:size(ranking, 2));
index = indices(order(1));
end

function isBetter = isBetterSummary(candidate, incumbent, goalTimeMode)
% Compare passing summaries using the public selection priority.
isBetter = selectValidatedCandidate([incumbent, candidate], [1; 2], ...
    goalTimeMode) == 2;
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
