function result = planAzElAvoidance(request)
%% Section 0: Header & Readme
% SYNTAX
%   result = planAzElAvoidance(request)
%**************************************************************************
% PURPOSE
%   - Find the earliest independently validated azimuth/elevation command
%     established within one deterministic adaptive planning budget.
%   - Return honest structured failure when success or infeasibility is not
%     established.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       .obstacles
%           Empty, canonical azElData struct array, or cell collection.
%       .initialState
%           time_s scalar plus 1-by-2 position_deg, velocity_deg_s, and
%           acceleration_deg_s2.
%       .goal
%           Fixed complete state (type="fixed", scalar time_s may be NaN)
%           or moving complete-state histories (type="moving", N samples).
%       .limits
%           azimuth_deg, elevation_deg, maxVelocity_deg_s, and
%           maxAcceleration_deg_s2 two-component rows.
%       .options (optional partial struct)
%           safetyMargin_deg=0.5, azimuthWrap=false,
%           temporalPadding_s=0, deadline_s=derived,
%           trailingDuration_s=0, planningWallTime_s=20, and independent
%           validation tolerances. No internal search controls are public.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable success/failure schema with command, arrival, waits,
%       validation, guarantee, timing, and adaptive diagnostics.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

%% Section 1: Validate Inputs & Apply Defaults
planningTimer = tic;
result = makeAzElPlannerResult();
try
    request = normalizeAzElPlannerRequest(request);
catch exception
    result.status = "invalid";
    result.reasonCode = string(exception.identifier);
    result.message = string(exception.message);
    result.planningElapsed_s = toc(planningTimer);
    result.diagnostics.stoppingReason = "InvalidInput";
    return;
end
publicOptions = rmfield(request.options, "azimuthDisplayRange_deg");
result.options = publicOptions;
result.limits = request.limits;

%% Section 2: Establish Independent Bounds & Endpoint Facts
schedule = buildAdaptiveSchedule(request);
earliestGoalQuery_s = request.initialState.time_s;
if request.goal.type == "moving"
    earliestGoalQuery_s = max(earliestGoalQuery_s, ...
        request.goal.time_s(1));
end
earliestGoalState = evaluateAzElGoal(request.goal, earliestGoalQuery_s);
lowerBound = estimateAzElLowerBound( ...
    request.initialState, earliestGoalState, request.limits);
result.guarantee.lowerBoundDuration_s = lowerBound.duration_s;
result.guarantee.model = lowerBound.model;

[initialClearance_deg, initialNearest] = azElObstacleClearance( ...
    request.obstacles, request.initialState.position_deg, ...
    request.initialState.time_s, request.options);
if initialClearance_deg <= request.options.safetyMargin_deg + ...
        request.options.clearanceTolerance_deg
    result.status = "infeasible";
    result.reasonCode = "BlockedInitialState";
    result.message = "The complete initial state is inside obstacle clearance.";
    if strlength(initialNearest.targetName) > 0
        result.message = result.message + " Blocking obstacle: " + ...
            initialNearest.targetName + ".";
    end
    result.planningElapsed_s = toc(planningTimer);
    result.diagnostics.stoppingReason = "ProvedBlockedInitialState";
    return;
end

if request.goal.type == "fixed" && ~schedule.hasTimeVariation
    fixedGoalState = evaluateAzElGoal(request.goal, ...
        request.initialState.time_s);
    [goalClearance_deg, goalNearest] = azElObstacleClearance( ...
        request.obstacles, fixedGoalState.position_deg, ...
        request.initialState.time_s, request.options);
    if goalClearance_deg <= request.options.safetyMargin_deg + ...
            request.options.clearanceTolerance_deg
        result.status = "infeasible";
        result.reasonCode = "BlockedTerminalState";
        result.message = ["The fixed terminal state remains inside static " ...
            "obstacle clearance."];
        if strlength(goalNearest.targetName) > 0
            result.message = result.message + " Blocking obstacle: " + ...
                goalNearest.targetName + ".";
        end
        result.planningElapsed_s = toc(planningTimer);
        result.diagnostics.stoppingReason = "ProvedBlockedTerminalState";
        return;
    end
end

if request.goal.type == "fixed" && request.initialState.time_s + ...
        lowerBound.duration_s > request.options.deadline_s + ...
        request.options.arrivalTolerance_s
    result.status = "infeasible";
    result.reasonCode = "ObstacleFreeBoundaryInfeasible";
    result.message = ["Even the independent obstacle-free lower bound " ...
        "exceeds the mission deadline."];
    result.planningElapsed_s = toc(planningTimer);
    result.diagnostics.stoppingReason = "ProvedDeadlineInfeasible";
    return;
end

%% Section 3: Search Adaptive Coarse-To-Fine Candidates
bestArrivalTime_s = Inf;
bestCommand = struct();
bestWaits = struct([]);
bestValidation = struct();
bestWasDirect = false;
levelTemplate = struct( ...
    "level", 0, ...
    "spatialResolution_deg", NaN, ...
    "temporalResolution_s", NaN, ...
    "nodeCount", 0, ...
    "edgeCount", 0, ...
    "routeCount", 0, ...
    "candidateCount", 0, ...
    "validationFailureCount", 0, ...
    "incumbentArrivalTime_s", NaN, ...
    "planningElapsed_s", 0);
result.diagnostics.levels = repmat(levelTemplate, ...
    numel(schedule.spatialResolution_deg), 1);
result.diagnostics.adaptiveInputs = struct( ...
    "goalSeparation_deg", schedule.goalSeparation_deg, ...
    "minimumObstacleFeature_deg", schedule.minimumFeature_deg, ...
    "obstacleCadence_s", schedule.obstacleCadence_s, ...
    "hasTimeVariation", schedule.hasTimeVariation);

completedLevelCount = 0;
for levelIndex = 1:numel(schedule.spatialResolution_deg)
    if toc(planningTimer) >= request.options.planningWallTime_s
        result.diagnostics.stoppingReason = "PlanningBudgetExhausted";
        break;
    end
    levelStart_s = toc(planningTimer);
    spatialResolution_deg = ...
        schedule.spatialResolution_deg(levelIndex);
    temporalResolution_s = ...
        schedule.temporalResolution_s(levelIndex);
    if isfinite(bestArrivalTime_s)
        nominalArrivalTime_s = bestArrivalTime_s;
    else
        nominalArrivalTime_s = max( ...
            request.initialState.time_s + lowerBound.duration_s, ...
            schedule.referenceTime_s);
        nominalArrivalTime_s = min(nominalArrivalTime_s, ...
            request.options.deadline_s);
    end
    if request.goal.type == "moving"
        nominalArrivalTime_s = min(max(nominalArrivalTime_s, ...
            request.goal.time_s(1)), request.goal.time_s(end));
    end

    [routes, routeStatistics] = buildAdaptiveRoutes(request, ...
        spatialResolution_deg, nominalArrivalTime_s);
    search = searchAzElRoutes(routes, request, ...
        temporalResolution_s, bestArrivalTime_s, planningTimer);

    completedLevelCount = completedLevelCount + 1;
    levelRecord = struct( ...
        "level", levelIndex, ...
        "spatialResolution_deg", spatialResolution_deg, ...
        "temporalResolution_s", temporalResolution_s, ...
        "nodeCount", routeStatistics.nodeCount, ...
        "edgeCount", routeStatistics.edgeCount, ...
        "routeCount", routeStatistics.routeCount, ...
        "candidateCount", search.candidateCount, ...
        "validationFailureCount", search.validationFailureCount, ...
        "incumbentArrivalTime_s", bestArrivalTime_s, ...
        "planningElapsed_s", toc(planningTimer) - levelStart_s);

    if search.success && search.arrivalTime_s < bestArrivalTime_s
        bestArrivalTime_s = search.arrivalTime_s;
        bestCommand = search.command;
        bestWaits = search.waits;
        bestValidation = search.validation;
        bestWasDirect = search.routeIndex == 1;
        result.diagnostics.incumbentChanges(end + 1, :) = ...
            [levelIndex, bestArrivalTime_s];
        levelRecord.incumbentArrivalTime_s = bestArrivalTime_s;
    end
    result.diagnostics.levels(levelIndex) = levelRecord;
    result.diagnostics.validationFailures = [ ...
        result.diagnostics.validationFailures; search.failureMessages];

    if search.budgetExpired
        result.diagnostics.stoppingReason = "PlanningBudgetExhausted";
        break;
    end
    if isempty(request.obstacles) && request.goal.type == "fixed" && ...
            isfinite(bestArrivalTime_s)
        result.diagnostics.stoppingReason = ...
            "ObstacleFreeSCurveMinimumEstablished";
        break;
    end
    if levelIndex == numel(schedule.spatialResolution_deg)
        result.diagnostics.stoppingReason = "AdaptiveScheduleExhausted";
    end
end
result.diagnostics.levels = ...
    result.diagnostics.levels(1:completedLevelCount);
result.diagnostics.validationFailures = unique( ...
    result.diagnostics.validationFailures, "stable");

%% Section 4: Assemble The Best Independently Validated Result
result.planningElapsed_s = toc(planningTimer);
if isfinite(bestArrivalTime_s)
    result.success = true;
    result.status = "success";
    result.reasonCode = "ValidatedCommandFound";
    result.message = "Returned the earliest independently validated command found.";
    result.command = bestCommand;
    result.arrivalTime_s = bestArrivalTime_s;
    result.waits = bestWaits;
    result.validation = bestValidation;
    result.executionDuration_s = bestArrivalTime_s - ...
        request.initialState.time_s;
    result.operationalArrivalDelay_s = NaN;
    result.guarantee.feasibility = "Validated feasible";
    result.guarantee.lowerBoundDuration_s = lowerBound.duration_s;
    result.guarantee.arrivalGap_s = result.executionDuration_s - ...
        lowerBound.duration_s;
    if isempty(request.obstacles) && bestWasDirect
        result.guarantee.optimality = ...
            "Minimum arrival under a stated model";
        result.guarantee.model = ["C2 constant-jerk S-curve direct motion " ...
            "with exact endpoint position, velocity, and acceleration"];
        if isempty(schedule.temporalResolution_s)
            result.guarantee.tolerance_s = NaN;
        else
            result.guarantee.tolerance_s = max(1e-6, ...
                0.02 .* schedule.temporalResolution_s(1));
        end
    else
        result.guarantee.optimality = "Best found";
        result.guarantee.model = ["Deterministic adaptive boundary-route " ...
            "and quintic timing search"];
        if completedLevelCount > 0
            result.guarantee.tolerance_s = ...
                schedule.temporalResolution_s(completedLevelCount);
        end
    end
else
    result.status = "unknown";
    if result.diagnostics.stoppingReason == "PlanningBudgetExhausted"
        result.reasonCode = "PlanningBudgetExhausted";
        result.message = ["The planning budget ended without a validated " ...
            "command or an infeasibility proof."];
    else
        result.reasonCode = "SearchInconclusive";
        result.message = ["Adaptive refinement ended without a validated " ...
            "command or an infeasibility proof."];
    end
    result.guarantee.feasibility = "Unvalidated";
    result.guarantee.optimality = "Unknown";
end
end
