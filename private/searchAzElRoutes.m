function search = searchAzElRoutes(routes, request, ...
        temporalResolution_s, incumbentArrivalTime_s, planningTimer)
%% Section 0: Header & Readme
% SYNTAX
%   search = searchAzElRoutes(routes, request, temporalResolution_s, ...
%       incumbentArrivalTime_s, planningTimer)
%**************************************************************************
% PURPOSE
%   - Search arrival, motion-duration, and physically feasible wait choices
%     for internally generated routes while respecting one global budget.
%**************************************************************************
% INPUTS
%   - routes (cell column)
%       Planner-generated spatial route seeds.
%   - request (normalized scalar planning request)
%   - temporalResolution_s (positive scalar private resolution)
%   - incumbentArrivalTime_s (scalar, Inf when absent)
%   - planningTimer (tic handle)
%**************************************************************************
% OUTPUTS
%   - search (scalar struct)
%       Best command, validation, waits, counts, and failure summaries.
%**************************************************************************
% UNITS
%   - Time fields are seconds.

%% Section 1: Initialize Search Bounds
search = searchTemplate();
startTime_s = request.initialState.time_s;
deadline_s = request.options.deadline_s;
latestUsefulArrival_s = min(deadline_s, incumbentArrivalTime_s - ...
    0.25 .* temporalResolution_s);
if ~isfinite(latestUsefulArrival_s)
    latestUsefulArrival_s = deadline_s;
end
if latestUsefulArrival_s <= startTime_s
    return;
end

startCanWait = norm(request.initialState.velocity_deg_s) <= ...
    request.options.velocityTolerance_deg_s && ...
    norm(request.initialState.acceleration_deg_s2) <= ...
    request.options.accelerationTolerance_deg_s2;
hasTimeVariation = sceneHasTimeVariation(request);

%% Section 2: Search Routes In Deterministic Order
for routeIndex = 1:numel(routes)
    if budgetExpired(request, planningTimer)
        search.budgetExpired = true;
        break;
    end
    routePosition_deg = routes{routeIndex};
    earliestArrival_s = estimateEarliestArrival( ...
        routePosition_deg, request, temporalResolution_s);
    if earliestArrival_s > latestUsefulArrival_s
        continue;
    end
    candidateArrival_s = buildArrivalSequence(earliestArrival_s, ...
        latestUsefulArrival_s, temporalResolution_s);

    routeFound = false;
    for arrivalIndex = 1:numel(candidateArrival_s)
        arrivalTime_s = candidateArrival_s(arrivalIndex);
        goalState = evaluateAzElGoal(request.goal, arrivalTime_s);
        candidateRoute_deg = routePosition_deg;
        candidateRoute_deg(end, :) = goalState.position_deg;
        waitDuration_s = buildWaitSequence(arrivalTime_s, ...
            candidateRoute_deg, goalState, request, ...
            temporalResolution_s, startCanWait, hasTimeVariation);

        for waitIndex = 1:numel(waitDuration_s)
            if budgetExpired(request, planningTimer)
                search.budgetExpired = true;
                break;
            end
            try
                [candidateCommand, candidateWaits] = buildAzElCommand( ...
                    candidateRoute_deg, request.initialState, goalState, ...
                    request.limits, request.options, arrivalTime_s, ...
                    waitDuration_s(waitIndex));
            catch exception
                search.failureMessages(end + 1, 1) = ...
                    string(exception.identifier);
                continue;
            end
            if request.options.trailingDuration_s > 0
                candidateCommand = appendAzElTrailing(candidateCommand, ...
                    request.goal, request.options, request.limits, ...
                    arrivalTime_s);
            end

            search.candidateCount = search.candidateCount + 1;
            motion = validateAzElMotion(candidateCommand, ...
                request.limits, request.options);
            if ~motion.isValid
                search.validationFailureCount = ...
                    search.validationFailureCount + 1;
                search.failureMessages(end + 1, 1) = ...
                    "MotionLimitViolation";
                continue;
            end
            validation = validateAzElCommand( ...
                candidateCommand, request, arrivalTime_s);
            if validation.isValid
                if arrivalTime_s < search.arrivalTime_s
                    search.success = true;
                    search.command = candidateCommand;
                    search.waits = candidateWaits;
                    search.validation = validation;
                    search.arrivalTime_s = arrivalTime_s;
                    search.routeIndex = routeIndex;
                    search.routePosition_deg = candidateRoute_deg;
                end
                routeFound = true;
                break;
            end
            search.validationFailureCount = ...
                search.validationFailureCount + 1;
            search.failureMessages(end + 1, 1) = ...
                summarizeValidationFailure(validation);
        end
        if routeFound || search.budgetExpired
            break;
        end
    end
    if search.budgetExpired
        break;
    end
end
end

function earliestArrival_s = estimateEarliestArrival(routePosition_deg, ...
        request, temporalResolution_s)
%% Section 0: Header & Readme
% SYNTAX
%   earliestArrival_s = estimateEarliestArrival(routePosition_deg, ...
%       request, temporalResolution_s)
%**************************************************************************
% PURPOSE
%   - Find the earliest motion-feasible arrival bracket for one route.
%**************************************************************************
% INPUTS
%   - routePosition_deg (N-by-2 numeric)
%   - request (normalized scalar request)
%   - temporalResolution_s (positive scalar)
%**************************************************************************
% OUTPUTS
%   - earliestArrival_s (finite scalar or Inf)
%**************************************************************************
% UNITS
%   - Time is seconds.

startTime_s = request.initialState.time_s;
if request.goal.type == "moving"
    earliestArrival_s = max(startTime_s + temporalResolution_s, ...
        request.goal.time_s(1));
    for iterationIndex = 1:5
        if earliestArrival_s > request.options.deadline_s
            earliestArrival_s = Inf;
            return;
        end
        goalState = evaluateAzElGoal(request.goal, earliestArrival_s);
        routePosition_deg(end, :) = goalState.position_deg;
        routeBound_s = routeDurationLowerBound( ...
            routePosition_deg, request, goalState);
        updatedArrival_s = max(request.goal.time_s(1), ...
            startTime_s + routeBound_s);
        if updatedArrival_s <= earliestArrival_s + ...
                0.25 * temporalResolution_s
            break;
        end
        earliestArrival_s = updatedArrival_s;
    end
    return;
end

goalState = evaluateAzElGoal(request.goal, startTime_s);
maximumDuration_s = request.options.deadline_s - startTime_s;
lowerDuration_s = routeDurationLowerBound( ...
    routePosition_deg, request, goalState);
lowerDuration_s = max(lowerDuration_s, 1e-4);
candidateDuration_s = lowerDuration_s;
previousDuration_s = 0;
feasibleDuration_s = Inf;

while candidateDuration_s <= maximumDuration_s * (1 + 1e-12)
    [candidateCommand, ~] = buildAzElCommand(routePosition_deg, ...
        request.initialState, goalState, request.limits, request.options, ...
        startTime_s + candidateDuration_s, 0);
    motion = validateAzElMotion(candidateCommand, ...
        request.limits, request.options);
    if motion.isValid
        feasibleDuration_s = candidateDuration_s;
        break;
    end
    previousDuration_s = candidateDuration_s;
    candidateDuration_s = min(maximumDuration_s, ...
        max(candidateDuration_s .* 1.35, ...
            candidateDuration_s + temporalResolution_s));
    if candidateDuration_s <= previousDuration_s + eps(previousDuration_s)
        break;
    end
end
if ~isfinite(feasibleDuration_s)
    earliestArrival_s = Inf;
    return;
end

lower_s = previousDuration_s;
upper_s = feasibleDuration_s;
for iterationIndex = 1:36
    if upper_s - lower_s <= max(1e-6, 0.02 .* temporalResolution_s)
        break;
    end
    middle_s = 0.5 .* (lower_s + upper_s);
    [candidateCommand, ~] = buildAzElCommand(routePosition_deg, ...
        request.initialState, goalState, request.limits, request.options, ...
        startTime_s + middle_s, 0);
    motion = validateAzElMotion(candidateCommand, ...
        request.limits, request.options);
    if motion.isValid
        upper_s = middle_s;
    else
        lower_s = middle_s;
    end
end
earliestArrival_s = startTime_s + upper_s;
end

function lowerDuration_s = routeDurationLowerBound(routePosition_deg, ...
        request, goalState)
%% Section 0: Header & Readme
% SYNTAX
%   lowerDuration_s = routeDurationLowerBound(routePosition_deg, ...
%       request, goalState)
%**************************************************************************
% PURPOSE
%   - Combine endpoint physics and per-axis route travel into a lower bound.
%**************************************************************************
% INPUTS
%   - routePosition_deg (N-by-2 numeric)
%   - request (normalized scalar request)
%   - goalState (scalar complete-state struct)
%**************************************************************************
% OUTPUTS
%   - lowerDuration_s (nonnegative scalar)
%**************************************************************************
% UNITS
%   - Duration is seconds.

endpointBound = estimateAzElLowerBound( ...
    request.initialState, goalState, request.limits);
axisTravel_deg = sum(abs(diff(routePosition_deg, 1, 1)), 1);
routeRateBound_s = max(axisTravel_deg ./ ...
    request.limits.maxVelocity_deg_s);
lowerDuration_s = max(endpointBound.duration_s, routeRateBound_s);
end

function sequence_s = buildArrivalSequence(earliest_s, latest_s, step_s)
%% Section 0: Header & Readme
% SYNTAX
%   sequence_s = buildArrivalSequence(earliest_s, latest_s, step_s)
%**************************************************************************
% PURPOSE
%   - Build an earliest-first deterministic time sequence including bounds.
%**************************************************************************
% INPUTS
%   - earliest_s, latest_s, step_s (finite scalars)
%**************************************************************************
% OUTPUTS
%   - sequence_s (numeric row vector)
%**************************************************************************
% UNITS
%   - Time is seconds.

if earliest_s > latest_s
    sequence_s = zeros(1, 0);
    return;
end
sequence_s = earliest_s:step_s:latest_s;
if isempty(sequence_s)
    sequence_s = earliest_s;
end
if latest_s - sequence_s(end) > 1e-10
    sequence_s(end + 1) = latest_s;
end
end

function waitDuration_s = buildWaitSequence(arrivalTime_s, ...
        routePosition_deg, goalState, request, temporalResolution_s, ...
        startCanWait, hasTimeVariation)
%% Section 0: Header & Readme
% SYNTAX
%   waitDuration_s = buildWaitSequence(arrivalTime_s, ...
%       routePosition_deg, goalState, request, temporalResolution_s, ...
%       startCanWait, hasTimeVariation)
%**************************************************************************
% PURPOSE
%   - Enumerate only physically stationary departure waits under the same
%     planner-selected temporal refinement used for the whole request.
%**************************************************************************
% INPUTS
%   - arrivalTime_s (finite scalar)
%   - routePosition_deg (N-by-2 numeric)
%   - goalState (scalar complete-state struct)
%   - request (normalized scalar request)
%   - temporalResolution_s (positive scalar)
%   - startCanWait, hasTimeVariation (logical scalars)
%**************************************************************************
% OUTPUTS
%   - waitDuration_s (numeric row vector)
%**************************************************************************
% UNITS
%   - Time is seconds.

waitDuration_s = 0;
if ~startCanWait || ~hasTimeVariation
    return;
end
minimumMotion_s = routeDurationLowerBound( ...
    routePosition_deg, request, goalState);
maximumWait_s = arrivalTime_s - request.initialState.time_s - ...
    minimumMotion_s;
if maximumWait_s < temporalResolution_s
    return;
end
waitDuration_s = 0:temporalResolution_s:maximumWait_s;
maximumWaitCandidateCount = 32;
if numel(waitDuration_s) > maximumWaitCandidateCount
    selectedIndices = unique(round(linspace(1, numel(waitDuration_s), ...
        maximumWaitCandidateCount)));
    waitDuration_s = waitDuration_s(selectedIndices);
end
end

function isExpired = budgetExpired(request, planningTimer)
%% Section 0: Header & Readme
% SYNTAX
%   isExpired = budgetExpired(request, planningTimer)
%**************************************************************************
% PURPOSE
%   - Apply the one caller-facing global planning wall-time bound.
%**************************************************************************
% INPUTS
%   - request (normalized scalar request)
%   - planningTimer (tic handle)
%**************************************************************************
% OUTPUTS
%   - isExpired (logical scalar)
%**************************************************************************
% UNITS
%   - Time is seconds.

isExpired = toc(planningTimer) >= request.options.planningWallTime_s;
end

function hasVariation = sceneHasTimeVariation(request)
%% Section 0: Header & Readme
% SYNTAX
%   hasVariation = sceneHasTimeVariation(request)
%**************************************************************************
% PURPOSE
%   - Detect whether timing or an explicit wait can change scene occupancy.
%**************************************************************************
% INPUTS
%   - request (normalized scalar request)
%**************************************************************************
% OUTPUTS
%   - hasVariation (logical scalar)
%**************************************************************************
% UNITS
%   - Not applicable.

hasVariation = request.goal.type == "moving";
for obstacleIndex = 1:numel(request.obstacles)
    obstacle = request.obstacles{obstacleIndex};
    for sampleIndex = 2:numel(obstacle.time_s)
        if ~isequaln(obstacle.az_deg{1}, obstacle.az_deg{sampleIndex}) || ...
                ~isequaln(obstacle.el_deg{1}, obstacle.el_deg{sampleIndex})
            hasVariation = true;
            return;
        end
    end
end
end

function message = summarizeValidationFailure(validation)
%% Section 0: Header & Readme
% SYNTAX
%   message = summarizeValidationFailure(validation)
%**************************************************************************
% PURPOSE
%   - Convert independent validation evidence to a stable factual category.
%**************************************************************************
% INPUTS
%   - validation (scalar validation struct)
%**************************************************************************
% OUTPUTS
%   - message (scalar string)
%**************************************************************************
% UNITS
%   - Not applicable.

if ~validation.motion.isValid
    message = "MotionLimitViolation";
elseif ~validation.collision.resolved
    message = "CollisionClearanceUnresolved";
elseif ~validation.collision.collisionFree
    message = "ObstacleClearanceViolation";
elseif ~validation.terminalStateIsValid
    message = "TerminalStateMismatch";
elseif ~validation.firstArrivalIsValid
    message = "FirstArrivalMismatch";
else
    message = "IndependentValidationFailure";
end
end

function search = searchTemplate()
%% Section 0: Header & Readme
% SYNTAX
%   search = searchTemplate()
%**************************************************************************
% PURPOSE
%   - Return the stable private route-search result schema.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - search (scalar struct)
%**************************************************************************
% UNITS
%   - Time fields are seconds.

search = struct( ...
    "success", false, ...
    "command", struct(), ...
    "waits", struct([]), ...
    "validation", struct(), ...
    "arrivalTime_s", Inf, ...
    "routeIndex", 0, ...
    "routePosition_deg", zeros(0, 2), ...
    "candidateCount", 0, ...
    "validationFailureCount", 0, ...
    "failureMessages", strings(0, 1), ...
    "budgetExpired", false);
end
