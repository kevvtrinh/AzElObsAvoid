function result = searchArrivalTimes(request, planningEnvironment, baseResult, ...
        priorAttempts, maximumTrialCount, trialInterval_s)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.searchArrivalTimes( ...
%       request, planningEnvironment, baseResult, priorAttempts, ...
%       maximumTrialCount)
%   result = obstacleAvoidance.planning.searchArrivalTimes( ...
%       request, planningEnvironment, baseResult, priorAttempts, ...
%       maximumTrialCount, trialInterval_s)
%**************************************************************************
% PURPOSE
%   - Try a list of possible arrival times, starting with the earliest listed.
%   - Check each returned motion against the original planning request.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Prepared states, limits, and options for the arrival-time search.
%   - planningEnvironment (scalar struct)
%       Prepared obstacles and the initial obstacle-vertex connections.
%   - baseResult (scalar struct)
%       Earlier result to keep if no trial improves it. A validated motion
%       here is kept even when a trial failure stops the search, unless that
%       failure is an independent-validation rejection.
%   - priorAttempts (struct array)
%       Planner attempts completed before arrival-time search.
%   - maximumTrialCount (positive integer scalar)
%       Maximum number of planning trials. Times rejected by the endpoint
%       checks do not use a trial. The caller may lower this limit when a
%       validated motion is already available.
%   - trialInterval_s (1-by-2 numeric vector, optional)
%       [after, latest]: try only times later than after and no later than
%       latest. Omit it to try the whole request window.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       A validated motion or a failure result. Running out of trials keeps an
%       earlier valid motion when available; otherwise the reason is
%       "arrivalSearchExhausted". A trial failure that stops the search also
%       keeps an earlier valid motion, except an independent-validation
%       rejection, which is returned so the defect stays visible. Without an
%       earlier valid motion, a stopping failure keeps its own reason. Trying
%       discrete times does not prove the globally earliest arrival.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Find The Available Time Range

searchTimer       = tic;
initialState      = request.initialState;
suppliedGoalState = request.originalInputs.suppliedGoalState;
trialOptions      = request.options;
validateattributes(maximumTrialCount, {'numeric'}, ...
    {'scalar', 'finite', 'integer', 'positive'});
hasTrialInterval = nargin >= 6;
if hasTrialInterval
    validateattributes(trialInterval_s, {'numeric'}, ...
        {'real', 'finite', 'numel', 2, 'nondecreasing'});
end

searchStartTime_s = initialState.time_s;
latestTrialTime_s = request.goalState.time_s;
arrivalTimeStep_s = trialOptions.TemporalResolution_s;

sampleAndDeadlineTimes_s = latestTrialTime_s;
for obstacleIndex = 1:numel(planningEnvironment.preparedObstacles)
    sampleAndDeadlineTimes_s = [sampleAndDeadlineTimes_s; planningEnvironment.preparedObstacles(obstacleIndex).time_s(:)]; %#ok<AGROW>
end
if ~isempty(request.goalState.targetMotion)
    sampleAndDeadlineTimes_s = [sampleAndDeadlineTimes_s; request.goalState.targetMotion.time_s(:)];
    % Only try times covered by the supplied target history.
    searchStartTime_s = max(searchStartTime_s, request.goalState.targetMotion.time_s(1));
    latestTrialTime_s = min(latestTrialTime_s, request.goalState.targetMotion.time_s(end));
end

%% Section 2: Narrow The Arrival Range

% Exclude times earlier than the vehicle can travel to a fixed goal.
% If a valid motion exists, only try times that would improve its arrival.
earliestTrialTime_s = searchStartTime_s;
if isempty(request.goalState.targetMotion)
    minimumTravelTime_s = obstacleAvoidance.input.minimumTravelTime( ...
        initialState, request.goalState, request.limits);
    earliestTrialTime_s = max(earliestTrialTime_s, initialState.time_s + minimumTravelTime_s);
end
bestSoFarArrivalTime_s = NaN;
if baseResult.Success
    bestSoFarArrivalTime_s = baseResult.ArrivalTime_s;
    latestTrialTime_s      = min(latestTrialTime_s, bestSoFarArrivalTime_s - ...
        trialOptions.ArrivalTimeTolerance_s);
end
% Keep the physical bound for reporting; the interval only narrows the trials.
earliestPossibleArrival_s = earliestTrialTime_s;
if hasTrialInterval
    earliestTrialTime_s = max(earliestTrialTime_s, trialInterval_s(1));
    latestTrialTime_s   = min(latestTrialTime_s, trialInterval_s(2));
else
    trialInterval_s = [NaN, NaN];
end

%% Section 3: List Possible Arrival Times

% Example: with a 0.5 s step, regular trial times are 0.5 s apart.
% MaxArrivalCandidates caps this regular grid before it is allocated. Sample
% times inside the resulting time range are added separately below.
firstStepIndex = max(1, ceil((earliestTrialTime_s - searchStartTime_s - ...
    trialOptions.ArrivalTimeTolerance_s) / arrivalTimeStep_s));
naturalLastStepIndex = floor((latestTrialTime_s - searchStartTime_s) / arrivalTimeStep_s);
if ~isfinite(firstStepIndex)
    error('planner:UnrepresentableTemporalResolution', ...
        'TemporalResolution_s cannot form a finite arrival grid.');
end
lastStepIndex = min(naturalLastStepIndex, firstStepIndex + ...
    trialOptions.MaxArrivalCandidates - 1);
gridTimes_s = searchStartTime_s + (firstStepIndex:lastStepIndex).' * arrivalTimeStep_s;
if ~isempty(gridTimes_s) && (gridTimes_s(1) <= searchStartTime_s || ...
        any(~isfinite(gridTimes_s)) || any(diff(gridTimes_s) <= 0))
    error('planner:UnrepresentableTemporalResolution', ...
        'TemporalResolution_s does not advance time at this absolute time scale.');
end
gridWasTruncated = lastStepIndex < naturalLastStepIndex;
if gridWasTruncated
    latestTrialTime_s = min(latestTrialTime_s, gridTimes_s(end));
end

% Include obstacle and target sample times as well as regularly spaced
% times. These samples can mark changes that matter to arrival planning.
sampleAndDeadlineTimes_s = unique(sampleAndDeadlineTimes_s);
sampleAndDeadlineTimes_s = sampleAndDeadlineTimes_s( ...
    sampleAndDeadlineTimes_s > initialState.time_s & ...
    sampleAndDeadlineTimes_s >= earliestTrialTime_s - trialOptions.ArrivalTimeTolerance_s & ...
    sampleAndDeadlineTimes_s <= latestTrialTime_s);
candidateArrivalTimes_s = unique([gridTimes_s; sampleAndDeadlineTimes_s]);
candidateArrivalTimes_s = candidateArrivalTimes_s(candidateArrivalTimes_s <= latestTrialTime_s);
if hasTrialInterval
    % The interval excludes its start: try times after it only.
    candidateArrivalTimes_s = candidateArrivalTimes_s(candidateArrivalTimes_s > trialInterval_s(1));
end

trialOptions.GoalTimeMode = "fixedArrival";
% Each trial must meet one exact arrival time while retaining the original
% request for final validation, including through wrapped planning.
if ~isempty(request.parentRequest)
    parentRequest = request.parentRequest;
else
    parentRequest = obstacleAvoidance.planning.createParentRequest(request);
end

%% Section 4: Check Endpoints And Plan Each Viable Arrival Time

storageCount           = min(maximumTrialCount, numel(candidateArrivalTimes_s));
trialTimes_s           = NaN(storageCount, 1);
rejectedArrivalTimes_s = NaN(numel(candidateArrivalTimes_s), 1);
attempts               = priorAttempts;
attemptStartIndex      = numel(attempts) + 1;

selectedResult                = struct([]);
stoppingFailureResult         = struct([]);
trialWasSelected              = false;
failureStopsPlanning          = false;
solverTrialCount              = 0;
rejectedTimeCount             = 0;
nextUnprocessedCandidateIndex = 0;
for candidateIndex = 1:numel(candidateArrivalTimes_s)
    if solverTrialCount >= maximumTrialCount
        nextUnprocessedCandidateIndex = candidateIndex;
        break
    end
    trialTime_s                    = candidateArrivalTimes_s(candidateIndex);
    endpointGoalState              = request.goalState;
    endpointGoalState.time_s       = trialTime_s;
    endpointFeasible               = false;
    trialEarliestPossibleArrival_s = initialState.time_s;
    % For an intercept, the target's position changes with the trial time.
    % Calculate matching velocity or acceleration only when requested.
    try
        if ~isempty(endpointGoalState.targetMotion)
            if trialOptions.MatchTargetVelocity || trialOptions.MatchTargetAcceleration
                [targetPosition_units, targetVelocity_units_s, ...
                    targetAcceleration_units_s2] = ...
                    obstacleAvoidance.input.targetPositionAtTime( ...
                    endpointGoalState.targetMotion, trialTime_s);
            else
                targetPosition_units = obstacleAvoidance.input.targetPositionAtTime( ...
                    endpointGoalState.targetMotion, trialTime_s);
            end
            endpointGoalState.position_units = targetPosition_units;
            if trialOptions.MatchTargetVelocity
                endpointGoalState.velocity_units_s = targetVelocity_units_s;
            end
            if trialOptions.MatchTargetAcceleration
                endpointGoalState.acceleration_units_s2 = targetAcceleration_units_s2;
            end
        end
        trialEarliestPossibleArrival_s = initialState.time_s + ...
            obstacleAvoidance.input.minimumTravelTime( ...
            initialState, endpointGoalState, request.limits);
        [endpointFeasible, endpointMessage, endpointReason] = ...
            obstacleAvoidance.input.validatePlannerEndpoints( ...
            planningEnvironment.preparedObstacles, initialState, endpointGoalState, ...
            request.limits, trialOptions);
        if endpointFeasible && trialTime_s < trialEarliestPossibleArrival_s - ...
                trialOptions.ArrivalTimeTolerance_s
            endpointFeasible = false;
        end
    catch exception
        % A target path can have a corner where a requested derivative is
        % undefined. Skip that time; other errors still need to be reported.
        if string(exception.identifier) ~= "planner:UndefinedTargetDerivative"
            rethrow(exception);
        end
    end
    if endpointFeasible && norm(endpointGoalState.position_units - ...
            initialState.position_units) <= trialOptions.ConstraintTolerance
        % This planner requires distinct start and goal positions.
        endpointFeasible = false;
    end
    if ~endpointFeasible
        rejectedTimeCount                         = rejectedTimeCount + 1;
        rejectedArrivalTimes_s(rejectedTimeCount) = trialTime_s;
        continue
    end

    solverTrialCount                  = solverTrialCount + 1;
    trialTimes_s(solverTrialCount, 1) = trialTime_s;

    % Rebuild the trial from the supplied goal so input normalization and
    % target-matching checks still apply to the original endpoint constraints.
    trialGoalState        = suppliedGoalState;
    trialGoalState.time_s = trialTime_s;

    trialRequest                         = parentRequest;
    trialRequest.FixedArrivalTrialTime_s = trialTime_s;
    % Save the endpoint check with all its inputs. The next planner call
    % can reuse it only when its prepared inputs match these values.
    endpointValidationOptions = struct( ...
        'GoalTimeMode',           trialOptions.GoalTimeMode, ...
        'WrapX',                  trialOptions.WrapX, ...
        'WrapY',                  trialOptions.WrapY, ...
        'ArrivalTimeTolerance_s', trialOptions.ArrivalTimeTolerance_s);
    initialEndpointState = struct( ...
        'time_s',                initialState.time_s, ...
        'position_units',        initialState.position_units, ...
        'velocity_units_s',      initialState.velocity_units_s, ...
        'acceleration_units_s2', initialState.acceleration_units_s2);
    goalEndpointState = struct( ...
        'time_s',                endpointGoalState.time_s, ...
        'position_units',        endpointGoalState.position_units, ...
        'velocity_units_s',      endpointGoalState.velocity_units_s, ...
        'acceleration_units_s2', endpointGoalState.acceleration_units_s2);
    endpointValidationKey                   = struct();
    endpointValidationKey.PreparedObstacles = planningEnvironment.preparedObstacles;
    endpointValidationKey.RequestHorizon_s  = [initialState.time_s, endpointGoalState.time_s];
    endpointValidationKey.InitialState      = initialEndpointState;
    endpointValidationKey.GoalState         = goalEndpointState;
    endpointValidationKey.Limits            = request.limits;
    endpointValidationKey.Options           = endpointValidationOptions;
    trialRequest.EndpointValidation         = struct( ...
        'Key',      endpointValidationKey, ...
        'Feasible', endpointFeasible, ...
        'Message',  endpointMessage, ...
        'Reason',   endpointReason);
    % Arrival trials share the obstacles and start time. Save the initial
    % obstacle connections so the next call can reuse them after checking
    % that the obstacles, limits, time, and tolerance still match.
    trialRequest.InitialVertexVisibility = struct( ...
        'Key', struct( ...
            'PreparedObstacles',   {planningEnvironment.preparedObstacles}, ...
            'SnapshotTime_s',      initialState.time_s, ...
            'Limits',              request.limits, ...
            'ConstraintTolerance', trialOptions.ConstraintTolerance), ...
        'VertexVisibility', planningEnvironment.vertexVisibility);
    attempt = obstacleAvoidance.planning.createAttemptRecord( ...
        numel(attempts) + 1, "arrivalTimeTrial");
    attempt.TrialTime_s               = trialTime_s;
    attempt.EarliestPossibleArrival_s = trialEarliestPossibleArrival_s;
    attempt.BestSoFarArrival_s        = bestSoFarArrivalTime_s;
    attempt.SolverAttempted           = true;
    attemptTimer                      = tic;
    try
        preparedTrialRequest = obstacleAvoidance.planning.prepareRequest( ...
            planningEnvironment.preparedObstacles, initialState, trialGoalState, ...
            request.limits, trialOptions, trialRequest);

        % Plan this trial while retaining the original request for validation.
        if preparedTrialRequest.options.WrapX || preparedTrialRequest.options.WrapY
            trialResult = obstacleAvoidance.planning.planWrappedMotion(preparedTrialRequest);
        else
            trialResult = obstacleAvoidance.planning.planMotion(preparedTrialRequest);
        end
    catch exception
        failureIsExpected = string(exception.identifier) == ...
            ["planner:UndefinedTargetDerivative", "planTrajectory:CoincidentEndpoints"];
        if any(failureIsExpected)
            attempt.FailureStage       = "endpoint";
            attempt.FailureKind        = string(exception.identifier);
            attempt.NextMethodAllowed  = true;
            attempt.NextMethodReason   = attempt.FailureKind;
            attempt.NextAttemptAllowed = true;
            attempt.ElapsedTime_s      = toc(attemptTimer);
            attempts(end + 1, 1)       = attempt; %#ok<AGROW>
            continue
        end
        rethrow(exception);
    end
    attempt.ElapsedTime_s          = toc(attemptTimer);
    attempt.ChildAttempts          = trialResult.Attempts;
    attempt.GraphConnected         = trialResult.VisibilityGraph.IsConnected;
    attempt.GraphIsFullyEnumerated = trialResult.VisibilityGraph.GraphIsFullyEnumerated;
    attempt.RouteNodeCount         = size(trialResult.Route_units, 1);
    attempt.RouteLength_units      = trialResult.VisibilityGraph.RouteLength_units;
    attempt.ExpandedCount          = trialResult.VisibilityGraph.ExpandedCount;
    attempt.IterationCount         = sumAttemptField(trialResult.Attempts, "IterationCount");
    attempt.CandidateSuccess       = trialResult.Success || ...
        readLogical(trialResult.SolverDiagnostics, "Accepted");
    attempt.OptimizerFeasible           = readLogical(trialResult, "OptimizerFeasible");
    attempt.OptimizerIterateUnavailable = readLogical( ...
        trialResult, "OptimizerIterateUnavailable");
    [attempt.FailureStage, attempt.FailureKind, ...
        attempt.AlternativeGuideEligible] = readTrialFailure(trialResult);
    attempt.Success = trialResult.Success;
    if trialResult.Success && isfinite(trialResult.ArrivalTime_s)
        attempt.CandidateArrival_s = trialResult.ArrivalTime_s;
    end
    if trialResult.Success
        % Times are tried in increasing order. Select this successful trial;
        % times between the listed candidates remain unchecked.
        for priorIndex = reshape(find([attempts.Success]), 1, [])
            attempts(priorIndex).Selected = false;
        end
        attempt.Selected     = true;
        attempts(end + 1, 1) = attempt; %#ok<AGROW>
        selectedResult       = trialResult;
        trialWasSelected     = true;
        break
    end
    attempt.NextMethodAllowed = ...
        obstacleAvoidance.planning.nextMethodAllowed(trialResult);
    if string(trialResult.TerminationReason) == "noVisibilityRoute" && ...
            isempty(request.goalState.targetMotion)
        % The static graph cannot connect these fixed endpoints. Trying the
        % same graph at another arrival time cannot create a route.
        attempt.NextMethodAllowed = false;
    end
    attempt.NextMethodReason   = attempt.FailureKind;
    attempt.NextAttemptAllowed = attempt.NextMethodAllowed;
    if attempt.NextMethodAllowed
        attempts(end + 1, 1) = attempt; %#ok<AGROW>
        continue
    end
    % Keep the trial's own explanation with its attempt record.
    attempt.Message       = string(trialResult.Message);
    attempts(end + 1, 1)  = attempt; %#ok<AGROW>
    stoppingFailureResult = trialResult;
    % Keep the time that failed. Wrapped planning may replace the request
    % fields later, but this value must still identify the attempted arrival.
    stoppingFailureResult.FixedArrivalTrialTime_s = trialTime_s;
    failureStopsPlanning                          = true;
    break
end

%% Section 5: Return The Selected Motion Or Explain Why Planning Stopped

% Distinguish a capped time grid, an exhausted trial budget, and a checked
% list with no success. None proves that all other arrival times are impossible.
trialTimes_s           = trialTimes_s(1:solverTrialCount);
rejectedArrivalTimes_s = rejectedArrivalTimes_s(1:rejectedTimeCount);
candidateLimitReached  = ~trialWasSelected && ~failureStopsPlanning && gridWasTruncated;
trialLimitReached      = ~trialWasSelected && ~failureStopsPlanning && ...
    nextUnprocessedCandidateIndex > 0;
searchWindowExhausted = ~trialWasSelected && ~failureStopsPlanning && ...
    ~candidateLimitReached && ~trialLimitReached;
% A stopping trial failure ends the search but does not erase a validated
% motion found earlier. An independent-validation rejection is different: the
% engine returned motion the validator refused, and that defect must be seen.
stoppingFailureIsValidationDefect = failureStopsPlanning && ...
    string(stoppingFailureResult.TerminationReason) == "invalidMotion";
keepBestExistingMotion = baseResult.Success && ~trialWasSelected && ...
    ~stoppingFailureIsValidationDefect;
% When a kept motion hides a stopping failure, keep that failure's details
% beside it so the defect can still be traced.
stoppingTrial = struct([]);
if keepBestExistingMotion && failureStopsPlanning
    stoppingTrial = struct( ...
        'AttemptIndex',      numel(attempts), ...
        'TrialTime_s',       stoppingFailureResult.FixedArrivalTrialTime_s, ...
        'TerminationReason', string(stoppingFailureResult.TerminationReason), ...
        'Message',           string(stoppingFailureResult.Message), ...
        'FailureStage',      attempts(end).FailureStage, ...
        'FailureKind',       attempts(end).FailureKind, ...
        'SolverDiagnostics', stoppingFailureResult.SolverDiagnostics);
end
if keepBestExistingMotion
    acceptedIndices = find([attempts.Success]);
    if ~isempty(acceptedIndices)
        acceptedTimes_s     = [attempts(acceptedIndices).CandidateArrival_s];
        [~, selectedOffset] = min(acceptedTimes_s);
        selectedIndex       = acceptedIndices(selectedOffset);
        attempts(selectedIndex).Selected = true;
    end
end
if trialWasSelected
    result = selectedResult;
elseif keepBestExistingMotion || ~failureStopsPlanning
    result = baseResult;
else
    % The returned failure replaces any earlier motion, so select nothing.
    [attempts.Selected] = deal(false);
    result = stoppingFailureResult;
end
result.Attempts       = attempts;
result.TemporalSearch = struct( ...
    'Resolution_s',              arrivalTimeStep_s, ...
    'TrialTime_s',               trialTimes_s, ...
    'MaximumTrialCount',         maximumTrialCount, ...
    'SolverTrialCount',          solverTrialCount, ...
    'PrescreenedCandidateCount', rejectedTimeCount, ...
    'PrescreenedTime_s',         rejectedArrivalTimes_s, ...
    'CandidateLimitReached',     candidateLimitReached, ...
    'TrialLimitReached',         trialLimitReached, ...
    'SearchWindowExhausted',     searchWindowExhausted, ...
    'TerminalFailure',           failureStopsPlanning, ...
    'GlobalEarliestProven',      false, ...
    'EarliestPossibleArrival_s', earliestPossibleArrival_s, ...
    'TrialInterval_s',           trialInterval_s, ...
    'BestSoFarArrival_s',        bestSoFarArrivalTime_s, ...
    'BestSoFar',                 keepBestExistingMotion, ...
    'StoppingTrial',             stoppingTrial, ...
    'AttemptStartIndex',         attemptStartIndex);
if trialWasSelected
    result.Message = "An arrival-time trial passed independent validation; earlier gaps remain unsearched.";
elseif keepBestExistingMotion && failureStopsPlanning
    result.Message = "An arrival-time trial failure stopped the search; the validated best plan so far was kept.";
elseif keepBestExistingMotion
    result.Message = "No earlier declared clock was proven; the validated best plan so far was kept.";
elseif ~result.Success && ~failureStopsPlanning
    result.TerminationReason = "arrivalSearchExhausted";
    result.Message           = "No declared arrival trial was proven. Unsearched times and solver failures do not prove infeasibility.";
end
result.ElapsedTime_s = baseResult.ElapsedTime_s + toc(searchTimer);
end

%% Section 6: Local Functions

function [failureStage, failureKind, alternativeGuideEligible] = readTrialFailure(trialResult)
    % Read the trial's failure fields. If absent, use its last planning attempt.
    failureStage             = readString(trialResult, "FailureStage");
    failureKind              = readString(trialResult, "FailureKind");
    alternativeGuideEligible = readLogical(trialResult, "AlternativeGuideEligible");
    if failureStage == "" && ~isempty(trialResult.Attempts)
        failureStage             = string(trialResult.Attempts(end).FailureStage);
        failureKind              = string(trialResult.Attempts(end).FailureKind);
        alternativeGuideEligible = logical( ...
            trialResult.Attempts(end).AlternativeGuideEligible);
    end
end

function total = sumAttemptField(attempts, fieldName)
    % Total a diagnostic field across the trial's planning attempts.
    total = 0;
    if ~isempty(attempts)
        total = sum([attempts.(fieldName)]);
    end
end

function value = readLogical(record, fieldName)
    value = false;
    if isstruct(record) && isscalar(record) && isfield(record, fieldName)
        value = logical(record.(fieldName));
    end
end

function value = readString(record, fieldName)
    value = "";
    if isstruct(record) && isscalar(record) && isfield(record, fieldName)
        value = string(record.(fieldName));
    end
end
