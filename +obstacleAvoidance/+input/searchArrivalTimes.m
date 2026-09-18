function result = searchArrivalTimes(request, requestContext, scene, baseResult, ...
        priorAttempts, plannerCore, attemptFactory, fallbackPolicy, maximumTrialCount)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.input.searchArrivalTimes( ...
%       request, requestContext, scene, baseResult, priorAttempts, ...
%       plannerCore, attemptFactory, fallbackPolicy, maximumTrialCount)
%**************************************************************************
% PURPOSE
%   - Search declared chronological fixed-arrival trials.
%   - Preserve the outer request for the planner's single acceptance gate.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Normalized planar request that owns the chronological trials.
%   - requestContext (scalar struct)
%       Original planar inputs, provenance, and optional outer request.
%   - scene (scalar struct)
%       Prepared geometry owned by the planar request.
%   - baseResult (scalar struct)
%       Outcome retained on exhaustion, including any valid incumbent.
%   - priorAttempts (struct array)
%       Planner attempts completed before chronological search.
%   - plannerCore (function handle)
%       Private planner implementation carrying the outer request context.
%   - attemptFactory (function handle)
%       Creates one planner-level attempt record with the public schema.
%   - fallbackPolicy (function handle)
%       Returns true only when another physical arrival clock is admitted.
%   - maximumTrialCount (positive integer scalar)
%       Solver attempts available to this search. A validated incumbent may
%       deliberately use a smaller refinement budget than the public maximum.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Valid candidate or an honest exhausted-search outcome. Exhaustion
%       retains a validated incumbent when one exists; otherwise it returns
%       Success = false with TerminationReason "arrivalSearchExhausted".
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Collect The Physical Window And Its Source Boundaries

searchTimer       = tic;
initialState      = request.initialState;
suppliedGoalState = requestContext.suppliedGoalState;
trialOptions      = request.options;
if nargin < 9 || isempty(maximumTrialCount)
    maximumTrialCount = trialOptions.MaxArrivalTrials;
end
validateattributes(maximumTrialCount, {'numeric'}, ...
    {'scalar', 'finite', 'integer', 'positive'});

startTime_s   = initialState.time_s;
horizonTime_s = request.goalState.time_s;
resolution_s  = trialOptions.TemporalResolution_s;

boundaryTimes_s = horizonTime_s;
for obstacleIndex = 1:numel(scene.preparedObstacles)
    boundaryTimes_s = [boundaryTimes_s; scene.preparedObstacles(obstacleIndex).time_s(:)]; %#ok<AGROW>
end
if ~isempty(request.goalState.targetMotion)
    boundaryTimes_s = [boundaryTimes_s; request.goalState.targetMotion.time_s(:)];
    startTime_s     = max(startTime_s, request.goalState.targetMotion.time_s(1));
    horizonTime_s   = min(horizonTime_s, request.goalState.targetMotion.time_s(end));
end

%% Section 2: Bound The Window By Physics And The Valid Incumbent

earliestTime_s = startTime_s;
if isempty(request.goalState.targetMotion)
    minimumTravelTime_s = obstacleAvoidance.input.minimumTravelTime( ...
        initialState, request.goalState, request.limits);
    earliestTime_s = max(earliestTime_s, initialState.time_s + minimumTravelTime_s);
end
incumbentArrivalTime_s = NaN;
if baseResult.Success
    incumbentArrivalTime_s = baseResult.ArrivalTime_s;
    horizonTime_s = min(horizonTime_s, incumbentArrivalTime_s - ...
        trialOptions.ArrivalTimeTolerance_s);
end

%% Section 3: Declare The Bounded Chronological Candidate Array

firstStepIndex = max(1, ceil((earliestTime_s - startTime_s - ...
    trialOptions.ArrivalTimeTolerance_s) / resolution_s));
naturalLastStepIndex = floor((horizonTime_s - startTime_s) / resolution_s);
if ~isfinite(firstStepIndex)
    error('planner:UnrepresentableTemporalResolution', ...
        'TemporalResolution_s cannot form a finite arrival grid.');
end
lastStepIndex = min(naturalLastStepIndex, firstStepIndex + ...
    trialOptions.MaxArrivalCandidates - 1);
gridTimes_s = startTime_s + (firstStepIndex:lastStepIndex).' * resolution_s;
if ~isempty(gridTimes_s) && (gridTimes_s(1) <= startTime_s || ...
        any(~isfinite(gridTimes_s)) || any(diff(gridTimes_s) <= 0))
    error('planner:UnrepresentableTemporalResolution', ...
        'TemporalResolution_s does not advance time at this absolute time scale.');
end
gridWasTruncated = lastStepIndex < naturalLastStepIndex;
if gridWasTruncated
    horizonTime_s = min(horizonTime_s, gridTimes_s(end));
end

% Exact scene boundaries remain candidates inside the bounded grid window.
boundaryTimes_s      = unique(boundaryTimes_s);
boundaryTimes_s      = boundaryTimes_s( ...
    boundaryTimes_s > initialState.time_s & ...
    boundaryTimes_s >= earliestTime_s - trialOptions.ArrivalTimeTolerance_s & ...
    boundaryTimes_s <= horizonTime_s);
candidateTimes_s     = unique([gridTimes_s; boundaryTimes_s]);
candidateTimes_s     = candidateTimes_s(candidateTimes_s <= horizonTime_s);

trialOptions.GoalTimeMode = "fixedArrival";
if ~isempty(requestContext.outerRequest)
    outerRequest = requestContext.outerRequest;
else
    outerRequest = struct( ...
        'SuppliedLimits',     requestContext.suppliedLimits, ...
        'SuppliedGoalState',  requestContext.suppliedGoalState, ...
        'RequestedGoalState', requestContext.requestedGoalState, ...
        'Obstacles',          {requestContext.obstacles}, ...
        'GoalTime_s',         request.goalState.time_s, ...
        'GoalTimeMode',       request.options.GoalTimeMode);
end

%% Section 4: Screen Endpoint Physics, Then Spend The Solver Budget

storageCount = min(maximumTrialCount, numel(candidateTimes_s));
trialTimes_s        = NaN(storageCount, 1);
prescreenedTimes_s  = NaN(numel(candidateTimes_s), 1);
attempts            = priorAttempts;
attemptStartIndex   = numel(attempts) + 1;

selectedResult               = struct([]);
terminalResult               = struct([]);
trialWasSelected             = false;
terminalFailure              = false;
triedCount                   = 0;
prescreenCount               = 0;
nextUnprocessedCandidateIndex = 0;
for candidateIndex = 1:numel(candidateTimes_s)
    if triedCount >= maximumTrialCount
        nextUnprocessedCandidateIndex = candidateIndex;
        break
    end
    trialTime_s = candidateTimes_s(candidateIndex);
    endpointGoalState = request.goalState;
    endpointGoalState.time_s = trialTime_s;
    endpointFeasible = false;
    trialNecessaryArrivalTime_s = initialState.time_s;
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
        trialNecessaryArrivalTime_s = initialState.time_s + ...
            obstacleAvoidance.input.minimumTravelTime( ...
            initialState, endpointGoalState, request.limits);
        [endpointFeasible, endpointMessage, endpointReason] = ...
            obstacleAvoidance.input.validatePlannerEndpoints( ...
            scene.preparedObstacles, initialState, endpointGoalState, ...
            request.limits, trialOptions);
        if endpointFeasible && trialTime_s < trialNecessaryArrivalTime_s - ...
                trialOptions.ArrivalTimeTolerance_s
            endpointFeasible = false;
        end
    catch exception
        if string(exception.identifier) ~= "planner:UndefinedTargetDerivative"
            rethrow(exception);
        end
    end
    if endpointFeasible && norm(endpointGoalState.position_units - ...
            initialState.position_units) <= trialOptions.ConstraintTolerance
        endpointFeasible = false;
    end
    if ~endpointFeasible
        prescreenCount = prescreenCount + 1;
        prescreenedTimes_s(prescreenCount) = trialTime_s;
        continue
    end

    triedCount                  = triedCount + 1;
    trialTimes_s(triedCount, 1) = trialTime_s;
    trialGoalState              = suppliedGoalState;
    trialGoalState.time_s       = trialTime_s;
    trialRequest                = outerRequest;
    trialRequest.FixedArrivalTrialTime_s = trialTime_s;
    % The recursive core may reuse this pass only after matching every gate
    % input against its independently normalized and prepared request.
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
    endpointValidationKey = struct();
    endpointValidationKey.PreparedObstacles = scene.preparedObstacles;
    endpointValidationKey.RequestHorizon_s  = [initialState.time_s, endpointGoalState.time_s];
    endpointValidationKey.InitialState      = initialEndpointState;
    endpointValidationKey.GoalState         = goalEndpointState;
    endpointValidationKey.Limits            = request.limits;
    endpointValidationKey.Options           = endpointValidationOptions;
    trialRequest.EndpointValidation = struct( ...
        'Key',      endpointValidationKey, ...
        'Feasible', endpointFeasible, ...
        'Message',  endpointMessage, ...
        'Reason',   endpointReason);
    attempt = attemptFactory(numel(attempts) + 1, ...
        "chronologicalFixedArrival");
    attempt.TrialTime_s             = trialTime_s;
    attempt.NecessaryArrivalBound_s = trialNecessaryArrivalTime_s;
    attempt.IncumbentArrival_s      = incumbentArrivalTime_s;
    attempt.SolverAttempted         = true;
    attemptTimer                    = tic;
    try
        candidate = plannerCore(scene.preparedObstacles, initialState, trialGoalState, ...
            request.limits, trialOptions, trialRequest);
    catch exception
        failureIsExpected = string(exception.identifier) == ...
            ["planner:UndefinedTargetDerivative", "planTrajectory:CoincidentEndpoints"];
        if any(failureIsExpected)
            attempt.FailureStage                 = "endpoint";
            attempt.FailureKind                  = string(exception.identifier);
            attempt.MethodFallbackEligible      = true;
            attempt.MethodFallbackReason        = attempt.FailureKind;
            attempt.FallbackEligible            = true;
            attempt.ElapsedTime_s               = toc(attemptTimer);
            attempts(end + 1, 1)                = attempt; %#ok<AGROW>
            continue
        end
        rethrow(exception);
    end
    attempt.ElapsedTime_s = toc(attemptTimer);
    attempt.ChildAttempts = candidate.Attempts;
    attempt.GraphConnected = candidate.VisibilityGraph.IsConnected;
    attempt.GraphIsFullyEnumerated = candidate.VisibilityGraph.GraphIsFullyEnumerated;
    attempt.RouteNodeCount    = size(candidate.Route_units, 1);
    attempt.RouteLength_units = candidate.VisibilityGraph.RouteLength_units;
    attempt.ExpandedCount     = candidate.VisibilityGraph.ExpandedCount;
    attempt.IterationCount    = sumAttemptField(candidate.Attempts, "IterationCount");
    attempt.CandidateSuccess  = candidate.Success || ...
        readLogical(candidate.SolverDiagnostics, "Accepted");
    attempt.OptimizerFeasible = readLogical(candidate, "OptimizerFeasible");
    attempt.OptimizerIterateUnavailable = readLogical( ...
        candidate, "OptimizerIterateUnavailable");
    [attempt.FailureStage, attempt.FailureKind, ...
        attempt.AlternativeGuideEligible] = failureEvidence(candidate);
    attempt.Success = candidate.Success;
    if candidate.Success && isfinite(candidate.ArrivalTime_s)
        attempt.CandidateArrival_s = candidate.ArrivalTime_s;
    end
    if candidate.Success
        for priorIndex = reshape(find([attempts.Success]), 1, [])
            attempts(priorIndex).Selected = false;
        end
        attempt.Selected = true;
        attempts(end + 1, 1) = attempt; %#ok<AGROW>
        selectedResult   = candidate;
        trialWasSelected = true;
        break
    end
    attempt.MethodFallbackEligible = fallbackPolicy(candidate);
    if string(candidate.TerminationReason) == "noVisibilityRoute" && ...
            isempty(request.goalState.targetMotion)
        % Static geometry and a fixed goal do not change with the clock.
        % Repeating the same disconnected exact graph cannot reveal a path.
        attempt.MethodFallbackEligible = false;
    end
    attempt.MethodFallbackReason   = attempt.FailureKind;
    attempt.FallbackEligible       = attempt.MethodFallbackEligible;
    if attempt.MethodFallbackEligible
        attempts(end + 1, 1) = attempt; %#ok<AGROW>
        continue
    end
    attempts(end + 1, 1) = attempt; %#ok<AGROW>
    terminalResult = candidate;
    % A terminal record declares the clock it attempted. A periodic failure
    % boundary later applies its own outer request, which carries no trial
    % marker, so publishing it here keeps the declaration through that.
    terminalResult.FixedArrivalTrialTime_s = trialTime_s;
    terminalFailure = true;
    break
end

%% Section 5: Record The Honest Chronological Search Outcome

trialTimes_s = trialTimes_s(1:triedCount);
prescreenedTimes_s = prescreenedTimes_s(1:prescreenCount);
candidateLimitReached = ~trialWasSelected && ~terminalFailure && gridWasTruncated;
trialLimitReached = ~trialWasSelected && ~terminalFailure && ...
    nextUnprocessedCandidateIndex > 0;
searchWindowExhausted = ~trialWasSelected && ~terminalFailure && ...
    ~candidateLimitReached && ~trialLimitReached;
retainedIncumbent = baseResult.Success && ~trialWasSelected && ~terminalFailure;
if retainedIncumbent
    acceptedIndices = find([attempts.Success]);
    if ~isempty(acceptedIndices)
        acceptedTimes_s = [attempts(acceptedIndices).CandidateArrival_s];
        [~, selectedOffset] = min(acceptedTimes_s);
        selectedIndex = acceptedIndices(selectedOffset);
        attempts(selectedIndex).Selected = true;
    end
end
if trialWasSelected
    result = selectedResult;
elseif terminalFailure
    result = terminalResult;
else
    result = baseResult;
end
result.Attempts = attempts;
result.TemporalSearch = struct( ...
    'Resolution_s',                  resolution_s, ...
    'TrialTime_s',                   trialTimes_s, ...
    'MaximumTrialCount',             maximumTrialCount, ...
    'SolverTrialCount',              triedCount, ...
    'PrescreenedCandidateCount',     prescreenCount, ...
    'PrescreenedTime_s',             prescreenedTimes_s, ...
    'CandidateLimitReached',         candidateLimitReached, ...
    'TrialLimitReached',             trialLimitReached, ...
    'SearchWindowExhausted',         searchWindowExhausted, ...
    'TerminalFailure',               terminalFailure, ...
    'GlobalEarliestProven',          false, ...
    'NecessaryArrivalBound_s',       earliestTime_s, ...
    'IncumbentArrival_s',            incumbentArrivalTime_s, ...
    'RetainedIncumbent',             retainedIncumbent, ...
    'AttemptStartIndex',             attemptStartIndex);
if trialWasSelected
    result.Message = "A chronological fixed-arrival trial passed independent validation; earlier gaps remain unsearched.";
elseif retainedIncumbent
    result.Message = "No earlier declared clock was certified; the validated incumbent was retained.";
elseif ~result.Success && ~terminalFailure
    result.TerminationReason = "arrivalSearchExhausted";
    result.Message = "No declared arrival trial was certified. Unsearched times and solver failures do not prove infeasibility.";
end
result.ElapsedTime_s = baseResult.ElapsedTime_s + toc(searchTimer);
end

function [failureStage, failureKind, alternativeGuideEligible] = failureEvidence(candidate)
    % Prefer the top-level typed result; otherwise use the decisive child attempt.
    failureStage = readString(candidate, "FailureStage");
    failureKind  = readString(candidate, "FailureKind");
    alternativeGuideEligible = readLogical(candidate, "AlternativeGuideEligible");
    if failureStage == "" && ~isempty(candidate.Attempts)
        failureStage = string(candidate.Attempts(end).FailureStage);
        failureKind  = string(candidate.Attempts(end).FailureKind);
        alternativeGuideEligible = logical( ...
            candidate.Attempts(end).AlternativeGuideEligible);
    end
end

function total = sumAttemptField(attempts, fieldName)
    % Sum one numeric child-attempt field without coupling selection to it.
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
