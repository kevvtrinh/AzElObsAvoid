function result = searchArrivalTimes(previous, plannerCore)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.input.searchArrivalTimes(previous, plannerCore)
%**************************************************************************
% PURPOSE
%   - Search declared chronological fixed-arrival trials.
%   - Preserve the outer request for the planner's single acceptance gate.
%**************************************************************************
% INPUTS
%   - previous (scalar struct)
%       Planner result containing the request and any valid incumbent.
%   - plannerCore (function handle)
%       Private planner implementation carrying the outer request context.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Valid candidate or an honest exhausted-search outcome. An exhausted
%       search returns Success = false with TerminationReason
%       "arrivalSearchExhausted" and retains a valid incumbent. Invalid
%       input throws an error.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Collect The Physical Window And Its Source Boundaries

searchTimer       = tic;
initialState      = previous.Inputs.initialState;
suppliedGoalState = previous.SuppliedGoalState;
trialOptions      = previous.Options;

startTime_s   = initialState.time_s;
horizonTime_s = previous.Inputs.goalState.time_s;
resolution_s  = trialOptions.TemporalResolution_s;

boundaryTimes_s = horizonTime_s;
for obstacleIndex = 1:numel(previous.PreparedObstacles)
    boundaryTimes_s = [boundaryTimes_s; previous.PreparedObstacles(obstacleIndex).time_s(:)]; %#ok<AGROW>
end
if isfield(suppliedGoalState, 'targetMotion') && ~isempty(suppliedGoalState.targetMotion)
    boundaryTimes_s = [boundaryTimes_s; suppliedGoalState.targetMotion.time_s(:)];
    startTime_s     = max(startTime_s, suppliedGoalState.targetMotion.time_s(1));
    horizonTime_s   = min(horizonTime_s, suppliedGoalState.targetMotion.time_s(end));
end

%% Section 2: Bound The Window By Physics And The Valid Incumbent

earliestTime_s = startTime_s;
if isempty(previous.Inputs.goalState.targetMotion)
    minimumTravelTime_s = obstacleAvoidance.input.minimumTravelTime( ...
        initialState, previous.Inputs.goalState, previous.Limits);
    earliestTime_s = max(earliestTime_s, initialState.time_s + minimumTravelTime_s);
end
incumbentArrivalTime_s = NaN;
if previous.Success
    incumbentArrivalTime_s = previous.ArrivalTime_s;
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
if isfield(previous, 'OuterRequest')
    outerRequest = previous.OuterRequest;
else
    outerRequest = struct( ...
        'SuppliedLimits',     previous.SuppliedLimits, ...
        'SuppliedGoalState',  previous.SuppliedGoalState, ...
        'RequestedGoalState', previous.RequestedGoalState, ...
        'Obstacles',          {previous.Inputs.obstacles}, ...
        'GoalTime_s',         previous.Inputs.goalState.time_s, ...
        'GoalTimeMode',       previous.Options.GoalTimeMode);
end

%% Section 4: Screen Endpoint Physics, Then Spend The Solver Budget

maximumTrialCount = trialOptions.MaxArrivalTrials;
storageCount = min(maximumTrialCount, numel(candidateTimes_s));
trialTimes_s            = NaN(storageCount, 1);
trialTerminationReasons = strings(storageCount, 1);

result             = previous;
trialWasSelected   = false;
triedCount         = 0;
prescreenCount     = 0;
for candidateIndex = 1:numel(candidateTimes_s)
    if triedCount >= maximumTrialCount
        break
    end
    trialTime_s = candidateTimes_s(candidateIndex);
    endpointGoalState = previous.Inputs.goalState;
    endpointGoalState.time_s = trialTime_s;
    endpointFeasible = false;
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
        [endpointFeasible, ~, ~] = ...
            obstacleAvoidance.input.validatePlannerEndpoints( ...
            previous.PreparedObstacles, initialState, endpointGoalState, ...
            previous.Limits, trialOptions);
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
        continue
    end

    triedCount                  = triedCount + 1;
    trialTimes_s(triedCount, 1) = trialTime_s;
    trialGoalState              = suppliedGoalState;
    trialGoalState.time_s       = trialTime_s;
    trialRequest                = outerRequest;
    trialRequest.FixedArrivalTrialTime_s = trialTime_s;
    try
        candidate = plannerCore(previous.PreparedObstacles, initialState, trialGoalState, ...
            previous.RequestedLimits, trialOptions, trialRequest);
    catch exception
        failureIsExpected = string(exception.identifier) == ...
            ["planner:UndefinedTargetDerivative", "planTrajectory:CoincidentEndpoints"];
        if any(failureIsExpected)
            trialTerminationReasons(triedCount) = string(exception.identifier);
            continue
        end
        rethrow(exception);
    end
    trialTerminationReasons(triedCount) = candidate.TerminationReason;
    if candidate.Success
        result           = candidate;
        trialWasSelected = true;
        break
    end
end

%% Section 5: Record The Honest Chronological Search Outcome

trialTimes_s = trialTimes_s(1:triedCount);
candidateLimitReached = ~trialWasSelected && gridWasTruncated && ...
    triedCount < maximumTrialCount;
result.TemporalSearch = struct( ...
    'Resolution_s',                  resolution_s, ...
    'TrialTime_s',                   trialTimes_s, ...
    'TrialTerminationReason',        trialTerminationReasons(1:triedCount), ...
    'SolverTrialCount',              triedCount, ...
    'PrescreenedCandidateCount',     prescreenCount, ...
    'CandidateLimitReached',         candidateLimitReached, ...
    'GlobalEarliestProven',          false, ...
    'NecessaryArrivalBound_s',       earliestTime_s, ...
    'IncumbentArrival_s',            incumbentArrivalTime_s, ...
    'RetainedIncumbent',             previous.Success && ~trialWasSelected, ...
    'PriorTerminationReason',        previous.TerminationReason);
if trialWasSelected
    result.Message = "A chronological fixed-arrival trial passed independent validation; earlier gaps remain unsearched.";
elseif ~result.Success
    result.TerminationReason = "arrivalSearchExhausted";
    result.Message = "No declared arrival trial was certified. Unsearched times and solver failures do not prove infeasibility.";
end
result.ElapsedTime_s = previous.ElapsedTime_s + toc(searchTimer);
end
