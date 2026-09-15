function result = searchArrivalTimes(previous)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.input.searchArrivalTimes(previous)
%**************************************************************************
% PURPOSE
%   - Search declared chronological fixed-arrival trials.
%   - Preserve the outer request for the planner's single acceptance gate.
%**************************************************************************
% INPUTS
%   - previous (scalar struct)
%       Planner result containing the request and any valid incumbent.
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

% Every declared obstacle and target sample time is a candidate arrival,
% independent of the grid, because the scene can only change there.
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
    horizonTime_s = min(horizonTime_s, incumbentArrivalTime_s - trialOptions.ArrivalTimeTolerance_s);
end

%% Section 3: Declare The Chronological Trial Times And Outer Request

% Keep the declared grid origin, but apply the physical bound before the
% trial budget. Impossible early times must not exclude later feasible ones.
firstStepIndex = max(1, ceil((earliestTime_s - startTime_s - trialOptions.ArrivalTimeTolerance_s) / resolution_s));
lastStepIndex  = min(firstStepIndex + trialOptions.MaxArrivalTrials - 1, ...
    floor((horizonTime_s - startTime_s) / resolution_s));

trialTimes_s = unique([startTime_s + (firstStepIndex:lastStepIndex)' * resolution_s; boundaryTimes_s]);
timeIsAfterStart  = trialTimes_s > initialState.time_s;
timeMeetsBound    = trialTimes_s >= earliestTime_s - trialOptions.ArrivalTimeTolerance_s;
timeWithinHorizon = trialTimes_s <= horizonTime_s;
trialTimes_s = trialTimes_s(timeIsAfterStart & timeMeetsBound & timeWithinHorizon);
trialTimes_s = trialTimes_s(1:min(numel(trialTimes_s), trialOptions.MaxArrivalTrials));

trialOptions.GoalTimeMode = "fixedArrival";
% Trials are accepted against the public request: the periodic request when
% this search runs inside the unwrapped frame, otherwise this one.
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

%% Section 4: Solve Each Candidate Independently On Its Actual Clock

trialTerminationReasons = strings(numel(trialTimes_s), 1);

result           = previous;
trialWasSelected = false;
triedCount       = 0;
for trialIndex = 1:numel(trialTimes_s)
    trialGoalState        = suppliedGoalState;
    trialGoalState.time_s = trialTimes_s(trialIndex);
    trialRequest          = outerRequest;
    trialRequest.FixedArrivalTrialTime_s = trialTimes_s(trialIndex);
    triedCount            = trialIndex;
    try
        candidate = planner(previous.PreparedObstacles, initialState, trialGoalState, ...
            previous.RequestedLimits, trialOptions, trialRequest);
    catch exception
        failureIsExpected = string(exception.identifier) == ...
            ["planner:UndefinedTargetDerivative", "planTrajectory:CoincidentEndpoints"];
        if any(failureIsExpected)
            trialTerminationReasons(trialIndex) = string(exception.identifier);
            continue;
        end
        rethrow(exception);
    end
    trialTerminationReasons(trialIndex) = candidate.TerminationReason;
    if candidate.Success
        result           = candidate;
        trialWasSelected = true;
        break;
    end
end

%% Section 5: Record The Honest Chronological Search Outcome

trialTimes_s = trialTimes_s(1:triedCount);
result.TemporalSearch = struct( ...
    'Resolution_s',            resolution_s, ...
    'TrialTime_s',             trialTimes_s, ...
    'TrialTerminationReason',  trialTerminationReasons(1:triedCount), ...
    'GlobalEarliestProven',    false, ...
    'NecessaryArrivalBound_s', earliestTime_s, ...
    'IncumbentArrival_s',      incumbentArrivalTime_s, ...
    'RetainedIncumbent',       previous.Success && ~trialWasSelected, ...
    'PriorTerminationReason',  previous.TerminationReason);
if trialWasSelected
    result.Message = "A chronological fixed-arrival trial passed independent validation; earlier gaps remain unsearched.";
elseif ~result.Success
    result.TerminationReason = "arrivalSearchExhausted";
    result.Message = "No declared arrival trial was certified. Unsearched times and solver failures do not prove infeasibility.";
end
result.ElapsedTime_s = previous.ElapsedTime_s + toc(searchTimer);
end
