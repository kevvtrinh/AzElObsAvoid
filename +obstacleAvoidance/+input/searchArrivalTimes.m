function result = searchArrivalTimes(previous)
%% Section 0: Header & Readme
% SYNTAX: result = obstacleAvoidance.input.searchArrivalTimes(previous)
% PURPOSE: Search declared chronological fixed-arrival trials without assuming
%   monotone feasibility or inserting a stationary wait at the initial state.
% INPUTS: Public result containing the original request and any valid incumbent.
% OUTPUTS: Independently valid candidate or honest exhausted-search outcome;
%   TemporalSearch records trial times, results, and unsearched intervals.
% UNITS: Coordinate units and seconds.

%% Section 1: Enumerate Physical Times And Source Boundaries
timer = tic;
initial = previous.Inputs.initialState;
goal = previous.SuppliedGoalState;
options = previous.Options;
start_s = initial.time_s; horizon_s = previous.Inputs.goalState.time_s;
resolution_s = options.TemporalResolution_s;
boundaries_s = horizon_s;
for k = 1:numel(previous.PreparedObstacles)
    boundaries_s = [boundaries_s;previous.PreparedObstacles(k).time_s(:)]; %#ok<AGROW>
end
if isfield(goal,'targetMotion') && ~isempty(goal.targetMotion)
    boundaries_s = [boundaries_s;goal.targetMotion.time_s(:)];
    start_s = max(start_s,goal.targetMotion.time_s(1));
    horizon_s = min(horizon_s,goal.targetMotion.time_s(end));
end
earliest_s = start_s;
if isempty(previous.Inputs.goalState.targetMotion)
    earliest_s = max(earliest_s,initial.time_s+obstacleAvoidance.input.minimumTravelTime( ...
        initial,previous.Inputs.goalState,previous.Limits));
end
incumbentArrival_s = NaN;
if previous.Success
    incumbentArrival_s = previous.ArrivalTime_s;
    horizon_s = min(horizon_s,incumbentArrival_s-options.ArrivalTimeTolerance_s);
end
% Keep the original grid origin, but apply the physical bound before the
% trial budget. Impossible early times must not exclude later feasible ones.
firstStep = max(1,ceil((earliest_s-start_s-options.ArrivalTimeTolerance_s)/resolution_s));
lastStep = min(firstStep+options.MaxArrivalTrials-1,floor((horizon_s-start_s)/resolution_s));
times_s = unique([start_s+(firstStep:lastStep)'*resolution_s;boundaries_s]);
times_s = times_s(times_s>initial.time_s & times_s>=earliest_s-options.ArrivalTimeTolerance_s & times_s<=horizon_s);
times_s = times_s(1:min(numel(times_s),options.MaxArrivalTrials));
options.GoalTimeMode = "fixedArrival";
reasons = strings(numel(times_s),1);
result = previous;
selectedTrial = false;
tried = 0;

%% Section 2: Solve Each Candidate Independently On Its Actual Clock
for k = 1:numel(times_s)
    trialGoal = goal; trialGoal.time_s = times_s(k);
    tried = k;
    try
        candidate = planner(previous.PreparedObstacles,initial,trialGoal,previous.RequestedLimits,options);
        candidate.Inputs.obstacles = previous.Inputs.obstacles;
    catch exception
        if any(string(exception.identifier)==["planner:UndefinedTargetDerivative","planTrajectory:CoincidentEndpoints"])
            reasons(k) = string(exception.identifier);
            continue;
        end
        rethrow(exception);
    end
    reasons(k) = candidate.TerminationReason;
    if candidate.Success
        result = candidate;
        selectedTrial = true;
        break;
    end
end
times_s = times_s(1:tried);
searchedEnd_s = min(earliest_s,horizon_s);
if tried>0, searchedEnd_s = times_s(end); end
breaks_s = unique([min(earliest_s,horizon_s);times_s;horizon_s]);
result.TemporalSearch = struct('Resolution_s',resolution_s,'Budget',options.MaxArrivalTrials, ...
    'TrialTime_s',times_s,'TrialTerminationReason',reasons(1:tried), ...
    'UnsearchedOpenIntervals_s',[breaks_s(1:end-1),breaks_s(2:end)], ...
    'UnsearchedTail_s',[searchedEnd_s,horizon_s],'GlobalEarliestProven',false, ...
    'NecessaryArrivalBound_s',earliest_s,'IncumbentArrival_s',incumbentArrival_s, ...
    'RetainedIncumbent',previous.Success && ~selectedTrial, ...
    'PriorTerminationReason',previous.TerminationReason);
if selectedTrial
    result.SuppliedLimits = previous.SuppliedLimits;
    result.SuppliedGoalState = previous.SuppliedGoalState;
    result.RequestedGoalState = previous.RequestedGoalState;
    result.FixedArrivalTrialTime_s = result.ArrivalTime_s;
    result.Inputs.goalState.time_s = previous.Inputs.goalState.time_s;
    result.Options.GoalTimeMode = "earliestArrival";
    result.Inputs.options = result.Options;
    result.Message = "A chronological fixed-arrival trial passed independent validation; earlier gaps remain unsearched.";
    result.Validation = obstacleAvoidance.validateTrajectory(result);
    result.Success = result.Validation.Passed;
elseif ~result.Success
    result.TerminationReason = "arrivalSearchExhausted";
    result.Message = "No declared arrival trial was certified. Unsearched times and solver failures do not prove infeasibility.";
end
result.ElapsedTime_s = previous.ElapsedTime_s+toc(timer);
end
