function result = planWrappedMotion(request)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.planWrappedMotion( ...
%       request)
%**************************************************************************
% PURPOSE
%   - A wrapped axis (azimuth 359 meets 0) is planned in plain unwrapped
%     coordinates. Obstacles are copied one full turn up and down, and the
%     goal has several copies, one per whole turn, inside the range the
%     vehicle can reach in time. Each goal copy is one ordinary request.
%   - Goal copies are planned nearest first and each result is accepted
%     against the wrapped request the user made. The answer is the earliest
%     valid copy (earliestArrival) or the shortest (fixedArrival); a fixed
%     goal copy whose straight-line distance cannot beat the best plan so
%     far is not planned at all.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Normalized wrapped request. Its goal holds the nearest goal copy or
%       the unwrapped target path, and its wrapped intervals are the range
%       the vehicle can reach.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Planner record accepted against the wrapped request, with a
%       WrappedGoalCopies field recording every goal copy tried, its
%       whole-turn shift, and its outcome. Failure of every copy returns
%       Success = false.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Build The Unwrapped Scene: Obstacle Copies And Plain Options

timer           = tic;
wrapAxes        = [request.options.WrapX, request.options.WrapY];
intervals_units = [request.context.requestedLimits.xInterval_units; ...
    request.context.requestedLimits.yInterval_units];
reachableRange_units      = [request.limits.xInterval_units; request.limits.yInterval_units];
period_units    = diff(intervals_units, 1, 2).';
obstacleCopies          = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
    request.context.obstacles, intervals_units, wrapAxes, reachableRange_units);

unwrappedOptions       = request.options;
unwrappedOptions.WrapX = false;
unwrappedOptions.WrapY = false;
parentRequest = obstacleAvoidance.planning.createParentRequest(request);

%% Section 2: List Every Goal Copy In The Reachable Range, Nearest First

offsetsByAxis = {0, 0};
for axisIndex = find(wrapAxes)
    lowestCopy = ceil((reachableRange_units(axisIndex, 1) - ...
        request.goalState.position_units(axisIndex)) / period_units(axisIndex));
    highestCopy = floor((reachableRange_units(axisIndex, 2) - ...
        request.goalState.position_units(axisIndex)) / period_units(axisIndex));
    offsetsByAxis{axisIndex} = (lowestCopy:highestCopy) * period_units(axisIndex);
end
[dx_units, dy_units] = ndgrid(offsetsByAxis{1}, offsetsByAxis{2});
offsets_units = [dx_units(:), dy_units(:)];
if isempty(offsets_units)
    % No copy is reachable inside the reachable range; plan the nearest copy so the
    % ordinary request reports the honest failure.
    offsets_units = [0, 0];
end
chord_units = vecnorm(request.goalState.position_units + offsets_units - ...
    request.initialState.position_units, 2, 2);
[~, order]  = sortrows([chord_units, offsets_units]);
offsets_units = offsets_units(order, :);
chord_units   = chord_units(order);

%% Section 3: Plan Each Candidate As A Plain Request And Keep The Best

candidateCount   = size(offsets_units, 1);
reasons          = strings(candidateCount, 1);
wasPlanned       = false(candidateCount, 1);
result           = [];
bestKey          = [];
bestOffset_units = [NaN, NaN];
selectedGoalCopyRequest = [];
isEarliest       = request.options.GoalTimeMode == "earliestArrival";
% The chord to a fixed goal bounds motion length and travel time from below;
% a moving target can be met earlier and nearer, so it is never pruned.
canPrune = isempty(request.goalState.targetMotion);
for candidateIndex = 1:candidateCount
    goalCopyRequest           = request;
    goalCopyRequest.goalState = shiftGoal( ...
        request.goalState, offsets_units(candidateIndex, :));
    goalCopyRequest.options   = unwrappedOptions;
    if ~isempty(bestKey) && canPrune
        bound = chord_units(candidateIndex);
        slack = 0;
        if isEarliest
            bound = goalCopyRequest.initialState.time_s + ...
                obstacleAvoidance.input.minimumTravelTime( ...
                goalCopyRequest.initialState, goalCopyRequest.goalState, goalCopyRequest.limits);
            slack = goalCopyRequest.options.ArrivalTimeTolerance_s;
        end
        if bestKey(1) <= bound + slack
            reasons(candidateIndex) = "chordBoundNotBetter";
            continue
        end
    end
    goalCopyRequest.goalState = resolveMatchedCopyDerivatives( ...
        goalCopyRequest.goalState, request.options);
    goalIsRequiredEndpoint = isempty(goalCopyRequest.goalState.targetMotion) || ...
        goalCopyRequest.options.GoalTimeMode == "fixedArrival";
    endpointsCoincide = norm(goalCopyRequest.goalState.position_units - ...
        goalCopyRequest.initialState.position_units) <= goalCopyRequest.options.ConstraintTolerance;
    if goalIsRequiredEndpoint && endpointsCoincide
        error("planTrajectory:CoincidentEndpoints", ...
            "Initial and goal positions must be distinct.");
    end
    goalCopyRequest.context = struct( ...
        'obstacles',          {obstacleCopies}, ...
        'suppliedLimits',     goalCopyRequest.limits, ...
        'requestedLimits',    goalCopyRequest.limits, ...
        'suppliedGoalState',  goalCopyRequest.goalState, ...
        'requestedGoalState', goalCopyRequest.goalState, ...
        'parentRequest',      parentRequest);
    wasPlanned(candidateIndex) = true;
    candidate = obstacleAvoidance.planning.planMotion(goalCopyRequest);
    reasons(candidateIndex) = candidate.TerminationReason;
    if candidate.Success
        if isEarliest
            key = [candidate.ArrivalTime_s, candidate.MotionLength_units];
        else
            key = [candidate.MotionLength_units, candidate.ArrivalTime_s];
        end
        isBetter = isempty(bestKey) || key(1) < bestKey(1) || (key(1) == bestKey(1) && key(2) < bestKey(2));
        if isBetter
            result                  = candidate;
            selectedGoalCopyRequest = goalCopyRequest;
            bestKey                 = key;
            bestOffset_units        = offsets_units(candidateIndex, :);
        end
    elseif isempty(result)
        result                  = candidate;
        selectedGoalCopyRequest = goalCopyRequest;
    end
end

%% Section 4: Record The Candidates On The Returned Result

if isempty(bestKey) && isfield(result, 'ParentRequest')
    % Every candidate failed: report the nearest copy's failure against
    % the wrapped request.
    result = assembleFailedCopyResult(selectedGoalCopyRequest, result, parentRequest);
end
result.WrappedGoalCopies = struct( ...
    'GoalOffset_units',           bestOffset_units, ...
    'CandidateOffsets_units',     offsets_units, ...
    'CandidatePlanned',           wasPlanned, ...
    'CandidateTerminationReason', reasons, ...
    'ObstacleCopyCount',         numel(obstacleCopies));
result.ElapsedTime_s = toc(timer);
end

%% Section 5: Local Functions

function goalState = shiftGoal(goalState, offset_units)
    % Move the goal, and an unwrapped target with it, to another copy.
    goalState.position_units = goalState.position_units + offset_units;
    if isfield(goalState, 'targetMotion') && ~isempty(goalState.targetMotion)
        goalState.targetMotion.position_units = goalState.targetMotion.position_units + offset_units;
    end
end

function goalState = resolveMatchedCopyDerivatives(goalState, options)
    % Recheck derivatives after target unwrapping creates the copied goal.
    derivativeNames     = ["velocity_units_s", "acceleration_units_s2"];
    matches             = [options.MatchTargetVelocity, options.MatchTargetAcceleration];
    if ~any(matches)
        return
    end
    [~, tgtVel, tgtAcc] = obstacleAvoidance.input.targetPositionAtTime( ...
        goalState.targetMotion, goalState.time_s);
    derivatives         = [tgtVel; tgtAcc];
    for derivativeIndex = find(matches)
        derivativeName     = derivativeNames(derivativeIndex);
        targetDerivative   = derivatives(derivativeIndex, :);
        derivativeResidual = abs(goalState.(derivativeName) - targetDerivative);
        if any(derivativeResidual > options.ConstraintTolerance)
            error('planner:ConflictingTargetDerivative', ...
                'Explicit and matched target derivatives conflict.');
        end
        goalState.(derivativeName) = targetDerivative;
    end
end

function result = assembleFailedCopyResult(goalCopyRequest, goalCopyResult, parentRequest)
    % Compose the retained copy failure with the wrapped declaration.
    declarationRequest           = goalCopyRequest;
    declarationRequest.goalState = goalCopyResult.Inputs.goalState;
    declarationRequest.goalState.time_s = parentRequest.GoalTime_s;
    declarationRequest.options.WrapX = parentRequest.WrapX;
    declarationRequest.options.WrapY = parentRequest.WrapY;
    declarationRequest.options.GoalTimeMode = parentRequest.GoalTimeMode;
    declarationRequest.context.obstacles          = parentRequest.Obstacles;
    declarationRequest.context.suppliedLimits     = parentRequest.SuppliedLimits;
    declarationRequest.context.requestedLimits    = parentRequest.RequestedLimits;
    declarationRequest.context.suppliedGoalState  = parentRequest.SuppliedGoalState;
    declarationRequest.context.requestedGoalState = parentRequest.RequestedGoalState;
    declarationRequest.context.parentRequest      = [];

    result = obstacleAvoidance.planning.createEmptyResult( ...
        goalCopyResult.PreparedObstacles, declarationRequest, ...
        goalCopyResult.VisibilityGraph, goalCopyResult.Attempts, goalCopyResult.ElapsedTime_s);
    declarationFieldNames = ["Inputs", "Limits", "Options", ...
        "SuppliedLimits", "RequestedLimits", "RequestedGoalState", ...
        "SuppliedGoalState", "ParentRequest"];
    for fieldName = reshape(string(fieldnames(goalCopyResult)), 1, [])
        if ~any(fieldName == declarationFieldNames)
            result.(fieldName) = goalCopyResult.(fieldName);
        end
    end
end
