function result = planWrappedMotion(request)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.planWrappedMotion(request)
%**************************************************************************
% PURPOSE
%   - Plan across wrapped interval ends using copies of the goal and
%     obstacles. An x copy is shifted by whole wrap lengths. Example: on a
%     360-unit axis, start = 350 and goal = 10 can be planned as 350 to 370.
%     A y copy over an end is a pole copy: y is mirrored about that end
%     and x turns by half a turn. Example: on x [0 360], y [-90 90], a goal
%     at (190, 89) has a pole copy at (10, 91).
%   - Try goal copies nearest first. Keep the earliest validated motion for
%     earliestArrival, or the shortest validated motion for fixedArrival.
%     Skip a fixed goal copy when it cannot improve the current result.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Prepared request with the nearest fixed goal copy or an unwrapped
%       target path. Wrapped-axis limits contain the start position +/-
%       maximum speed x available time; obstacles and acceleration limits
%       may make some positions in this range unreachable.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Selected motion checked against the original wrapped request.
%       WrappedGoalCopies records each listed copy's move from the prepared
%       goal, whether it is a pole copy, and whether it was planned or
%       skipped. If all planned copies fail, Success = false.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Copy Obstacles Into Unwrapped Coordinates

totalTimer             = tic;
wrapModes              = [request.options.WrapX, request.options.WrapY];
wrappedIntervals_units = [request.originalInputs.requestedLimits.xInterval_units; ...
    request.originalInputs.requestedLimits.yInterval_units];
planningRange_units = [request.limits.xInterval_units; request.limits.yInterval_units];
obstacleCopies      = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
    request.obstacles, wrappedIntervals_units, wrapModes, planningRange_units, ...
    [request.initialState.time_s, request.goalState.time_s]);

% The copied coordinates no longer jump at the seam. Plan them with wrapping
% disabled, while retaining the original request for final validation.
unwrappedOptions       = request.options;
unwrappedOptions.WrapX = "false";
unwrappedOptions.WrapY = "false";
parentRequest          = obstacleAvoidance.planning.createParentRequest(request);

%% Section 2: List Goal Copies Inside The Planning Range

% Example: on a 360-unit x axis, goal = 10 has copies at -350, 10, and 370.
% Keep the copies inside the planning range, then try the nearest ones first.
% A fixed arrival meets a moving target at the deadline, so its copies come
% from the deadline position. For earliest arrival, use the path's x and y
% bounds from start to deadline. This box is a conservative superset: a
% diagonal target can overlap each axis at different times without entering
% the planning range at one time. The copy is still a real goal copy; its
% arrival trials report if it cannot be reached. Piecewise linear and PCHIP
% paths stay within their sample bounds between samples.
goalPosition_units = request.goalState.position_units;
goalBounds_units   = [goalPosition_units(:), goalPosition_units(:)];
if ~isempty(request.goalState.targetMotion) && request.options.GoalTimeMode == "earliestArrival"
    targetMotion   = request.goalState.targetMotion;
    targetTimes_s  = double(targetMotion.time_s(:));
    windowTimes_s  = unique([request.initialState.time_s; request.goalState.time_s; ...
        targetTimes_s(targetTimes_s > request.initialState.time_s & targetTimes_s < request.goalState.time_s)]);
    windowTimes_s  = windowTimes_s(windowTimes_s >= targetTimes_s(1) & windowTimes_s <= targetTimes_s(end));
    windowTarget_units = obstacleAvoidance.input.targetPositionAtTime(targetMotion, windowTimes_s);
    goalBounds_units   = [min(windowTarget_units, [], 1).', max(windowTarget_units, [], 1).'];
end
goalImages = obstacleAvoidance.input.listWrapImages( ...
    goalBounds_units, wrappedIntervals_units, wrapModes, planningRange_units);
goalCopyTransforms = [goalImages.XOffset_units, goalImages.YScale, goalImages.YOffset_units];
if isempty(goalCopyTransforms)
    % No equivalent endpoint, or target path during the allowed times, enters
    % the maximum-speed range. A motion cannot reach it by the deadline.
    % Return before route search; an identity copy here would claim that the
    % copy stage found a candidate it did not find.
    % Prepare the copied obstacles as a normal motion request does. Invalid
    % geometry still raises its input error, and the failure keeps usable
    % prepared copies for inspection and plotting.
    preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles( ...
        obstacleCopies, [request.initialState.time_s, request.goalState.time_s], true);
    % An interval without a usable geometry model stops the request first,
    % as on the normal path: the endpoint check below queries that geometry.
    failureMessage = obstacleAvoidance.planning.describeUnsupportedInterval( ...
        preparedObstacles, [request.initialState.time_s, request.goalState.time_s]);
    failureReason  = "unsupportedObstacleInterpolation";
    if strlength(failureMessage) == 0
        % The start state and the goal's derivatives are checked as for any
        % request, so a goal velocity above its limit is reported as such and
        % not as a time-window failure. The goal position has no copy to check.
        [endpointsAreFeasible, failureMessage, failureReason] = ...
            obstacleAvoidance.input.validatePlannerEndpoints( ...
            preparedObstacles, request.initialState, request.goalState, request.limits, request.options);
        if endpointsAreFeasible
            failureMessage = "No equivalent goal copy reaches the maximum-speed planning range by the deadline.";
            failureReason  = "timeWindowInfeasible";
        end
    end
    visibilityGraph = struct( ...
        'NodePosition_units',     zeros(0, 2), ...
        'AcceptedNodeIndex',      zeros(0, 2), ...
        'RejectedNodeIndex',      zeros(0, 2), ...
        'Route_units',            zeros(0, 2), ...
        'RouteLength_units',      Inf, ...
        'IsConnected',            false, ...
        'ExpandedCount',          0, ...
        'GraphIsFullyEnumerated', false, ...
        'SearchKind',             "notSearched");
    emptyAttempts = repmat(obstacleAvoidance.planning.createAttemptRecord(0, ""), 0, 1);
    result = obstacleAvoidance.planning.createEmptyResult( ...
        preparedObstacles, request, visibilityGraph, emptyAttempts, toc(totalTimer));
    result.Message           = failureMessage;
    result.TerminationReason = failureReason;
    result.Diagnostics.WrappedGoalCopies = struct( ...
        'GoalOffset_units',           [NaN, NaN], ...
        'GoalIsPoleCopy',             false, ...
        'CandidateOffsets_units',     zeros(0, 2), ...
        'CandidateIsPoleCopy',        false(0, 1), ...
        'CandidatePlanned',           false(0, 1), ...
        'CandidateTerminationReason', strings(0, 1), ...
        'ObstacleCopyCount',          numel(obstacleCopies));
    return
end
% Record each copy as its move from the prepared goal and whether it is a pole copy.
goalCopyPositions_units = [goalPosition_units(1) + goalCopyTransforms(:, 1), ...
    goalCopyTransforms(:, 2) * goalPosition_units(2) + goalCopyTransforms(:, 3)];
goalCopyOffsets_units   = [goalCopyTransforms(:, 1), ...
    (goalCopyTransforms(:, 2) - 1) * goalPosition_units(2) + goalCopyTransforms(:, 3)];
goalCopyIsPoleCopy      = goalCopyTransforms(:, 2) == -1;
directDistance_units    = vecnorm(goalCopyPositions_units - request.initialState.position_units, 2, 2);
[~, goalCopyOrder]      = sortrows([directDistance_units, goalCopyOffsets_units, goalCopyIsPoleCopy]);
goalCopyTransforms      = goalCopyTransforms(goalCopyOrder, :);
goalCopyOffsets_units   = goalCopyOffsets_units(goalCopyOrder, :);
goalCopyIsPoleCopy      = goalCopyIsPoleCopy(goalCopyOrder);
directDistance_units    = directDistance_units(goalCopyOrder);

%% Section 3: Plan Each Goal Copy And Keep The Best Motion

goalCopyCount              = size(goalCopyTransforms, 1);
goalCopyTerminationReasons = strings(goalCopyCount, 1);
goalCopyWasPlanned         = false(goalCopyCount, 1);

result                  = [];
selectedGoalCopyRequest = [];
selectedOffset_units    = [NaN, NaN];
selectedIsPoleCopy      = false;
hasValidMotion          = false;
bestArrivalTime_s       = Inf;
bestMotionLength_units  = Inf;

findEarliestArrival = request.options.GoalTimeMode == "earliestArrival";
goalPositionIsFixed = isempty(request.goalState.targetMotion);
for goalCopyIndex = 1:goalCopyCount
    goalCopyRequest           = request;
    goalCopyRequest.goalState = transformGoal( ...
        request.goalState, goalCopyTransforms(goalCopyIndex, :));
    goalCopyRequest.options = unwrappedOptions;

    % For a fixed goal, compare the best result with this copy's shortest
    % possible distance or earliest possible arrival. Obstacles can only make
    % those values worse. A moving target may be met closer, so try every copy.
    if hasValidMotion && goalPositionIsFixed
        if findEarliestArrival
            earliestPossibleArrival_s = goalCopyRequest.initialState.time_s + ...
                obstacleAvoidance.input.minimumTravelTime( ...
                goalCopyRequest.initialState, goalCopyRequest.goalState, goalCopyRequest.limits);
            copyCannotImproveResult = bestArrivalTime_s <= earliestPossibleArrival_s + ...
                goalCopyRequest.options.ArrivalTimeTolerance_s;
        else
            copyCannotImproveResult = bestMotionLength_units <= directDistance_units(goalCopyIndex);
        end
        if copyCannotImproveResult
            goalCopyTerminationReasons(goalCopyIndex) = "chordBoundNotBetter";
            continue
        end
    end

    % Resolve this copy's endpoint requirements before planning its motion.
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
    % The copy uses unwrapped inputs. Its parent retains the original wrapped
    % inputs so accepting a motion still requires satisfying the user's request.
    % Arrival trials rebuild the goal from suppliedGoalState at their own
    % time, so leave out values that were only matched to the target at the
    % deadline; each trial matches them again at its own arrival time. Keep
    % a value the caller supplied: it stays a constraint at every arrival.
    copySuppliedGoalState = goalCopyRequest.goalState;
    derivativeFieldNames  = ["velocity_units_s", "acceleration_units_s2"];
    callerGoalState       = request.originalInputs.suppliedGoalState;
    for derivativeIndex = find([request.options.MatchTargetVelocity, request.options.MatchTargetAcceleration])
        derivativeFieldName = derivativeFieldNames(derivativeIndex);
        callerSuppliedValue = isfield(callerGoalState, derivativeFieldName) && ...
            ~isempty(callerGoalState.(derivativeFieldName));
        if ~callerSuppliedValue && isfield(copySuppliedGoalState, derivativeFieldName)
            copySuppliedGoalState = rmfield(copySuppliedGoalState, derivativeFieldName);
        end
    end
    goalCopyRequest.obstacles      = obstacleCopies;
    goalCopyRequest.parentRequest  = parentRequest;
    goalCopyRequest.originalInputs = struct( ...
        'suppliedLimits',     goalCopyRequest.limits, ...
        'requestedLimits',    goalCopyRequest.limits, ...
        'suppliedGoalState',  copySuppliedGoalState, ...
        'requestedGoalState', goalCopyRequest.goalState);
    goalCopyWasPlanned(goalCopyIndex) = true;
    goalCopyResult                   = obstacleAvoidance.planning.planMotion(goalCopyRequest);
    goalCopyTerminationReasons(goalCopyIndex) = goalCopyResult.TerminationReason;

    if goalCopyResult.Success
        % Earliest arrival prefers less time, then less distance on an exact
        % tie. Fixed arrival prefers less distance, then an earlier arrival.
        if findEarliestArrival
            copyIsBetter = ~hasValidMotion || goalCopyResult.ArrivalTime_s < bestArrivalTime_s || ...
                (goalCopyResult.ArrivalTime_s == bestArrivalTime_s && ...
                goalCopyResult.MotionLength_units < bestMotionLength_units);
        else
            copyIsBetter = ~hasValidMotion || goalCopyResult.MotionLength_units < bestMotionLength_units || ...
                (goalCopyResult.MotionLength_units == bestMotionLength_units && ...
                goalCopyResult.ArrivalTime_s < bestArrivalTime_s);
        end
        if copyIsBetter
            result                  = goalCopyResult;
            selectedGoalCopyRequest = goalCopyRequest;
            selectedOffset_units    = goalCopyOffsets_units(goalCopyIndex, :);
            selectedIsPoleCopy      = goalCopyIsPoleCopy(goalCopyIndex);
            bestArrivalTime_s       = goalCopyResult.ArrivalTime_s;
            bestMotionLength_units  = goalCopyResult.MotionLength_units;
            hasValidMotion          = true;
        end
    elseif isempty(result)
        % Keep the first failure in case no copy produces valid motion.
        result                  = goalCopyResult;
        selectedGoalCopyRequest = goalCopyRequest;
    end
end

%% Section 4: Return The Selected Result And The Outcome Of Each Copy

if ~hasValidMotion && isfield(result.Diagnostics, 'ParentRequest')
    % Every candidate failed: report the nearest copy's failure against
    % the wrapped request.
    result = assembleFailedCopyResult(selectedGoalCopyRequest, result, parentRequest);
end
result.Diagnostics.WrappedGoalCopies = struct( ...
    'GoalOffset_units',           selectedOffset_units, ...
    'GoalIsPoleCopy',             selectedIsPoleCopy, ...
    'CandidateOffsets_units',     goalCopyOffsets_units, ...
    'CandidateIsPoleCopy',        goalCopyIsPoleCopy, ...
    'CandidatePlanned',           goalCopyWasPlanned, ...
    'CandidateTerminationReason', goalCopyTerminationReasons, ...
    'ObstacleCopyCount',          numel(obstacleCopies));
result.Diagnostics.ElapsedTime_s = toc(totalTimer);
end

%% Section 5: Local Functions

function goalState = transformGoal(goalState, transform)
    % Move the goal, and an unwrapped target with it, to another copy.
    % transform = [x offset, y scale, y offset]: [x y] -> [x + x offset,
    % y scale x y + y offset]. A pole copy (y scale = -1) also reverses the
    % y velocity and acceleration; x velocity and acceleration are unchanged.
    xOffset_units = transform(1);
    yScale        = transform(2);
    yOffset_units = transform(3);
    goalState.position_units = [goalState.position_units(1) + xOffset_units, ...
        yScale * goalState.position_units(2) + yOffset_units];
    goalState.velocity_units_s(2)      = yScale * goalState.velocity_units_s(2);
    goalState.acceleration_units_s2(2) = yScale * goalState.acceleration_units_s2(2);
    if isfield(goalState, 'targetMotion') && ~isempty(goalState.targetMotion)
        targetPosition_units = goalState.targetMotion.position_units;
        goalState.targetMotion.position_units = [targetPosition_units(:, 1) + xOffset_units, ...
            yScale * targetPosition_units(:, 2) + yOffset_units];
    end
end

function goalState = resolveMatchedCopyDerivatives(goalState, options)
    % Recheck target velocity and acceleration after shifting the target path.
    derivativeFieldNames  = ["velocity_units_s", "acceleration_units_s2"];
    matchTargetDerivative = [options.MatchTargetVelocity, options.MatchTargetAcceleration];
    if ~any(matchTargetDerivative)
        return
    end
    [~, targetVelocity_units_s, targetAcceleration_units_s2] = ...
        obstacleAvoidance.input.targetPositionAtTime(goalState.targetMotion, goalState.time_s);
    targetDerivativeValues = [targetVelocity_units_s; targetAcceleration_units_s2];
    for derivativeIndex = find(matchTargetDerivative)
        derivativeFieldName   = derivativeFieldNames(derivativeIndex);
        targetDerivativeValue = targetDerivativeValues(derivativeIndex, :);
        derivativeDifference  = abs(goalState.(derivativeFieldName) - targetDerivativeValue);
        % Do not replace an existing goal value that disagrees with the target.
        if any(derivativeDifference > options.ConstraintTolerance)
            error('planner:ConflictingTargetDerivative', ...
                'Explicit and matched target derivatives conflict.');
        end
        goalState.(derivativeFieldName) = targetDerivativeValue;
    end
end

function result = assembleFailedCopyResult(goalCopyRequest, goalCopyResult, parentRequest)
    % Return the failed copy with the original wrapped inputs so the caller
    % can compare the failure with the request they supplied.
    failureRequest                  = goalCopyRequest;
    failureRequest.goalState        = goalCopyResult.Inputs.goalState;
    failureRequest.goalState.time_s = parentRequest.GoalTime_s;

    failureRequest.options.WrapX        = parentRequest.WrapX;
    failureRequest.options.WrapY        = parentRequest.WrapY;
    failureRequest.options.GoalTimeMode = parentRequest.GoalTimeMode;
    failureRequest.obstacles            = parentRequest.Obstacles;
    failureRequest.parentRequest        = [];

    failureRequest.originalInputs.suppliedLimits     = parentRequest.SuppliedLimits;
    failureRequest.originalInputs.requestedLimits    = parentRequest.RequestedLimits;
    failureRequest.originalInputs.suppliedGoalState  = parentRequest.SuppliedGoalState;
    failureRequest.originalInputs.requestedGoalState = parentRequest.RequestedGoalState;

    result = obstacleAvoidance.planning.createEmptyResult( ...
        goalCopyResult.Diagnostics.PreparedObstacles, failureRequest, ...
        goalCopyResult.Diagnostics.VisibilityGraph, goalCopyResult.Diagnostics.Attempts, goalCopyResult.Diagnostics.ElapsedTime_s);
    % Keep the original request fields just assembled, and retain the failed
    % copy's other results, including its graph, attempts, and failure reason.
    for fieldName = reshape(string(fieldnames(goalCopyResult)), 1, [])
        if ~any(fieldName == ["Inputs", "Options", "Diagnostics"])
            result.(fieldName) = goalCopyResult.(fieldName);
        end
    end
    requestDiagnosticFieldNames = ["Limits", "SuppliedLimits", "RequestedLimits", ...
        "RequestedGoalState", "SuppliedGoalState", "ParentRequest"];
    for fieldName = reshape(string(fieldnames(goalCopyResult.Diagnostics)), 1, [])
        if ~any(fieldName == requestDiagnosticFieldNames)
            result.Diagnostics.(fieldName) = goalCopyResult.Diagnostics.(fieldName);
        end
    end
end
