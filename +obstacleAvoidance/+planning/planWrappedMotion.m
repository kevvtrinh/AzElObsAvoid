function result = planWrappedMotion(request)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.planWrappedMotion(request)
%**************************************************************************
% PURPOSE
%   - Plan across a coordinate seam by shifting copies of the goal and
%     obstacles by whole wrap lengths. Example: on a 360-unit axis,
%     start = 350 and goal = 10 can be planned as 350 to 370.
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
%       WrappedGoalCopies records each listed copy's shift and whether it
%       was planned or skipped. If all planned copies fail, Success = false.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Copy Obstacles Into Unwrapped Coordinates

totalTimer             = tic;
wrapAxes               = [request.options.WrapX, request.options.WrapY];
wrappedIntervals_units = [request.originalInputs.requestedLimits.xInterval_units; ...
    request.originalInputs.requestedLimits.yInterval_units];
planningRange_units = [request.limits.xInterval_units; request.limits.yInterval_units];
wrapLength_units    = diff(wrappedIntervals_units, 1, 2).';
obstacleCopies      = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
    request.obstacles, wrappedIntervals_units, wrapAxes, planningRange_units);

% The copied coordinates no longer jump at the seam. Plan them with wrapping
% disabled, while retaining the original request for final validation.
unwrappedOptions       = request.options;
unwrappedOptions.WrapX = false;
unwrappedOptions.WrapY = false;
parentRequest          = obstacleAvoidance.planning.createParentRequest(request);

%% Section 2: List Goal Copies Inside The Planning Range

% Example: on a 360-unit axis, goal = 10 has copies at -350, 10, and 370.
% Keep the copies inside the planning range, then try the nearest ones first.
goalCopyOffsetsByAxis = {0, 0};
for axisIndex = find(wrapAxes)
    minimumWrapCount = ceil((planningRange_units(axisIndex, 1) - ...
        request.goalState.position_units(axisIndex)) / wrapLength_units(axisIndex));
    maximumWrapCount = floor((planningRange_units(axisIndex, 2) - ...
        request.goalState.position_units(axisIndex)) / wrapLength_units(axisIndex));
    goalCopyOffsetsByAxis{axisIndex} = (minimumWrapCount:maximumWrapCount) * wrapLength_units(axisIndex);
end
% If both axes wrap, include every combination of x and y shifts.
[xOffsets_units, yOffsets_units] = ndgrid(goalCopyOffsetsByAxis{1}, goalCopyOffsetsByAxis{2});
goalCopyOffsets_units = [xOffsets_units(:), yOffsets_units(:)];
if isempty(goalCopyOffsets_units)
    % Keep the prepared goal when no listed copy lies in the range. The normal
    % planning checks will determine whether this request can be satisfied.
    goalCopyOffsets_units = [0, 0];
end
directDistance_units = vecnorm(request.goalState.position_units + goalCopyOffsets_units - ...
    request.initialState.position_units, 2, 2);
[~, goalCopyOrder]     = sortrows([directDistance_units, goalCopyOffsets_units]);
goalCopyOffsets_units = goalCopyOffsets_units(goalCopyOrder, :);
directDistance_units  = directDistance_units(goalCopyOrder);

%% Section 3: Plan Each Goal Copy And Keep The Best Motion

goalCopyCount              = size(goalCopyOffsets_units, 1);
goalCopyTerminationReasons = strings(goalCopyCount, 1);
goalCopyWasPlanned         = false(goalCopyCount, 1);

result                  = [];
selectedGoalCopyRequest = [];
selectedOffset_units    = [NaN, NaN];
hasValidMotion          = false;
bestArrivalTime_s       = Inf;
bestMotionLength_units  = Inf;

findEarliestArrival = request.options.GoalTimeMode == "earliestArrival";
goalPositionIsFixed = isempty(request.goalState.targetMotion);
for goalCopyIndex = 1:goalCopyCount
    goalCopyRequest           = request;
    goalCopyRequest.goalState = shiftGoal( ...
        request.goalState, goalCopyOffsets_units(goalCopyIndex, :));
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
    goalCopyRequest.obstacles      = obstacleCopies;
    goalCopyRequest.parentRequest  = parentRequest;
    goalCopyRequest.originalInputs = struct( ...
        'suppliedLimits',     goalCopyRequest.limits, ...
        'requestedLimits',    goalCopyRequest.limits, ...
        'suppliedGoalState',  goalCopyRequest.goalState, ...
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

if ~hasValidMotion && isfield(result, 'ParentRequest')
    % Every candidate failed: report the nearest copy's failure against
    % the wrapped request.
    result = assembleFailedCopyResult(selectedGoalCopyRequest, result, parentRequest);
end
result.WrappedGoalCopies = struct( ...
    'GoalOffset_units',           selectedOffset_units, ...
    'CandidateOffsets_units',     goalCopyOffsets_units, ...
    'CandidatePlanned',           goalCopyWasPlanned, ...
    'CandidateTerminationReason', goalCopyTerminationReasons, ...
    'ObstacleCopyCount',          numel(obstacleCopies));
result.ElapsedTime_s = toc(totalTimer);
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
        goalCopyResult.PreparedObstacles, failureRequest, ...
        goalCopyResult.VisibilityGraph, goalCopyResult.Attempts, goalCopyResult.ElapsedTime_s);
    % Keep the original request fields just assembled, and retain the failed
    % copy's other results, including its graph, attempts, and failure reason.
    requestFieldNames = ["Inputs", "Limits", "Options", ...
        "SuppliedLimits", "RequestedLimits", "RequestedGoalState", ...
        "SuppliedGoalState", "ParentRequest"];
    for fieldName = reshape(string(fieldnames(goalCopyResult)), 1, [])
        if ~any(fieldName == requestFieldNames)
            result.(fieldName) = goalCopyResult.(fieldName);
        end
    end
end
