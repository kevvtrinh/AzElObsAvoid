function result = finalizeCandidate(preparedObstacles, request, visibilityGraph, ...
        priorResult, motionCandidate, solverDiagnostics, additionalValidationFields)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.finalizeCandidate( ...
%       preparedObstacles, request, visibilityGraph, priorResult, ...
%       motionCandidate, solverDiagnostics, additionalValidationFields)
%**************************************************************************
% PURPOSE
%   - Combine the request, route, planning attempts, and BMTP motion into
%     one result, then check it with the independent validator.
%**************************************************************************
% INPUTS
%   - preparedObstacles (struct array)
%       Prepared obstacle geometry owned by the normalized request.
%   - request (scalar struct)
%       Normalized initial state, goal state, limits, and resolved options,
%       plus the original inputs and the parent request, when this
%       motion was planned as one trial of a larger request.
%   - visibilityGraph (scalar struct)
%       Graph used to create the motion's starting route. Route_units holds
%       that route, which may differ from the final optimized motion.
%   - priorResult (scalar struct)
%       The planner record so far; its Attempts and ElapsedTime_s carry
%       into the assembled record.
%   - motionCandidate (scalar struct)
%       BMTP output, including motion data when the engine succeeds.
%   - solverDiagnostics (scalar struct)
%       Solver measurements and failure details.
%   - additionalValidationFields (scalar struct)
%       Extra timing requirements for validation: a required arrival time
%       or an allowed arrival range. Use struct() when neither is needed.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Motion, planning details, and independent validation status.
%       Motion that fails validation returns Success = false with a
%       diagnostic Message and TerminationReason; invalid input throws.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Choose The Request Used For Validation

attempts               = priorResult.Attempts;
elapsedTime_s          = priorResult.ElapsedTime_s;
route_units            = visibilityGraph.Route_units;
validationTimingFields = additionalValidationFields;

% A trial may use a different arrival time or a shifted goal copy.
% If BMTP found motion, check it against the original request as well.
validationRequest = request;
parentRequest     = request.parentRequest;
if ~isempty(parentRequest) && motionCandidate.Success
    validationRequest.parentRequest = [];

    validationRequest.originalInputs.suppliedLimits     = parentRequest.SuppliedLimits;
    validationRequest.originalInputs.suppliedGoalState  = parentRequest.SuppliedGoalState;
    validationRequest.originalInputs.requestedGoalState = parentRequest.RequestedGoalState;
    validationRequest.originalInputs.requestedLimits    = parentRequest.RequestedLimits;

    validationRequest.obstacles     = parentRequest.Obstacles;
    validationRequest.options.WrapX = parentRequest.WrapX;
    validationRequest.options.WrapY = parentRequest.WrapY;

    % Keep the selected goal position, velocity, and acceleration. Restore
    % the original goal time so validation also checks the requested deadline.
    validationRequest.goalState.time_s     = parentRequest.GoalTime_s;
    validationRequest.options.GoalTimeMode = parentRequest.GoalTimeMode;

    % Also retain the exact arrival time required by this trial. For example,
    % a 12 s trial under a 20 s deadline must arrive at 12 s and no later than 20 s.
    if ~isnan(parentRequest.FixedArrivalTrialTime_s)
        validationTimingFields.FixedArrivalTrialTime_s = ...
            parentRequest.FixedArrivalTrialTime_s;
    end
end

%% Section 2: Combine The Motion And Planning Details

result = obstacleAvoidance.planning.createEmptyResult( ...
    preparedObstacles, validationRequest, visibilityGraph, attempts, elapsedTime_s);

% Initialize these fields in a fixed order, then copy the BMTP values.
% Every planning method returns the same field order, including on failure.
result.MotionLength_units              = Inf;
result.IntegratedSquaredJerk_units2_s5 = Inf;
result.MaximumConstraintViolation      = Inf;
result.OptimizerFeasible               = false;
result.OptimizerIterateUnavailable     = false;
result.AlternativeGuideEligible        = false;
result.FailureStage                    = "notRun";
result.FailureKind                     = "notRun";

for fieldName = reshape(string(fieldnames(motionCandidate)), 1, [])
    result.(fieldName) = motionCandidate.(fieldName);
end
result.Route_units       = route_units;
result.SolverDiagnostics = solverDiagnostics;

for fieldName = reshape(string(fieldnames(validationTimingFields)), 1, [])
    result.(fieldName) = validationTimingFields.(fieldName);
end

goalState = request.goalState;
if motionCandidate.Success && ~isempty(goalState.targetMotion)
    % Record where the target is at the actual arrival time so the caller
    % can see the meeting point, including any requested velocity matching.
    targetMotion  = goalState.targetMotion;
    arrivalTime_s = motionCandidate.ArrivalTime_s;
    % Calculate target velocity and acceleration only when matching them.
    % Position alone is still defined where a piecewise-linear path turns.
    targetDerivativeIsNeeded = request.options.MatchTargetVelocity || ...
        request.options.MatchTargetAcceleration;
    if targetDerivativeIsNeeded
        [targetPosition_units, targetVelocity_units_s, targetAcceleration_units_s2] = ...
            obstacleAvoidance.input.targetPositionAtTime(targetMotion, arrivalTime_s);
    else
        targetPosition_units = obstacleAvoidance.input.targetPositionAtTime(targetMotion, arrivalTime_s);
    end
    result.Intercept = struct( ...
        'Time_s',                 arrivalTime_s, ...
        'TargetPosition_units',   targetPosition_units, ...
        'TerminalVelocityPolicy', "explicit");
    if all(motionCandidate.velocity_units_s(end, :) == 0)
        result.Intercept.TerminalVelocityPolicy = "zero";
    end
    if request.options.MatchTargetVelocity
        result.Intercept.TerminalVelocityPolicy = "matched";
        result.Intercept.TargetVelocity_units_s = targetVelocity_units_s;
    end
    if request.options.MatchTargetAcceleration
        result.Intercept.TargetAcceleration_units_s2 = targetAcceleration_units_s2;
    end
end

%% Section 3: Independently Validate The Returned Motion

% Engine success alone is not planner success. The independent check must
% confirm the returned motion satisfies the request and avoids obstacles.
result.Validation = obstacleAvoidance.validateTrajectory(result);
result.Success    = motionCandidate.Success && result.Validation.Passed;

if motionCandidate.Success && ~result.Validation.Passed
    result.Message = "BMTP returned motion that failed independent validation: " + ...
        result.Validation.Message;
    result.TerminationReason = "invalidMotion";
end
end
