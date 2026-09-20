function result = finalizeCandidate(preparedObstacles, request, requestContext, ...
        visibilityGraph, attempts, elapsedTime_s, usesTimedResultSchema, ...
        validationDeclarations, candidate, route_units, diagnostics)
%% Section 0: Header & Readme
% SYNTAX
%   result = ...
%       obstacleAvoidance.planning.finalizeCandidate( ...
%       preparedObstacles, request, requestContext, visibilityGraph, attempts, ...
%       elapsedTime_s, usesTimedResultSchema, validationDeclarations, ...
%       candidate, route_units, diagnostics)
%**************************************************************************
% PURPOSE
%   - Assemble one complete planner record from explicit request, geometry,
%     orchestration, and BMTP candidate inputs, then apply the independent
%     validator.
%**************************************************************************
% INPUTS
%   - preparedObstacles (struct array)
%       Prepared obstacle geometry owned by the normalized request.
%   - request (scalar struct)
%       Normalized initial state, goal state, limits, and resolved options.
%   - requestContext (scalar struct)
%       Original obstacles, supplied/requested provenance, and optional
%       outer-request context.
%   - visibilityGraph (scalar struct)
%       Spatial or timed guide associated with the candidate.
%   - attempts (struct array)
%       Planner-level attempt history to retain.
%   - elapsedTime_s (nonnegative scalar)
%       Planner time accumulated before candidate assembly.
%   - usesTimedResultSchema (logical scalar)
%       True when the timed result requires its stable outcome field ordering.
%   - validationDeclarations (scalar struct)
%       Optional fixed-clock or free-window fields needed by validation.
%   - candidate (scalar struct)
%       BMTP motion candidate.
%   - route_units (N-by-2 numeric array)
%       Selected route positions.
%   - diagnostics (scalar struct)
%       Solver diagnostic record.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Planner result after the uniform candidate-acceptance decision.
%       Motion that fails validation returns Success = false with a
%       diagnostic Message and TerminationReason; invalid input throws.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Choose The Acceptance Declaration

acceptanceDeclaration = struct( ...
    'request',        request, ...
    'requestContext', requestContext);
parentRequest = requestContext.parentRequest;
if ~isempty(parentRequest) && candidate.Success
    acceptanceDeclaration.requestContext.parentRequest = [];
    acceptanceDeclaration.requestContext.suppliedLimits = parentRequest.SuppliedLimits;
    acceptanceDeclaration.requestContext.suppliedGoalState = parentRequest.SuppliedGoalState;
    acceptanceDeclaration.requestContext.requestedGoalState = parentRequest.RequestedGoalState;
    acceptanceDeclaration.requestContext.requestedLimits = parentRequest.RequestedLimits;
    acceptanceDeclaration.requestContext.obstacles = parentRequest.Obstacles;
    acceptanceDeclaration.request.options.WrapX = parentRequest.WrapX;
    acceptanceDeclaration.request.options.WrapY = parentRequest.WrapY;
    % The declared goal keeps the whole effective inner goal except its clock,
    % including the selected target unwrapping and resolved derivatives.
    acceptanceDeclaration.request.goalState.time_s = parentRequest.GoalTime_s;
    acceptanceDeclaration.request.options.GoalTimeMode = parentRequest.GoalTimeMode;
    % Only an arrival-time trial declares the clock it was asked to meet.
    if ~isnan(parentRequest.FixedArrivalTrialTime_s)
        validationDeclarations.FixedArrivalTrialTime_s = ...
            parentRequest.FixedArrivalTrialTime_s;
    end
end

%% Section 2: Assemble The Complete Candidate Record

result = obstacleAvoidance.planning.createEmptyResult( ...
    preparedObstacles, acceptanceDeclaration.request, ...
    acceptanceDeclaration.requestContext, visibilityGraph, attempts, elapsedTime_s);

% Timed planning declares these stable outcome fields before a BMTP candidate
% exists. Preserve that ordering when assembling the candidate record.
if usesTimedResultSchema
    result.MotionLength_units                  = Inf;
    result.IntegratedSquaredJerk_units2_s5     = Inf;
    result.MaximumConstraintViolation          = Inf;
    result.OptimizerFeasible                    = false;
    result.OptimizerIterateUnavailable          = false;
    result.AlternativeGuideEligible             = false;
    result.FailureStage                         = "notRun";
    result.FailureKind                          = "notRun";
end

for fieldName = reshape(string(fieldnames(candidate)), 1, [])
    result.(fieldName) = candidate.(fieldName);
end
result.Route_units       = route_units;
result.SolverDiagnostics = diagnostics;

for fieldName = reshape(string(fieldnames(validationDeclarations)), 1, [])
    result.(fieldName) = validationDeclarations.(fieldName);
end

goalState = request.goalState;
if candidate.Success && ~isempty(goalState.targetMotion)
    targetMotion  = goalState.targetMotion;
    arrivalTime_s = candidate.ArrivalTime_s;
    % Target derivatives are evaluated only when a matching option needs
    % them: a position-only intercept at a linear target corner is valid.
    matchesDerivative = request.options.MatchTargetVelocity || ...
        request.options.MatchTargetAcceleration;
    if matchesDerivative
        [tgtPosition_units, tgtVel_units_s, tgtAcc_units_s2] = ...
            obstacleAvoidance.input.targetPositionAtTime(targetMotion, arrivalTime_s);
    else
        tgtPosition_units = obstacleAvoidance.input.targetPositionAtTime(targetMotion, arrivalTime_s);
    end
    result.Intercept = struct( ...
        'Time_s',                     arrivalTime_s, ...
        'TargetPosition_units',       tgtPosition_units, ...
        'TerminalVelocityPolicy',     "explicit");
    if all(candidate.velocity_units_s(end, :) == 0)
        result.Intercept.TerminalVelocityPolicy = "zero";
    end
    if request.options.MatchTargetVelocity
        result.Intercept.TerminalVelocityPolicy = "matched";
        result.Intercept.TargetVelocity_units_s = tgtVel_units_s;
    end
    if request.options.MatchTargetAcceleration
        result.Intercept.TargetAcceleration_units_s2 = tgtAcc_units_s2;
    end
end

%% Section 3: Apply The One Public Acceptance Gate

result.Validation = obstacleAvoidance.validateTrajectory(result);
result.Success    = candidate.Success && result.Validation.Passed;

if candidate.Success && ~result.Validation.Passed
    result.Message = "BMTP returned motion that failed independent validation: " + ...
        result.Validation.Message;
    result.TerminationReason = "invalidMotion";
end
end
