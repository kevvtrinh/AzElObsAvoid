function result = finalizeCandidate(preparedObstacles, request, visibilityGraph, ...
        priorResult, candidate, diagnostics, declaration)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.finalizeCandidate( ...
%       preparedObstacles, request, visibilityGraph, priorResult, ...
%       candidate, diagnostics, declaration)
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
%       Normalized initial state, goal state, limits, and resolved options,
%       with its context (original obstacles, supplied and requested
%       provenance, and the parent request when this is a child request).
%   - visibilityGraph (scalar struct)
%       Spatial or timed guide associated with the candidate; its
%       Route_units is the route the candidate was seeded from.
%   - priorResult (scalar struct)
%       The planner record so far; its Attempts and ElapsedTime_s carry
%       into the assembled record.
%   - candidate (scalar struct)
%       BMTP motion candidate.
%   - diagnostics (scalar struct)
%       Solver diagnostic record.
%   - declaration (scalar struct)
%       Fields the validator must see beside the candidate (a fixed trial
%       clock or a free goal window), or struct() when there are none.
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

attempts               = priorResult.Attempts;
elapsedTime_s          = priorResult.ElapsedTime_s;
route_units            = visibilityGraph.Route_units;
validationDeclarations = declaration;
% The declared request is the one the result answers: the request itself,
% or, for an accepted child request, the parent request it was planned for.
declaredRequest = request;
parentRequest   = request.context.parentRequest;
if ~isempty(parentRequest) && candidate.Success
    declaredRequest.context.parentRequest      = [];
    declaredRequest.context.suppliedLimits     = parentRequest.SuppliedLimits;
    declaredRequest.context.suppliedGoalState  = parentRequest.SuppliedGoalState;
    declaredRequest.context.requestedGoalState = parentRequest.RequestedGoalState;
    declaredRequest.context.requestedLimits    = parentRequest.RequestedLimits;
    declaredRequest.context.obstacles          = parentRequest.Obstacles;
    declaredRequest.options.WrapX = parentRequest.WrapX;
    declaredRequest.options.WrapY = parentRequest.WrapY;
    % The declared goal keeps the whole effective inner goal except its clock,
    % including the selected target unwrapping and resolved derivatives.
    declaredRequest.goalState.time_s    = parentRequest.GoalTime_s;
    declaredRequest.options.GoalTimeMode = parentRequest.GoalTimeMode;
    % Only an arrival-time trial declares the clock it was asked to meet.
    if ~isnan(parentRequest.FixedArrivalTrialTime_s)
        validationDeclarations.FixedArrivalTrialTime_s = ...
            parentRequest.FixedArrivalTrialTime_s;
    end
end

%% Section 2: Assemble The Complete Candidate Record

result = obstacleAvoidance.planning.createEmptyResult( ...
    preparedObstacles, declaredRequest, visibilityGraph, attempts, elapsedTime_s);

% Every record declares its outcome fields before the candidate fills them,
% so the field order does not depend on which method produced the candidate.
result.MotionLength_units              = Inf;
result.IntegratedSquaredJerk_units2_s5 = Inf;
result.MaximumConstraintViolation      = Inf;
result.OptimizerFeasible               = false;
result.OptimizerIterateUnavailable     = false;
result.AlternativeGuideEligible        = false;
result.FailureStage                    = "notRun";
result.FailureKind                     = "notRun";

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
