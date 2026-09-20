function result = createEmptyResult( ...
        preparedObstacles, request, requestContext, visibilityGraph, attempts, elapsedTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.createEmptyResult( ...
%       preparedObstacles, request, requestContext, visibilityGraph, attempts, elapsedTime_s)
%**************************************************************************
% PURPOSE
%   - Construct the stable planner record from explicit request, geometry,
%     provenance, and orchestration inputs before any candidate is present.
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
%       Current spatial or timed guide record.
%   - attempts (struct array)
%       Planner-level attempt history to retain.
%   - elapsedTime_s (nonnegative scalar)
%       Planner time accumulated before this record is assembled.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Complete empty planner schema. Expected planning failure is recorded
%       with Success = false; invalid input is checked by the public planner.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Assemble The Empty Record

result                              = struct();
result.Success                      = false;
result.Message                      = "Planning has not completed.";
result.TerminationReason            = "notStarted";
result.Inputs                       = struct("obstacles", {requestContext.obstacles}, ...
    "initialState", request.initialState, "goalState", request.goalState);
result.PreparedObstacles            = preparedObstacles;
result.Limits                       = request.limits;
result.Options                      = request.options;
result.VisibilityGraph              = visibilityGraph;
result.Route_units                  = zeros(0, 2);
result.time_s                       = zeros(0, 1);
result.position_units               = zeros(0, 2);
result.velocity_units_s             = zeros(0, 2);
result.acceleration_units_s2        = zeros(0, 2);
result.jerk_units_s3                = zeros(0, 2);
result.Polynomial                   = struct();
result.SeparationProof             = struct();
result.SolverDiagnostics            = struct();
result.Attempts                     = attempts;
result.Validation                   = struct("Passed", false, "Message", "No motion is available.");
result.ArrivalTime_s                = NaN;
result.Intercept                    = struct( ...
    'Time_s',                     NaN, ...
    'TargetPosition_units',       request.goalState.position_units, ...
    'TerminalVelocityPolicy',     "zero");
result.TrajectoryDuration_s         = NaN;
result.ElapsedTime_s                = elapsedTime_s;

result.SuppliedLimits     = requestContext.suppliedLimits;
result.RequestedLimits    = requestContext.requestedLimits;
result.RequestedGoalState = requestContext.requestedGoalState;
result.SuppliedGoalState  = requestContext.suppliedGoalState;
if ~isempty(requestContext.parentRequest)
    result.ParentRequest = requestContext.parentRequest;
end
end
