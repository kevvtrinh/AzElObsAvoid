function result = createEmptyResult( ...
        preparedObstacles, request, visibilityGraph, attempts, elapsedTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.createEmptyResult( ...
%       preparedObstacles, request, visibilityGraph, attempts, elapsedTime_s)
%**************************************************************************
% PURPOSE
%   - Create the standard result fields before motion is available,
%     retaining the request, prepared obstacles, route, and completed attempts.
%**************************************************************************
% INPUTS
%   - preparedObstacles (struct array)
%       Obstacle geometry prepared for this request.
%   - request (scalar struct)
%       Normalized initial state, goal state, limits, and resolved options.
%       Includes the original inputs and any parent request.
%   - visibilityGraph (scalar struct)
%       Connections and route found so far, including times for a timed route.
%   - attempts (struct array)
%       Planner-level attempt history to retain.
%   - elapsedTime_s (nonnegative scalar)
%       Planner time accumulated before this record is assembled.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Standard result with empty motion arrays. Expected failure is recorded
%       with Success = false; invalid input is checked by the public planner.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Assemble The Empty Record

% Use the same base fields for every planning method, including an early
% failure. Success stays false until motion passes independent validation.
result                   = struct();
result.Success           = false;
result.Message           = "Planning has not completed.";
result.TerminationReason = "notStarted";

result.Inputs = struct( ...
    "obstacles",    {request.obstacles}, ...
    "initialState", request.initialState, ...
    "goalState",    request.goalState);
result.PreparedObstacles = preparedObstacles;
result.Limits            = request.limits;
result.Options           = request.options;
result.VisibilityGraph   = visibilityGraph;

% Keep empty position and derivative arrays in [x y] column layout.
% Later stages fill the starting route and the actual motion separately.
result.Route_units           = zeros(0, 2);
result.time_s                = zeros(0, 1);
result.position_units        = zeros(0, 2);
result.velocity_units_s      = zeros(0, 2);
result.acceleration_units_s2 = zeros(0, 2);
result.jerk_units_s3         = zeros(0, 2);

result.Polynomial        = struct();
result.SeparationProof   = struct();
result.SolverDiagnostics = struct();
result.Attempts          = attempts;
result.Validation       = struct("Passed", false, "Message", "No motion is available.");

% NaN means no arrival has been calculated. The intercept fields are
% placeholders until a successful motion supplies the actual meeting point.
result.ArrivalTime_s = NaN;
result.Intercept     = struct( ...
    'Time_s',                 NaN, ...
    'TargetPosition_units',   request.goalState.position_units, ...
    'TerminalVelocityPolicy', "zero");
result.TrajectoryDuration_s = NaN;
result.ElapsedTime_s        = elapsedTime_s;

% Supplied values retain the fields present before normalization; entirely
% empty inputs already use defaults. Requested values also have missing fields
% filled and standard row shapes, before target matching or unwrapping.
% Retain both so later trials can be checked against the original request.
result.SuppliedLimits     = request.originalInputs.suppliedLimits;
result.RequestedLimits    = request.originalInputs.requestedLimits;
result.RequestedGoalState = request.originalInputs.requestedGoalState;
result.SuppliedGoalState  = request.originalInputs.suppliedGoalState;
if ~isempty(request.parentRequest)
    result.ParentRequest = request.parentRequest;
end
end
