function outerRequest = createOuterRequest(request, requestContext)
%% Section 0: Header & Readme
% SYNTAX
%   outerRequest = obstacleAvoidance.planning.createOuterRequest( ...
%       request, requestContext)
%**************************************************************************
% PURPOSE
%   - Declare the outer request a derived trial plans under: the supplied
%     and requested inputs and wrap options its acceptance declaration
%     must carry, the fixed-arrival trial clock (NaN until a chronological
%     trial sets it), and the two products a trial may reuse instead of
%     rebuilding, each declared empty here and filled by the trial's
%     producer under a key the trial rebuilds for itself.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Normalized planner states, limits, and options of the outer request.
%   - requestContext (scalar struct)
%       Original inputs and provenance of the outer request.
%**************************************************************************
% OUTPUTS
%   - outerRequest (scalar struct)
%       Private provenance for a derived request, one declared shape. A
%       NaN FixedArrivalTrialTime_s means no trial clock; an empty Key,
%       and for the validation Feasible = false, means no product to reuse.
%**************************************************************************
% UNITS
%   - Time is seconds and positions are coordinate units.
%**************************************************************************

%% Section 1: Declare The Outer Request

outerRequest = struct( ...
    'SuppliedLimits',          requestContext.suppliedLimits, ...
    'SuppliedGoalState',       requestContext.suppliedGoalState, ...
    'RequestedLimits',         requestContext.requestedLimits, ...
    'RequestedGoalState',      requestContext.requestedGoalState, ...
    'Obstacles',               {requestContext.obstacles}, ...
    'WrapX',                   request.options.WrapX, ...
    'WrapY',                   request.options.WrapY, ...
    'GoalTime_s',              request.goalState.time_s, ...
    'GoalTimeMode',            request.options.GoalTimeMode, ...
    'FixedArrivalTrialTime_s', NaN, ...
    'EndpointValidation',      struct('Key', [], 'Feasible', false, 'Message', "", 'Reason', ""), ...
    'InitialSkeleton',         struct('Key', [], 'Skeleton', []));
end
