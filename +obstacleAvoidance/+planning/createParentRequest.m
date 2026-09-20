function parentRequest = createParentRequest(request)
%% Section 0: Header & Readme
% SYNTAX
%   parentRequest = obstacleAvoidance.planning.createParentRequest( ...
%       request)
%**************************************************************************
% PURPOSE
%   - Some requests are answered by planning other requests: a wrapped
%     request as unwrapped copies, an earliest-arrival search as a series of
%     fixed-arrival trials. Each inner run carries this record of the
%     request the user actually made, so its result is declared against the
%     original inputs and wrap options rather than the trial's own.
%   - It also carries the trial clock (NaN until an arrival-time trial sets
%     it) and two products a trial may reuse instead of rebuilding, the
%     endpoint check and the vertex visibility, declared empty here and
%     filled by the trial's producer under a key the trial rebuilds itself.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Normalized planner states, limits, and options of the parent request;
%       its context holds the original inputs and provenance.
%**************************************************************************
% OUTPUTS
%   - parentRequest (scalar struct)
%       The record a child request plans under, always the same shape. A
%       NaN FixedArrivalTrialTime_s means no trial clock; an empty Key,
%       and for the validation Feasible = false, means no product to reuse.
%**************************************************************************
% UNITS
%   - Time is seconds and positions are coordinate units.
%**************************************************************************

%% Section 1: Declare The Parent Request

parentRequest = struct( ...
    'SuppliedLimits',          request.context.suppliedLimits, ...
    'SuppliedGoalState',       request.context.suppliedGoalState, ...
    'RequestedLimits',         request.context.requestedLimits, ...
    'RequestedGoalState',      request.context.requestedGoalState, ...
    'Obstacles',               {request.context.obstacles}, ...
    'WrapX',                   request.options.WrapX, ...
    'WrapY',                   request.options.WrapY, ...
    'GoalTime_s',              request.goalState.time_s, ...
    'GoalTimeMode',            request.options.GoalTimeMode, ...
    'FixedArrivalTrialTime_s', NaN, ...
    'EndpointValidation',      struct('Key', [], 'Feasible', false, 'Message', "", 'Reason', ""), ...
    'InitialVertexVisibility',         struct('Key', [], 'VertexVisibility', []));
end
