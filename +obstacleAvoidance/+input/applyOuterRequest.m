function result = applyOuterRequest(result, outerRequest)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.input.applyOuterRequest(result, outerRequest)
%**************************************************************************
% PURPOSE
%   - Carry the public request a motion must be accepted against into its
%     record. A periodic request is planned as plain requests in the
%     unwrapped frame, and the chronological search plans trials on their
%     own fixed clocks; the record that reaches the acceptance gate declares
%     the outer request so one validation asserts everything.
%**************************************************************************
% INPUTS
%   - result (scalar struct)
%       Planner record.
%   - outerRequest (scalar struct)
%       SuppliedLimits, SuppliedGoalState, RequestedGoalState, GoalTime_s,
%       and GoalTimeMode, plus optional RequestedLimits, Obstacles, WrapX,
%       WrapY (periodic requests) and FixedArrivalTrialTime_s (chronological
%       trials).
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       The record with the outer request declared and no OuterRequest field.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Declare The Outer Request In The Record

if isfield(result, 'OuterRequest')
    result = rmfield(result, 'OuterRequest');
end
if isfield(outerRequest, 'FixedArrivalTrialTime_s')
    result.FixedArrivalTrialTime_s = outerRequest.FixedArrivalTrialTime_s;
end
result.SuppliedLimits     = outerRequest.SuppliedLimits;
result.SuppliedGoalState  = outerRequest.SuppliedGoalState;
result.RequestedGoalState = outerRequest.RequestedGoalState;
if isfield(outerRequest, 'RequestedLimits')
    result.RequestedLimits = outerRequest.RequestedLimits;
end
if isfield(outerRequest, 'Obstacles')
    result.Inputs.obstacles = outerRequest.Obstacles;
end
if isfield(outerRequest, 'WrapX')
    result.Options.WrapX = outerRequest.WrapX;
    result.Options.WrapY = outerRequest.WrapY;
end
result.Inputs.goalState.time_s = outerRequest.GoalTime_s;
result.Options.GoalTimeMode    = outerRequest.GoalTimeMode;
end
