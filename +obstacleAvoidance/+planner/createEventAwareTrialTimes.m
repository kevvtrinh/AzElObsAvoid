function trialTime_s = createEventAwareTrialTimes( ...
        lowerTime_s, upperTime_s, eventTime_s, coarseIntervalCount)
%% Section 0: Header & Readme
% SYNTAX
%   trialTime_s = obstacleAvoidance.planner.createEventAwareTrialTimes( ...
%       lowerTime_s, upperTime_s, eventTime_s, coarseIntervalCount)
%**************************************************************************
% PURPOSE
%   - Create a bounded chronological trial grid that retains source events.
%   - Add one interior trial between consecutive events so a feasible window
%     is not represented only by its possibly infeasible endpoints.
%**************************************************************************
% INPUTS
%   - lowerTime_s, upperTime_s (finite numeric scalars)
%       Increasing inclusive trial range.
%   - eventTime_s (finite numeric vector or empty)
%       Source-derived event times; values outside the range are ignored.
%   - coarseIntervalCount (positive integer scalar)
%       Number of uniform background intervals spanning the trial range.
%**************************************************************************
% OUTPUTS
%   - trialTime_s (N-by-1 double vector)
%       Sorted unique event, event-midpoint, and uniform coarse trial times.
%**************************************************************************
% UNITS
%   - All time values are seconds.
%**************************************************************************

%% Section 1: Validate The Trial Range

validateattributes(lowerTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(upperTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', '>', lowerTime_s});
validateattributes(eventTime_s, {'numeric'}, ...
    {'real', 'finite', 'vector'});
validateattributes(coarseIntervalCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});

%% Section 2: Combine Uniform And Event-Aware Trials

eventTime_s = double(eventTime_s(:));
eventTime_s = eventTime_s(eventTime_s >= lowerTime_s & ...
    eventTime_s <= upperTime_s);
eventAnchorTime_s = unique([double(lowerTime_s); eventTime_s; ...
    double(upperTime_s)]);
eventMidpointTime_s = 0.5 * (eventAnchorTime_s(1:end - 1) + ...
    eventAnchorTime_s(2:end));
coarseTime_s = linspace(lowerTime_s, upperTime_s, ...
    coarseIntervalCount + 1).';
trialTime_s = unique([coarseTime_s; eventAnchorTime_s; ...
    eventMidpointTime_s]);
end
