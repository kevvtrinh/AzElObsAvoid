function time_s = createTimeLayers(obstacles, startTime_s, endTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   time_s = obstacleAvoidance.search.createTimeLayers(obstacles, startTime_s, endTime_s)
%**************************************************************************
% PURPOSE
%   - Build the physical time grid shared by route search and timed motion.
%**************************************************************************
% INPUTS
%   - obstacles (canonical or prepared obstacle array)
%       Obstacle histories that contribute source times and midpoints.
%   - startTime_s (finite scalar)
%       Requested interval start time.
%   - endTime_s (finite scalar)
%       Requested interval end time.
%**************************************************************************
% OUTPUTS
%   - time_s (column vector)
%       Sorted unique source, midpoint, endpoint, and uniform request times;
%       invalid input throws an error.
%**************************************************************************
% UNITS
%   - Time is seconds.
%**************************************************************************

%% Section 1: Retain The Complete Input-Derived Grid

time_s = [startTime_s; linspace(startTime_s, endTime_s, 9).'; endTime_s];
% Keep each obstacle event and interval midpoint, including narrow openings.
% Every supplied keyframe stays a search event: merged keyframe spans reduce
% cells, not the search's time resolution.
for obstacleIndex = 1:numel(obstacles)
    sourceTimes_s   = obstacles(obstacleIndex).time_s(:);
    midpointTimes_s = 0.5 * (sourceTimes_s(1:end - 1) + sourceTimes_s(2:end));
    time_s           = [time_s; sourceTimes_s; midpointTimes_s]; %#ok<AGROW>
end

% Keep only the requested horizon, and keep each retained time once.
time_s = unique(time_s(time_s >= startTime_s & time_s <= endTime_s));
end
