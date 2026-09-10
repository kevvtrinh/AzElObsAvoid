function time_s = createTimeLayers(obstacles, startTime_s, endTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   time_s = obstacleAvoidance.search.createTimeLayers(obstacles, startTime_s, endTime_s)
% PURPOSE
%   Share the physical time grid between route search and timed motion solving.
% INPUTS
%   Canonical or prepared obstacles and the requested start/end times.
% OUTPUTS
%   Sorted unique source, midpoint, endpoint, and uniform request times.
% UNITS
%   Seconds.

%% Section 1: Retain The Complete Input-Derived Grid
time_s = [startTime_s; linspace(startTime_s, endTime_s, 9).'; endTime_s];
% Keep each obstacle event and interval midpoint, including narrow openings.
for obstacleIndex = 1:numel(obstacles)
    sourceTime_s = obstacles(obstacleIndex).time_s(:);
    midpoint_s = 0.5 * (sourceTime_s(1:end - 1) + sourceTime_s(2:end));
    time_s = [time_s; sourceTime_s; midpoint_s]; %#ok<AGROW>
end
time_s = unique(time_s(time_s >= startTime_s & time_s <= endTime_s));
end
