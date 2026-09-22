function searchTimes_s = createTimeLayers(obstacles, startTime_s, endTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   searchTimes_s = obstacleAvoidance.search.createTimeLayers(obstacles, startTime_s, endTime_s)
%**************************************************************************
% PURPOSE
%   - Choose the times at which the timed route search can place nodes.
%     Include obstacle sample times, times halfway between samples, and
%     evenly spaced times across the requested trip.
%**************************************************************************
% INPUTS
%   - obstacles (canonical or prepared obstacle array)
%       Obstacle histories that contribute sample times and interval midpoints.
%   - startTime_s (finite scalar)
%       Requested interval start time.
%   - endTime_s (finite scalar)
%       Requested interval end time.
%**************************************************************************
% OUTPUTS
%   - searchTimes_s (column vector)
%       Increasing times within [startTime_s endTime_s], without duplicates.
%**************************************************************************
% UNITS
%   - Time is seconds.
%**************************************************************************

%% Section 1: Combine Request Times And Obstacle Sample Times

% Start with nine evenly spaced times, including departure and arrival.
% The search checks motion between times as well; these are not the only
% instants at which a collision matters.
searchTimes_s = [startTime_s; linspace(startTime_s, endTime_s, 9).'; endTime_s];

% Keep all recorded obstacle times, even if preparation combined several
% intervals into one motion model. Add each midpoint too: samples at 2 s
% and 6 s contribute 2 s, 4 s, and 6 s to the search.
for obstacleIndex = 1:numel(obstacles)
    obstacleSampleTimes_s = obstacles(obstacleIndex).time_s(:);
    midpointTimes_s       = 0.5 * (obstacleSampleTimes_s(1:end - 1) + obstacleSampleTimes_s(2:end));
    searchTimes_s         = [searchTimes_s; obstacleSampleTimes_s; midpointTimes_s]; %#ok<AGROW>
end

% Remove times outside the requested trip, then sort and remove duplicates.
searchTimes_s = unique(searchTimes_s(searchTimes_s >= startTime_s & searchTimes_s <= endTime_s));
end
