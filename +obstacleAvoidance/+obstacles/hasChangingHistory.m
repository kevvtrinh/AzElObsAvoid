function changing = hasChangingHistory(obstacles, startTime_s, endTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   changing = obstacleAvoidance.obstacles.hasChangingHistory( ...
%       obstacles, startTime_s, endTime_s)
%**************************************************************************
% PURPOSE
%   - Determine whether obstacle geometry or its active span changes during
%     a planning request.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle structure array)
%       Each obstacle supplies time_s and matched az_deg and el_deg cells.
%   - startTime_s (finite numeric scalar)
%       Beginning of the planning request.
%   - endTime_s (finite numeric scalar)
%       End of the planning request, not earlier than startTime_s.
%**************************************************************************
% OUTPUTS
%   - changing (scalar logical)
%       True when an obstacle is active for only part of the request or any
%       stored geometry slice differs from its first slice.
%**************************************************************************
% UNITS
%   - Time is seconds. Boundary coordinates are degrees.
%**************************************************************************

%% Section 1: Inspect Active Spans And Stored Geometry

changing = false;
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    hasFiniteActiveSpan = numel(obstacle.time_s) > 1;
    if hasFiniteActiveSpan && ...
            (obstacle.time_s(1) > startTime_s || ...
            obstacle.time_s(end) < endTime_s)
        changing = true;
        return;
    end

    for sampleIndex = 2:numel(obstacle.time_s)
        sameAzimuth = isequaln( ...
            obstacle.az_deg{sampleIndex}, obstacle.az_deg{1});
        sameElevation = isequaln( ...
            obstacle.el_deg{sampleIndex}, obstacle.el_deg{1});
        if ~sameAzimuth || ~sameElevation
            changing = true;
            return;
        end
    end
end
end
