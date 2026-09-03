function [isStaticHorizon, occupiedShape] = queryStaticHorizon( ...
        obstacles, startTime_s, endTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   isStaticHorizon = obstacleAvoidance.obstacles.queryStaticHorizon( ...
%       obstacles, startTime_s, endTime_s)
%   [isStaticHorizon, occupiedShape] = ...
%       obstacleAvoidance.obstacles.queryStaticHorizon( ...
%       obstacles, startTime_s, endTime_s)
%**************************************************************************
% PURPOSE
%   - Determine whether every prepared obstacle is time invariant and active
%     over a complete request horizon.
%   - Create the protected static union only when a caller requests it.
%**************************************************************************
% INPUTS
%   - obstacles (prepared obstacle struct array)
%       Each record supplies time_s and InternalPreparation.
%   - startTime_s (finite numeric scalar)
%       Inclusive beginning of the query horizon.
%   - endTime_s (finite numeric scalar)
%       Inclusive end of the query horizon, not earlier than startTime_s.
%**************************************************************************
% OUTPUTS
%   - isStaticHorizon (scalar logical)
%       True only when every obstacle is invariant and active throughout the
%       horizon. Empty time histories return false rather than indexing error.
%   - occupiedShape (scalar polyshape)
%       Union of StaticShape records when supported and requested; otherwise
%       an empty polyshape.
%**************************************************************************
% UNITS
%   - Time is seconds and protected boundary coordinates are degrees.
%**************************************************************************

%% Section 1: Evaluate Static Activity Over The Horizon

isStaticHorizon = true;
occupiedShape = polyshape();
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    sourceTime_s = double(obstacle.time_s(:));
    isActive = isscalar(sourceTime_s) || (~isempty(sourceTime_s) && ...
        startTime_s >= sourceTime_s(1) && endTime_s <= sourceTime_s(end));
    if ~obstacle.InternalPreparation.IsTimeInvariant || ~isActive
        isStaticHorizon = false;
        return;
    end
end

%% Section 2: Create The Optional Protected Union

if nargout < 2
    return;
end
for obstacleIndex = 1:numel(obstacles)
    occupiedShape = union(occupiedShape, ...
        obstacles(obstacleIndex).InternalPreparation.StaticShape);
end
end
