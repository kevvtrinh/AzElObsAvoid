function [sampledUnion, sampledShapeCount] = ...
        createSampledObstacleUnion(obstacles, sampleTimes_s)
%% Section 0: Header & Readme
% SYNTAX
%   [sampledUnion, sampledShapeCount] = ...
%       obstacleAvoidance.search.createSampledObstacleUnion( ...
%       obstacles, sampleTimes_s)
%**************************************************************************
% PURPOSE
%   - Create one topology-proposal shape from exact obstacle-time queries.
%   - Report how many nonempty sampled shapes contributed to the union.
%**************************************************************************
% INPUTS
%   - obstacles (prepared canonical obstacle struct array)
%       Complete protected histories used only for route proposals.
%   - sampleTimes_s (finite numeric vector)
%       Ordered physical times at which each obstacle is queried.
%**************************************************************************
% OUTPUTS
%   - sampledUnion (scalar polyshape)
%       Union of every nonempty sampled protected obstacle shape.
%   - sampledShapeCount (nonnegative integer scalar)
%       Number of nonempty obstacle-time shapes in the union.
%**************************************************************************
% UNITS
%   - Geometry is degrees and sampleTimes_s is seconds.
%**************************************************************************

%% Section 1: Query Every Obstacle At Every Time

% Visibility planning needs one conservative spatial proposal. Keep every
% exact time query in this stage so the caller can distinguish sampled-union
% geometry from the denser-history fallback.

validateattributes(sampleTimes_s, {'numeric'}, ...
    {'real', 'finite', 'vector'});
parts = cell(numel(sampleTimes_s) * numel(obstacles), 1);
sampledShapeCount = 0;
for timeIndex = 1:numel(sampleTimes_s)
    for obstacleIndex = 1:numel(obstacles)
        part = obstacleAvoidance.obstacles.shapeAtTime( ...
            obstacles(obstacleIndex), sampleTimes_s(timeIndex));
        if ~isempty(part.Vertices)
            sampledShapeCount = sampledShapeCount + 1;
            parts{sampledShapeCount} = part;
        end
    end
end

%% Section 2: Create The Proposal Union

% The union is route guidance only. Final acceptance still checks a timed
% polynomial motion against the complete authoritative obstacle histories.

sampledUnion = polyshape();
if sampledShapeCount > 0
    sampledUnion = union([parts{1:sampledShapeCount}]);
end
end
