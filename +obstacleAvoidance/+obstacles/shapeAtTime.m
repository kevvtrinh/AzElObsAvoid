function [shape, geometry] = shapeAtTime(obstacle, queryTime_s, geometryOnly)
%% Section 0: Header & Readme
% SYNTAX
%   [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
%       obstacle, queryTime_s)
%   [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
%       obstacle, queryTime_s, geometryOnly)
%
% PURPOSE
%   - Return protected obstacle geometry active at one physical time.
%   - Interpolate verified corresponding vertices and otherwise return a
%     conservative swept enclosure for the complete source interval.
%
% INPUTS
%   - obstacle (scalar canonical or prepared struct)
%   - queryTime_s (finite numeric scalar)
%   - geometryOnly (logical scalar, optional; default false)
%
% OUTPUTS
%   - shape (scalar polyshape or [])
%   - geometry (scalar struct)
%       Boundary, speed bound, topology, and source-slice metadata.
%       status is metadata and never deactivates supplied geometry.
%
% UNITS
%   - Geometry is degrees; time is seconds; speed is degrees per second.
%   - See obstacle_history_contract.md for the complete history model.
%

%% Section 1: Validate And Select The Source Interval

if ~isstruct(obstacle) || ~isscalar(obstacle) || ~all(isfield(obstacle, {'time_s', 'az_deg', 'el_deg'}))
    error("shapeAtTime:InvalidObstacle", "obstacle must be one canonical record.");
end
validateattributes(queryTime_s, {'numeric'}, {'real', 'finite', 'scalar'});
if nargin < 3 || isempty(geometryOnly)
    geometryOnly = false;
end
geometryOnly = obstacleAvoidance.input.normalizeLogicalScalar(geometryOnly, "geometryOnly", "shapeAtTime:InvalidGeometryOnly");
queryTime_s  = double(queryTime_s);
obstacle     = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
[shape, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, queryTime_s, geometryOnly);
end
