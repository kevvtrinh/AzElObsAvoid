function scene = snapshot(obstacles, time_s, includeConvexRegions)
%% Section 0: Header & Readme
% SYNTAX
%   scene = obstacleAvoidance.obstacles.snapshot(obstacles, time_s)
%   scene = obstacleAvoidance.obstacles.snapshot( ...
%       obstacles, time_s, includeConvexRegions)
%**************************************************************************
% PURPOSE
%   - Evaluate protected geometry for visibility planning.
%**************************************************************************
% INPUTS
%   - obstacles (prepared obstacle array)
%       Source-checked obstacle histories to evaluate.
%   - time_s (numeric scalar)
%       Physical snapshot time.
%   - includeConvexRegions (logical scalar, optional; default true)
%       Whether to include convex exclusion regions for each shape.
%**************************************************************************
% OUTPUTS
%   - scene (struct array)
%       Shapes, exact boundaries, and optional convex exclusion regions.
%       Unsupported or unprepared interval geometry throws an error.
%**************************************************************************
% UNITS
%   - Position uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Evaluate Each History

if nargin < 3
    includeConvexRegions = true;
end
scene = struct('ProtectedShape', {}, 'ProtectedVertices_units', {}, 'Regions_units', {});
for obstacleIndex = 1:numel(obstacles)
    [shape, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
        obstacles(obstacleIndex), time_s);
    if isempty(shape.Vertices)
        continue;
    end
    regions_units = cell(0, 1);
    if includeConvexRegions
        if geometry.UsesSweptCells
            regions_units = obstacles(obstacleIndex).InternalPreparation.IntervalStartRegions_units{geometry.LowerSampleIndex};
        else
            regions_units = obstacleAvoidance.geometry.convexRegions(shape);
        end
    end
    scene(end + 1) = struct('ProtectedShape', shape, 'ProtectedVertices_units', shape.Vertices, ...
        'Regions_units', {regions_units}); %#ok<AGROW>
end
end
