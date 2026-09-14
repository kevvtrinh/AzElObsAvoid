function scene = snapshot(obstacles, time_s, includeConvexRegions)
%% Section 0: Header & Readme
% SYNTAX: scene = obstacleAvoidance.obstacles.snapshot(obstacles, time_s)
%   scene = obstacleAvoidance.obstacles.snapshot(obstacles, time_s, includeConvexRegions)
% PURPOSE: Evaluate authoritative protected geometry for visibility planning.
% INPUTS: Prepared histories, scalar physical time, and an optional logical selecting whether the
%   convex exclusion partition is required. A caller that needs only the boundary for a visibility
%   graph passes false and leaves Regions_units empty rather than partitioning geometry it discards.
% OUTPUTS: Shapes, exact boundaries, and convex exclusion regions.
% UNITS: Seconds and coordinate units.

%% Section 1: Evaluate Each History
if nargin < 3, includeConvexRegions = true; end
scene = struct('ProtectedShape', {}, 'ProtectedVertices_units', {}, 'Regions_units', {});
for k = 1:numel(obstacles)
    [shape,geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacles(k), time_s);
    if isempty(shape.Vertices), continue; end
    regions_units = cell(0,1);
    if includeConvexRegions
        if geometry.GeometryModel=="sweptCorrespondingConvexCells"
            regions_units=obstacles(k).InternalPreparation.IntervalStartRegions_units{geometry.LowerSampleIndex};
        else
            regions_units=obstacleAvoidance.geometry.convexRegions(shape);
        end
    end
    scene(end+1) = struct('ProtectedShape', shape, 'ProtectedVertices_units', shape.Vertices, ...
        'Regions_units', {regions_units}); %#ok<AGROW>
end
end
