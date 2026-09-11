function scene = snapshot(obstacles, time_s)
%% Section 0: Header & Readme
% SYNTAX: scene = obstacleAvoidance.obstacles.snapshot(obstacles, time_s)
% PURPOSE: Evaluate authoritative protected geometry for visibility planning.
% INPUTS: Prepared histories and scalar physical time.
% OUTPUTS: Shapes, exact boundaries, and convex exclusion regions.
% UNITS: Seconds and coordinate units.

%% Section 1: Evaluate Each History
scene = struct('ProtectedShape', {}, 'ProtectedVertices_units', {}, 'Regions_units', {});
for k = 1:numel(obstacles)
    shape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacles(k), time_s);
    if isempty(shape.Vertices), continue; end
    scene(end+1) = struct('ProtectedShape', shape, 'ProtectedVertices_units', shape.Vertices, ...
        'Regions_units', {obstacleAvoidance.geometry.convexRegions(shape)}); %#ok<AGROW>
end
end
