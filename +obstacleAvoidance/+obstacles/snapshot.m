function obstacleSnapshot = snapshot(obstacles, time_s, includeConvexRegions)
%% Section 0: Header & Readme
% SYNTAX
%   obstacleSnapshot = obstacleAvoidance.obstacles.snapshot(obstacles, time_s)
%   obstacleSnapshot = obstacleAvoidance.obstacles.snapshot( ...
%       obstacles, time_s, includeConvexRegions)
%**************************************************************************
% PURPOSE
%   - Collect the prepared obstacle geometry for a requested time.
%     Safety margins are already included in the protected shapes.
%**************************************************************************
% INPUTS
%   - obstacles (prepared obstacle array)
%       Obstacle histories already processed by prepareObstacles.
%   - time_s (numeric scalar)
%       Time at which to read the obstacle geometry.
%   - includeConvexRegions (logical scalar, optional; default true)
%       Include convex obstacle pieces for motion constraints and geometry
%       checks. Set false when only shapes and boundaries are needed.
%**************************************************************************
% OUTPUTS
%   - obstacleSnapshot (struct array)
%       Protected shapes, boundaries, and optional convex obstacle pieces.
%       Some prepared models enclose the motion between two samples, so their
%       snapshot covers that interval rather than only the requested instant.
%       Unsupported or unprepared interval geometry throws an error.
%**************************************************************************
% UNITS
%   - Position uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Calculate Each Obstacle Shape At The Requested Time

if nargin < 3
    includeConvexRegions = true;
end
obstacleSnapshot = struct( ...
    'ProtectedShape',          {}, ...
    'ProtectedVertices_units', {}, ...
    'Regions_units',           {});
for obstacleIndex = 1:numel(obstacles)
    [protectedShape, shapeDetails] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
        obstacles(obstacleIndex), time_s);
    if isempty(protectedShape.Vertices)
        % An obstacle outside its active time range has no occupied area.
        % The snapshot includes only obstacles with a nonempty shape here.
        continue;
    end
    regions_units = cell(0, 1);
    if includeConvexRegions
        if shapeDetails.UsesMovingCells
            % These prepared pieces already enclose the obstacle's motion
            % over the sample interval. Reuse them without changing the area.
            intervalIndex = shapeDetails.LowerSampleIndex;
            regions_units = obstacles(obstacleIndex).InternalPreparation.IntervalStartRegions_units{intervalIndex};
        else
            % Split the shape into convex pieces: polygons with no inward
            % corners. The pieces together cover the same protected area.
            regions_units = obstacleAvoidance.geometry.convexRegions(protectedShape);
        end
    end
    obstacleSnapshot(end + 1) = struct( ...
        'ProtectedShape',          protectedShape, ...
        'ProtectedVertices_units', protectedShape.Vertices, ...
        'Regions_units',           {regions_units}); %#ok<AGROW>
end
end
