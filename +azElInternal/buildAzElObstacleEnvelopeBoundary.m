function boundary_deg = buildAzElObstacleEnvelopeBoundary( ...
        obstacles, envelopePadding_deg)
%% Section 0: Header & Readme
% SYNTAX
%   boundary_deg = azElInternal.buildAzElObstacleEnvelopeBoundary( ...
%       obstacles, envelopePadding_deg)
%**************************************************************************
% PURPOSE
%   - Represent every complete protected obstacle history by a union of
%     convex regions reusable across candidate seeds.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle array)
%   - envelopePadding_deg (nonnegative conservative numerical padding)
%**************************************************************************
% OUTPUTS
%   - boundary_deg (NaN-separated convex region vertices)
%**************************************************************************
% UNITS
%   - Boundary coordinates and padding are degrees.
%**************************************************************************

%% Section 1: Build The Complete Protected Envelope

validateattributes(envelopePadding_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
envelopeShape = polyshape();
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    geometryIsStatic = true;
    for sliceIndex = 2:numel(obstacle.az_deg)
        geometryIsStatic = geometryIsStatic && ...
            isequal(obstacle.az_deg{sliceIndex}, obstacle.az_deg{1}) && ...
            isequal(obstacle.el_deg{sliceIndex}, obstacle.el_deg{1});
    end
    if geometryIsStatic
        protectedShape = azElInternal.obstacleShapeAtTime( ...
            obstacle, obstacle.time_s(1));
        protectedEnvelope = polybuffer( ...
            protectedShape, envelopePadding_deg);
    else
        historyVertices_deg = [ ...
            vertcat(obstacle.az_deg{:}), vertcat(obstacle.el_deg{:})];
        historyVertices_deg = historyVertices_deg( ...
            all(isfinite(historyVertices_deg), 2), :);
        historyVertices_deg = unique( ...
            historyVertices_deg, "rows", "stable");
        hullIndex = convhull( ...
            historyVertices_deg(:, 1), historyVertices_deg(:, 2));
        hullShape = polyshape( ...
            historyVertices_deg(hullIndex(1:end - 1), :), ...
            "Simplify", false, "KeepCollinearPoints", true);
        protectedEnvelope = polybuffer(hullShape, envelopePadding_deg);
    end
    envelopeShape = union(envelopeShape, protectedEnvelope);
end
boundary_deg = envelopeShape.Vertices;
end
