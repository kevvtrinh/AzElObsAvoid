function boundary_deg = buildEnvelopeBoundary(obstacles, envelopePadding_deg)
%% Section 0: Header & Readme
% SYNTAX
%   boundary_deg = azElInternal.obstacles.buildEnvelopeBoundary( ...
%       obstacles, envelopePadding_deg)
%**************************************************************************
% PURPOSE
%   - Build one conservative boundary containing every protected obstacle
%     history. Static sources retain their exact shape; changing histories use
%     a convex hull of all stored vertices so a time-independent corridor can
%     never omit an observed protected position.
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

% Padding is numerical reserve on already-protected geometry. It is not the
% public safety margin and must therefore stay small and explicit.
validateattributes(envelopePadding_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
envelopeShape = polyshape();

% Add one conservative history envelope for each independent obstacle record.
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    geometryIsStatic = true;

    % Prove stationarity by comparing every stored slice with the first slice.
    for sliceIndex = 2:numel(obstacle.az_deg)
        geometryIsStatic = geometryIsStatic && ...
            isequal(obstacle.az_deg{sliceIndex}, obstacle.az_deg{1}) && ...
            isequal(obstacle.el_deg{sliceIndex}, obstacle.el_deg{1});
    end
    if geometryIsStatic
        % Reuse exact static geometry to avoid filling concavities needlessly.
        protectedShape = azElInternal.obstacles.shapeAtTime( obstacle, obstacle.time_s(1));
        protectedEnvelope = polybuffer( protectedShape, envelopePadding_deg);
    else
        % A time-independent certificate must cover the whole history, not an
        % interpolation snapshot. The convex hull is conservative by design.
        historyVertices_deg = [ vertcat(obstacle.az_deg{:}), vertcat(obstacle.el_deg{:})];
        historyVertices_deg = historyVertices_deg( all(isfinite(historyVertices_deg), 2), :);
        historyVertices_deg = unique( historyVertices_deg, "rows", "stable");
        hullIndex = convhull( historyVertices_deg(:, 1), historyVertices_deg(:, 2));
        hullShape = polyshape( ...
            historyVertices_deg(hullIndex(1:end - 1), :), "Simplify", false, "KeepCollinearPoints", true);
        protectedEnvelope = polybuffer(hullShape, envelopePadding_deg);
    end
    envelopeShape = union(envelopeShape, protectedEnvelope);
end
boundary_deg = envelopeShape.Vertices;
end
