function nodes_units = createTimedVisibilityNodes(shape, start_units, goal_units, limits, candidateOffset_units)
%% Section 0: Header & Readme
% SYNTAX
%   nodes_units = obstacleAvoidance.search.createTimedVisibilityNodes( ...
%       shape, start_units, goal_units, limits, candidateOffset_units)
%**************************************************************************
% PURPOSE
%   - Create a deterministic staging-node set for timed route search.
%**************************************************************************
% INPUTS
%   - shape (scalar polyshape)
%       Sampled swept proposal geometry.
%   - start_units (1-by-2 numeric row)
%       Request start position.
%   - goal_units (1-by-2 numeric row)
%       Request goal position.
%   - limits (scalar struct)
%       Workspace limits.
%   - candidateOffset_units (positive scalar)
%       Physical steering offset.
%**************************************************************************
% OUTPUTS
%   - nodes_units (N-by-2 numeric array)
%       Retained proposal nodes with the start first and the goal second.
%       An empty proposal shape returns the endpoints alone; invalid input
%       throws an error.
%**************************************************************************
% UNITS
%   - Positions and offsets are coordinate units.
%**************************************************************************

%% Section 1: Offset And Bound The Proposal Boundary

candidateShape = shape;
if ~isempty(shape.Vertices)
    candidateShape = polybuffer(shape, candidateOffset_units, "JointType", "miter");
end
rawNodes_units       = candidateShape.Vertices;
isInsideWorkspace    = rawNodes_units(:, 1) >= limits.xInterval_units(1) & ...
    rawNodes_units(:, 1) <= limits.xInterval_units(2) & ...
    rawNodes_units(:, 2) >= limits.yInterval_units(1) & ...
    rawNodes_units(:, 2) <= limits.yInterval_units(2);
candidateNodes_units = unique(rawNodes_units(isInsideWorkspace, :), "rows", "stable");

%% Section 2: Retain A Deterministic Global Boundary Cover

% The timed graph checks every retained node pair at every physical layer.
% This stage owns that proposal-only quadratic work budget; final acceptance
% still comes exclusively from BMTP and the independent validator.
workBudget = 1e6;
[edgeStart_units, ~] = obstacleAvoidance.geometry.boundaryToEdges(shape, 1e-12);
candidateCountLimit = max(2, floor(sqrt(2 * workBudget / max(1, size(edgeStart_units, 1)))) - 2);
if size(candidateNodes_units, 1) > candidateCountLimit
    candidateNodes_units = selectBoundaryCover( ...
        candidateNodes_units, start_units, goal_units, candidateCountLimit);
end
nodes_units = unique([start_units; goal_units; candidateNodes_units], "rows", "stable");
end

%% Section 3: Local Functions

function selected_units = selectBoundaryCover(candidates_units, start_units, goal_units, candidateCount)
    % Preserve endpoint access, global supports, and ring-order coverage.
    endpointCount   = min(4, floor(candidateCount / 6));
    directionCount  = min(16, floor(candidateCount / 3));
    uniformCount    = candidateCount - directionCount - 2 * endpointCount;
    selectedIndices = unique(round(linspace(1, size(candidates_units, 1), uniformCount))).';
    if directionCount > 0
        angle_rad           = (0:directionCount - 1).' * (2 * pi / directionCount);
        directions          = [cos(angle_rad), sin(angle_rad)];
        [~, supportIndices] = max(candidates_units * directions.', [], 1);
        selectedIndices     = [selectedIndices; supportIndices(:)];
    end
    for reference_units = [start_units; goal_units].'
        [~, proximityOrder] = sort(vecnorm(candidates_units - reference_units.', 2, 2));
        selectedIndices     = [selectedIndices; proximityOrder(1:endpointCount)]; %#ok<AGROW>
    end
    selectedIndices = unique(selectedIndices, "stable");
    selected_units  = candidates_units( ...
        selectedIndices(1:min(candidateCount, numel(selectedIndices))), :);
end
