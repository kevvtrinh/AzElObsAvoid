function nodes_units = createTimedVisibilityNodes( ...
    combinedSampleShape, start_units, goal_units, limits, candidateOffset_units)
%% Section 0: Header & Readme
% SYNTAX
%   nodes_units = obstacleAvoidance.search.createTimedVisibilityNodes( ...
%       combinedSampleShape, start_units, goal_units, limits, candidateOffset_units)
%**************************************************************************
% PURPOSE
%   - Place candidate route points around sampled obstacle boundaries.
%     The timed search later checks which connections are usable as the
%     obstacles move; these points alone do not establish a valid motion.
%**************************************************************************
% INPUTS
%   - combinedSampleShape (scalar polyshape)
%       Union of protected obstacle shapes sampled at selected times.
%   - start_units (1-by-2 numeric row)
%       Request start position.
%   - goal_units (1-by-2 numeric row)
%       Request goal position.
%   - limits (scalar struct)
%       Workspace limits.
%   - candidateOffset_units (positive scalar)
%       Distance beyond the sampled boundary used to place candidate points.
%**************************************************************************
% OUTPUTS
%   - nodes_units (N-by-2 numeric array)
%       Start, goal, and retained candidate points, with duplicates removed
%       in that order. An empty shape contributes no extra points.
%**************************************************************************
% UNITS
%   - Positions and offsets are coordinate units.
%**************************************************************************

%% Section 1: Place Candidate Points Outside The Sampled Boundary

% Offset only the shape used to place route points. This gives BMTP room
% to turn without changing the protected geometry used for collision checks.
offsetBoundaryShape = combinedSampleShape;
if ~isempty(combinedSampleShape.Vertices)
    offsetBoundaryShape = polybuffer(combinedSampleShape, candidateOffset_units, "JointType", "miter");
end
boundaryVertices_units = offsetBoundaryShape.Vertices;
isInsideWorkspace = boundaryVertices_units(:, 1) >= limits.xInterval_units(1) & ...
    boundaryVertices_units(:, 1) <= limits.xInterval_units(2) & ...
    boundaryVertices_units(:, 2) >= limits.yInterval_units(1) & ...
    boundaryVertices_units(:, 2) <= limits.yInterval_units(2);
candidateNodes_units = unique(boundaryVertices_units(isInsideWorkspace, :), "rows", "stable");

%% Section 2: Limit The Candidate Set For The Timed Search

% The work estimate grows with node pairs x boundary edges. Reserve two
% nodes for start and goal, then limit the extra candidates using that estimate.
% The search checks the retained points; it does not try every boundary vertex
% when this limit applies. BMTP and independent validation check the motion.
candidatePairWorkLimit = 1e6;
[boundaryStart_units, ~] = obstacleAvoidance.geometry.boundaryToEdges(combinedSampleShape, 1e-12);
maximumCandidateNodeCount = max(2, ...
    floor(sqrt(2 * candidatePairWorkLimit / max(1, size(boundaryStart_units, 1)))) - 2);
if size(candidateNodes_units, 1) > maximumCandidateNodeCount
    candidateNodes_units = selectBoundaryNodes( ...
        candidateNodes_units, start_units, goal_units, maximumCandidateNodeCount);
end
nodes_units = unique([start_units; goal_units; candidateNodes_units], "rows", "stable");
end

%% Section 3: Local Functions

function selectedNodes_units = selectBoundaryNodes( ...
        candidateNodes_units, start_units, goal_units, maximumCandidateNodeCount)
    % Combine points spaced through the boundary list, outermost points in
    % several directions, and points nearest each endpoint. These choices
    % spread candidates around the shape while keeping nearby access points.
    nodesPerEndpoint         = min(4, floor(maximumCandidateNodeCount / 6));
    extremeDirectionCount    = min(16, floor(maximumCandidateNodeCount / 3));
    boundaryOrderSampleCount = maximumCandidateNodeCount - extremeDirectionCount - 2 * nodesPerEndpoint;
    selectedNodeIndices      = unique(round(linspace(1, size(candidateNodes_units, 1), boundaryOrderSampleCount))).';

    % Dot products identify the farthest point in each sampled direction.
    if extremeDirectionCount > 0
        sampleAngles_rad        = (0:extremeDirectionCount - 1).' * (2 * pi / extremeDirectionCount);
        sampleDirections        = [cos(sampleAngles_rad), sin(sampleAngles_rad)];
        [~, extremeNodeIndices] = max(candidateNodes_units * sampleDirections.', [], 1);
        selectedNodeIndices     = [selectedNodeIndices; extremeNodeIndices(:)];
    end
    for endpointPosition_units = [start_units; goal_units].'
        [~, nearestNodeIndices] = sort(vecnorm(candidateNodes_units - endpointPosition_units.', 2, 2));
        selectedNodeIndices     = [selectedNodeIndices; nearestNodeIndices(1:nodesPerEndpoint)]; %#ok<AGROW>
    end

    % A point can be chosen more than once; keep its first occurrence.
    selectedNodeIndices = unique(selectedNodeIndices, "stable");
    selectedNodes_units = candidateNodes_units( ...
        selectedNodeIndices(1:min(maximumCandidateNodeCount, numel(selectedNodeIndices))), :);
end
