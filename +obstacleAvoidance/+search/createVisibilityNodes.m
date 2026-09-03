function nodes = createVisibilityNodes( ...
        shape, start_deg, goal_deg, limits, candidateOffset_deg, ...
        candidateLimit)
%% Section 0: Header & Readme
% SYNTAX
%   nodes = obstacleAvoidance.search.createVisibilityNodes( ...
%       shape, start_deg, goal_deg, limits, candidateOffset_deg, ...
%       candidateLimit)
%**************************************************************************
% PURPOSE
%   - Offset proposal boundaries and retain affordable workspace nodes.
%   - Expose raw nodes and discard reasons for graph diagnosis.
%**************************************************************************
% INPUTS
%   - shape (scalar polyshape)
%       Spatial obstacle representation used only to propose routes.
%   - start_deg, goal_deg (1-by-2 finite numeric rows)
%       Required endpoint positions in [azimuth elevation] order.
%   - limits (scalar struct)
%       Workspace intervals in degrees.
%   - candidateOffset_deg (nonnegative finite scalar)
%       Outward boundary offset used for this attempt.
%   - candidateLimit (integer scalar greater than or equal to 2)
%       Maximum number of obstacle-derived nodes to retain.
%**************************************************************************
% OUTPUTS
%   - nodes (scalar struct)
%       Offset shape, raw nodes, retained nodes, discard reasons, and the
%       complete graph-node array with start and goal first.
%**************************************************************************
% UNITS
%   - Positions and candidateOffset_deg are degrees.
%**************************************************************************

%% Section 1: Offset The Proposal Boundary

% Visibility edges must pass outside the proposal obstacle. Offset its
% boundary before selecting nodes so later segment checks use clearance from
% the occupied shape rather than vertices lying exactly on that shape.

candidateShape = shape;
if ~isempty(shape.Vertices)
    candidateShape = polybuffer( ...
        shape, candidateOffset_deg, "JointType", "miter");
end
rawNodes_deg = candidateShape.Vertices;

%% Section 2: Retain Workspace Nodes

% Nodes outside the physical workspace cannot lead to a valid motion. Mark
% those discards before stable de-duplication and work-bounded selection so
% an attempt remains explainable without changing the historical node order.

isInsideWorkspace = ...
    rawNodes_deg(:, 1) >= limits.azimuthInterval_deg(1) & ...
    rawNodes_deg(:, 1) <= limits.azimuthInterval_deg(2) & ...
    rawNodes_deg(:, 2) >= limits.elevationInterval_deg(1) & ...
    rawNodes_deg(:, 2) <= limits.elevationInterval_deg(2);
discardReasons = repmat("", size(rawNodes_deg, 1), 1);
discardReasons(~isInsideWorkspace) = "outsideWorkspace";
candidateNodes_deg = unique( ...
    rawNodes_deg(isInsideWorkspace, :), "rows", "stable");
if size(candidateNodes_deg, 1) > candidateLimit
    candidateNodes_deg = selectVisibilityCandidates( ...
        candidateNodes_deg, start_deg, goal_deg, candidateLimit);
end
positions_deg = unique( ...
    [start_deg; goal_deg; candidateNodes_deg], "rows", "stable");

%% Section 3: Assemble Node Details

nodes = struct( ...
    "CandidateShape", candidateShape, ...
    "RawNodes_deg", rawNodes_deg, ...
    "RawNodeDiscardReasons", discardReasons, ...
    "RetainedCandidateNodes_deg", candidateNodes_deg, ...
    "Positions_deg", positions_deg, ...
    "CandidateLimit", candidateLimit, ...
    "CandidateOffset_deg", candidateOffset_deg);
end

%% Section 4: Local Functions

function selected_deg = selectVisibilityCandidates( ...
        candidates_deg, start_deg, goal_deg, count)
% Retain global supports, endpoint access, and uniform boundary coverage.
endpointCount = min(4, floor(count / 6));
directionCount = min(16, floor(count / 3));
uniformCount = count - directionCount - 2 * endpointCount;
selected = unique(round(linspace( ...
    1, size(candidates_deg, 1), uniformCount))).';
if directionCount > 0
    angle_rad = (0:directionCount - 1).' * (2 * pi / directionCount);
    direction = [cos(angle_rad), sin(angle_rad)];
    [~, support] = max(candidates_deg * direction.', [], 1);
    selected = [selected; support(:)];
end
for reference_deg = [start_deg; goal_deg].'
    [~, order] = sort(vecnorm( ...
        candidates_deg - reference_deg.', 2, 2));
    selected = [selected; order(1:endpointCount)]; %#ok<AGROW>
end
selected = unique(selected, "stable");
selected_deg = candidates_deg( ...
    selected(1:min(count, numel(selected))), :);
end
