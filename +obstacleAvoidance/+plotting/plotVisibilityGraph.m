function handles = plotVisibilityGraph( ...
        visibilityGraph, proposal, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   handles = obstacleAvoidance.plotting.plotVisibilityGraph( ...
%       visibilityGraph, proposal)
%   handles = obstacleAvoidance.plotting.plotVisibilityGraph( ...
%       visibilityGraph, proposal, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot retained and rejected visibility work for every offset attempt.
%**************************************************************************
% INPUTS
%   - visibilityGraph (scalar visibility-graph struct)
%       Contains attempts, selected nodes, edges, and connectivity.
%   - proposal (scalar proposal-geometry struct)
%       Supplies the exact spatial exclusion shape used by the attempts.
%   - optionOverrides (scalar struct, optional; default struct())
%       Supports FigureVisible and Title.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Figure, axes, node, accepted-edge, and rejected-edge handles.
%**************************************************************************
% UNITS
%   - Position and candidate offsets are degrees.
%**************************************************************************

%% Section 1: Check The Returned Stages

if nargin < 3
    optionOverrides = struct();
end
requiredGraphFields = ["Attempts", "IsConnected"];
if ~isstruct(visibilityGraph) || ~isscalar(visibilityGraph) || ...
        ~all(isfield(visibilityGraph, cellstr(requiredGraphFields))) || ...
        ~isstruct(proposal) || ~isscalar(proposal) || ...
        ~isfield(proposal, "shape")
    error("plotVisibilityGraph:InvalidStage", ...
        "visibilityGraph and proposal must be completed stage records.");
end
[figureHandle, axesHandle, options] = ...
    obstacleAvoidance.plotting.createStageAxes( ...
    "Visibility graph attempts", optionOverrides);

%% Section 2: Plot Every Attempt

if ~isempty(proposal.shape.Vertices)
    plot(axesHandle, proposal.shape, "FaceColor", [0.7 0.7 0.7], ...
        "FaceAlpha", 0.12, "DisplayName", "Proposal geometry");
end
nodeHandles = gobjects(0);
acceptedHandles = gobjects(0);
rejectedHandles = gobjects(0);
for attemptIndex = 1:numel(visibilityGraph.Attempts)
    attempt = visibilityGraph.Attempts(attemptIndex);
    if isfield(attempt.Nodes, "RawNodes_deg") && ...
            ~isempty(attempt.Nodes.RawNodes_deg)
        raw_deg = attempt.Nodes.RawNodes_deg;
        nodeHandles(end + 1, 1) = scatter(axesHandle, ...
            raw_deg(:, 1), raw_deg(:, 2), 12, "x", ...
            "DisplayName", "Attempt " + attemptIndex + " raw nodes"); %#ok<AGROW>
    end
    acceptedHandles = appendEdges(axesHandle, acceptedHandles, ...
        attempt.AcceptedEdges_deg, "-", ...
        "Attempt " + attemptIndex + " accepted");
    rejectedHandles = appendEdges(axesHandle, rejectedHandles, ...
        attempt.RejectedEdges_deg, ":", ...
        "Attempt " + attemptIndex + " rejected");
end
subtitle(axesHandle, sprintf("attempts %d | connected %d", ...
    numel(visibilityGraph.Attempts), visibilityGraph.IsConnected));
legend(axesHandle, "Location", "best");
handles = struct("Figure", figureHandle, "Axes", axesHandle, ...
    "NodeHandles", nodeHandles, "AcceptedEdgeHandles", acceptedHandles, ...
    "RejectedEdgeHandles", rejectedHandles, "Options", options);
end

%% Section 3: Local Functions

function lineHandles = appendEdges( ...
        axesHandle, lineHandles, edges_deg, style, displayName)
% Draw one retained edge category without reconstructing pair decisions.
if isempty(edges_deg)
    return;
end
edgeCount = size(edges_deg, 1);
azimuth_deg = reshape([edges_deg(:, [1 3]), nan(edgeCount, 1)].', [], 1);
elevation_deg = reshape([edges_deg(:, [2 4]), nan(edgeCount, 1)].', [], 1);
lineHandles(end + 1, 1) = plot(axesHandle, azimuth_deg, elevation_deg, ...
    style, "DisplayName", displayName);
end
