function handles = plotProposalGeometry(proposal, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   handles = obstacleAvoidance.plotting.plotProposalGeometry(proposal)
%   handles = obstacleAvoidance.plotting.plotProposalGeometry( ...
%       proposal, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot the retained sampled-union or dense-envelope route proposal.
%**************************************************************************
% INPUTS
%   - proposal (scalar proposal-geometry struct)
%       Contains endpoints, representation, shape, and retained edges.
%   - optionOverrides (scalar struct, optional; default struct())
%       Supports FigureVisible and Title.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Figure, axes, proposal, and endpoint graphics handles.
%**************************************************************************
% UNITS
%   - Position is degrees and proposal sample times are seconds.
%**************************************************************************

%% Section 1: Check The Returned Stage

if nargin < 2
    optionOverrides = struct();
end
requiredFields = ["start_deg", "goal_deg", "representation", "shape"];
if ~isstruct(proposal) || ~isscalar(proposal) || ...
        ~all(isfield(proposal, cellstr(requiredFields)))
    error("plotProposalGeometry:InvalidProposal", ...
        "proposal must be a scalar proposal-geometry record.");
end
[figureHandle, axesHandle, options] = ...
    obstacleAvoidance.plotting.createStageAxes( ...
    "Route proposal geometry", optionOverrides);

%% Section 2: Plot The Retained Proposal

shapeHandle = gobjects(0);
if ~isempty(proposal.shape.Vertices)
    shapeHandle = plot(axesHandle, proposal.shape, ...
        "FaceColor", [0.8 0.3 0.2], "FaceAlpha", 0.18, ...
        "DisplayName", "Proposal exclusion geometry");
end
endpointHandles = plot(axesHandle, ...
    [proposal.start_deg(1), proposal.goal_deg(1)], ...
    [proposal.start_deg(2), proposal.goal_deg(2)], "o", ...
    "LineStyle", "none", "DisplayName", "Start and goal");
subtitle(axesHandle, sprintf("%s | samples %d", ...
    proposal.representation, proposal.sampledShapeCount));
legend(axesHandle, "Location", "best");
handles = struct("Figure", figureHandle, "Axes", axesHandle, ...
    "ShapeHandle", shapeHandle, "EndpointHandles", endpointHandles, ...
    "Options", options);
end
