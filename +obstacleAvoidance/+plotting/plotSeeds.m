function handles = plotSeeds(seedSet, scene, visibilityGraph, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   handles = obstacleAvoidance.plotting.plotSeeds( ...
%       seedSet, scene, visibilityGraph)
%   handles = obstacleAvoidance.plotting.plotSeeds( ...
%       seedSet, scene, visibilityGraph, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot ordered motion seeds before any seed is solved.
%**************************************************************************
% INPUTS
%   - seedSet (struct array)
%       Ordered seed records with position_deg, Source, and Index.
%   - scene, visibilityGraph (scalar structs)
%       Prepared-scene and graph context retained by prior stages.
%   - optionOverrides (scalar struct, optional; default struct())
%       Supports FigureVisible and Title.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Figure, axes, and seed graphics handles.
%**************************************************************************
% UNITS
%   - Seed positions are degrees.
%**************************************************************************

%% Section 1: Check The Returned Stages

if nargin < 4
    optionOverrides = struct();
end
requiredSeedFields = ["position_deg", "Source", "Index"];
if ~isstruct(seedSet) || (~isempty(seedSet) && ...
        ~all(isfield(seedSet, cellstr(requiredSeedFields)))) || ...
        ~isstruct(scene) || ~isstruct(visibilityGraph)
    error("plotSeeds:InvalidStage", ...
        "seedSet, scene, and visibilityGraph must be returned stage data.");
end
[figureHandle, axesHandle, options] = ...
    obstacleAvoidance.plotting.createStageAxes( ...
    "Ordered motion seeds", optionOverrides);

%% Section 2: Plot Ordered Seeds

sceneHandles = obstacleAvoidance.plotting.plotSceneSamples( ...
    axesHandle, scene);
seedHandles = gobjects(0);
for seedIndex = 1:numel(seedSet)
    position_deg = seedSet(seedIndex).position_deg;
    seedHandles(end + 1, 1) = plot(axesHandle, ...
        position_deg(:, 1), position_deg(:, 2), "-o", ...
        "DisplayName", sprintf("%d: %s", seedSet(seedIndex).Index, ...
        seedSet(seedIndex).Source)); %#ok<AGROW>
end
subtitle(axesHandle, sprintf("seeds %d | graph connected %d", ...
    numel(seedSet), logical(visibilityGraph.IsConnected)));
if ~isempty(seedHandles)
    legend(axesHandle, "Location", "best");
end
handles = struct("Figure", figureHandle, "Axes", axesHandle, ...
    "SceneHandles", sceneHandles, "SeedHandles", seedHandles, ...
    "Options", options);
end
