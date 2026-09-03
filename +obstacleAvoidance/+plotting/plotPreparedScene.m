function handles = plotPreparedScene(scene, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   handles = obstacleAvoidance.plotting.plotPreparedScene(scene)
%   handles = obstacleAvoidance.plotting.plotPreparedScene( ...
%       scene, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot prepared obstacle samples without constructing proposal geometry.
%**************************************************************************
% INPUTS
%   - scene (scalar prepared-scene struct)
%       Contains preparedObstacles and the request horizon.
%   - optionOverrides (scalar struct, optional; default struct())
%       Supports FigureVisible and Title.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Figure, axes, and obstacle-sample graphics handles.
%**************************************************************************
% UNITS
%   - Position is degrees and scene times are seconds.
%**************************************************************************

%% Section 1: Check The Returned Stage

if nargin < 2
    optionOverrides = struct();
end
requiredFields = ["preparedObstacles", "startTime_s", "endTime_s"];
if ~isstruct(scene) || ~isscalar(scene) || ...
        ~all(isfield(scene, cellstr(requiredFields)))
    error("plotPreparedScene:InvalidScene", ...
        "scene must be a scalar prepared-planning-scene record.");
end
[figureHandle, axesHandle, options] = ...
    obstacleAvoidance.plotting.createStageAxes( ...
    "Prepared obstacle histories", optionOverrides);

%% Section 2: Plot Retained Prepared Samples

obstacles = scene.preparedObstacles;
sampleHandles = obstacleAvoidance.plotting.plotSceneSamples( ...
    axesHandle, scene);
subtitle(axesHandle, sprintf("horizon %.3g to %.3g s | obstacles %d", ...
    scene.startTime_s, scene.endTime_s, numel(obstacles)));
if ~isempty(sampleHandles)
    legend(axesHandle, "Location", "best");
end
handles = struct("Figure", figureHandle, "Axes", axesHandle, ...
    "SampleHandles", sampleHandles, "Options", options);
end
