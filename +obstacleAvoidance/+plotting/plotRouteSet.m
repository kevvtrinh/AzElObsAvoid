function handles = plotRouteSet(routeSet, scene, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   handles = obstacleAvoidance.plotting.plotRouteSet(routeSet, scene)
%   handles = obstacleAvoidance.plotting.plotRouteSet( ...
%       routeSet, scene, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot retained timed and distinct spatial route suggestions.
%**************************************************************************
% INPUTS
%   - routeSet (scalar route-set struct)
%       Contains timed and spatial routes returned by route search.
%   - scene (scalar prepared-scene struct)
%       Supplies retained prepared obstacle samples for context.
%   - optionOverrides (scalar struct, optional; default struct())
%       Supports FigureVisible and Title.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Figure, axes, and route graphics handles.
%**************************************************************************
% UNITS
%   - Position is degrees and timed-route values are seconds.
%**************************************************************************

%% Section 1: Check The Returned Stages

if nargin < 3
    optionOverrides = struct();
end
requiredRouteFields = ["TimedRoute_deg", "SpatialRoutes_deg"];
if ~isstruct(routeSet) || ~isscalar(routeSet) || ...
        ~all(isfield(routeSet, cellstr(requiredRouteFields))) || ...
        ~isstruct(scene) || ~isscalar(scene)
    error("plotRouteSet:InvalidStage", ...
        "routeSet and scene must be completed stage records.");
end
[figureHandle, axesHandle, options] = ...
    obstacleAvoidance.plotting.createStageAxes( ...
    "Searched route set", optionOverrides);

%% Section 2: Plot Returned Routes

sceneHandles = obstacleAvoidance.plotting.plotSceneSamples( ...
    axesHandle, scene);
routeHandles = gobjects(0);
if ~isempty(routeSet.TimedRoute_deg)
    routeHandles(end + 1, 1) = plot(axesHandle, ...
        routeSet.TimedRoute_deg(:, 1), routeSet.TimedRoute_deg(:, 2), ...
        "-o", "LineWidth", 2, "DisplayName", "Timed route");
end
for routeIndex = 1:numel(routeSet.SpatialRoutes_deg)
    route_deg = routeSet.SpatialRoutes_deg{routeIndex};
    routeHandles(end + 1, 1) = plot(axesHandle, ...
        route_deg(:, 1), route_deg(:, 2), "--", ...
        "DisplayName", "Spatial route " + routeIndex); %#ok<AGROW>
end
subtitle(axesHandle, sprintf("timed %d | spatial classes %d", ...
    ~isempty(routeSet.TimedRoute_deg), numel(routeSet.SpatialRoutes_deg)));
if ~isempty(routeHandles)
    legend(axesHandle, "Location", "best");
end
handles = struct("Figure", figureHandle, "Axes", axesHandle, ...
    "SceneHandles", sceneHandles, "RouteHandles", routeHandles, ...
    "Options", options);
end
