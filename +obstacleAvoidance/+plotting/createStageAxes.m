function [figureHandle, axesHandle, options] = createStageAxes( ...
        defaultTitle, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   [figureHandle, axesHandle, options] = ...
%       obstacleAvoidance.plotting.createStageAxes(defaultTitle)
%   [figureHandle, axesHandle, options] = ...
%       obstacleAvoidance.plotting.createStageAxes( ...
%       defaultTitle, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Create consistently configured axes for read-only stage inspectors.
%**************************************************************************
% INPUTS
%   - defaultTitle (nonempty scalar text)
%       Title used when optionOverrides does not supply Title.
%   - optionOverrides (scalar struct, optional; default struct())
%       Supports FigureVisible and Title.
%**************************************************************************
% OUTPUTS
%   - figureHandle, axesHandle (scalar graphics handles)
%       New figure and explicitly owned spatial axes.
%   - options (scalar struct)
%       Resolved display controls.
%**************************************************************************
% UNITS
%   - Spatial axes use degrees.
%**************************************************************************

%% Section 1: Resolve Display Controls

if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end
defaults = struct("FigureVisible", "on", "Title", string(defaultTitle));
[options, unknownNames] = obstacleAvoidance.input.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("createStageAxes:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
options.FigureVisible = lower(string(options.FigureVisible));
options.Title = string(options.Title);
if ~isscalar(options.FigureVisible) || ...
        ~any(options.FigureVisible == ["on", "off"])
    error("createStageAxes:InvalidFigureVisible", ...
        "FigureVisible must be 'on' or 'off'.");
end
if ~isscalar(options.Title) || ismissing(options.Title) || ...
        strlength(options.Title) == 0
    error("createStageAxes:InvalidTitle", ...
        "Title must be nonempty scalar text.");
end

%% Section 2: Create Spatial Axes

figureHandle = figure("Name", options.Title, ...
    "Visible", options.FigureVisible);
axesHandle = axes(figureHandle);
hold(axesHandle, "on");
grid(axesHandle, "on");
box(axesHandle, "on");
axis(axesHandle, "equal");
xlabel(axesHandle, "Azimuth (deg)");
ylabel(axesHandle, "Elevation (deg)");
title(axesHandle, options.Title);
end
