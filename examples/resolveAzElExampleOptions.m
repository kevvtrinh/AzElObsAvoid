function [plannerOptions, displayOptions] = ...
        resolveAzElExampleOptions(exampleOverrides, scenarioDefaults)
%% Section 0: Header & Readme
% SYNTAX
%   [plannerOptions, displayOptions] = ...
%       resolveAzElExampleOptions(exampleOverrides, scenarioDefaults)
%**************************************************************************
% PURPOSE
%   - Resolve uniform example display controls and public planner overrides.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct or []; empty uses defaults)
%   - scenarioDefaults (scalar partial planAzElMotion options struct)
%**************************************************************************
% OUTPUTS
%   - plannerOptions (resolved planner options for the scenario)
%   - displayOptions (resolved plotAzElMotion options plus PlotOutputs)
%**************************************************************************
% UNITS
%   - Pause_s is seconds. Planner values retain their documented units.
%**************************************************************************

%% Section 1: Resolve Uniform Display Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
if nargin < 2 || isempty(scenarioDefaults)
    scenarioDefaults = struct();
end
if ~isstruct(exampleOverrides) || ~isscalar(exampleOverrides) || ...
        ~isstruct(scenarioDefaults) || ~isscalar(scenarioDefaults)
    error("resolveAzElExampleOptions:InvalidOptions", ...
        "exampleOverrides and scenarioDefaults must be scalar structs.");
end
displayDefaults = struct( ...
    "PlotOutputs", true, ...
    "FigureVisible", "on", ...
    "ShowWorkspace", true, ...
    "ShowKinematics", true, ...
    "ShowAnimation", true, ...
    "AnimationFrameStride", 4, ...
    "Pause_s", 0.01);
displayOptions = displayDefaults;
displayNames = string(fieldnames(displayDefaults));
for name = displayNames.'
    if isfield(exampleOverrides, name) && ~isempty(exampleOverrides.(name))
        displayOptions.(name) = exampleOverrides.(name);
    end
end
logicalNames = ["PlotOutputs", "ShowWorkspace", ...
    "ShowKinematics", "ShowAnimation"];
for name = logicalNames
    displayOptions.(name) = azElInternal.normalizeLogicalScalar( ...
        displayOptions.(name), name, ...
        "resolveAzElExampleOptions:InvalidLogicalOption");
end
displayOptions.FigureVisible = lower(string(displayOptions.FigureVisible));
if ~any(displayOptions.FigureVisible == ["on", "off"])
    error("resolveAzElExampleOptions:InvalidFigureVisible", ...
        "FigureVisible must be 'on' or 'off'.");
end
validateattributes(displayOptions.AnimationFrameStride, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(displayOptions.Pause_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});

%% Section 2: Forward Only Public Planner Options

plannerOptions = scenarioDefaults;
plannerNames = string(fieldnames(planAzElMotion()));
overrideNames = string(fieldnames(exampleOverrides));
unknownNames = setdiff(overrideNames, [plannerNames; displayNames], "stable");
if ~isempty(unknownNames)
    warning("resolveAzElExampleOptions:UnknownOptions", ...
        "Ignoring unknown example fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
for name = intersect(overrideNames, plannerNames, "stable").'
    if ~isempty(exampleOverrides.(name))
        plannerOptions.(name) = exampleOverrides.(name);
    end
end
displayOptions = rmfield(displayOptions, "PlotOutputs");
displayOptions.PlotOutputs = displayDefaults.PlotOutputs;
if isfield(exampleOverrides, "PlotOutputs") && ...
        ~isempty(exampleOverrides.PlotOutputs)
    displayOptions.PlotOutputs = azElInternal.normalizeLogicalScalar( ...
        exampleOverrides.PlotOutputs, "PlotOutputs", ...
        "resolveAzElExampleOptions:InvalidPlotOutputs");
end
end
