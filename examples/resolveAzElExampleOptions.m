function [plannerOptions, displayOptions] = resolveAzElExampleOptions( ...
        exampleOverrides, scenarioDefaults, defaultMaxJerk_deg_s3)
%% Section 0: Header & Readme
% SYNTAX
%   [plannerOptions, displayOptions] = resolveAzElExampleOptions(exampleOverrides, scenarioDefaults)
%   [plannerOptions, displayOptions] = resolveAzElExampleOptions( ...
%       exampleOverrides, scenarioDefaults, defaultMaxJerk_deg_s3)
%**************************************************************************
% PURPOSE
%   - Resolve uniform example display controls and public planner overrides.
%   - Accept the maintained main-branch display names through one mapping.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct or []; empty uses defaults)
%       Display controls and public planAzElMotion options are accepted.
%   - scenarioDefaults (scalar partial planAzElMotion options struct)
%   - defaultMaxJerk_deg_s3 (positive scalar or two-element vector)
%       Optional default is [2.5 2.5]. MaxJerk_deg_s3 can override it.
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
if nargin < 3 || isempty(defaultMaxJerk_deg_s3)
    defaultMaxJerk_deg_s3 = [2.5 2.5];
end
if ~isstruct(exampleOverrides) || ~isscalar(exampleOverrides) || ...
        ~isstruct(scenarioDefaults) || ~isscalar(scenarioDefaults)
    error("resolveAzElExampleOptions:InvalidOptions", "exampleOverrides and scenarioDefaults must be scalar structs.");
end
displayDefaults = struct( ...
    "PlotOutputs", true, ...
    "FigureVisible", "on", ...
    "Title", "Azimuth/elevation motion plan", ...
    "ShowWorkspace", true, ...
    "ShowKinematics", true, ...
    "ShowAnimation", true, ...
    "ShowSearchEdges", true, ...
    "ShowVisibilityGraphs", true, ...
    "FrameStride", 4, ...
    "Pause_s", 0.01, ...
    "ShowSweptSurfaces", true, "MaximumDisplayedSlicesPerObstacle", 30, "MaximumDisplayedVisibilitySnapshots", 30);
normalizedOverrides = normalizeDisplayAliases(exampleOverrides);
displayOptions = displayDefaults;
displayNames = string(fieldnames(displayDefaults));

for name = intersect(string(fieldnames(scenarioDefaults)), displayNames, "stable").'
    if ~isempty(scenarioDefaults.(name))
        displayOptions.(name) = scenarioDefaults.(name);
    end
end

for name = displayNames.'
    if isfield(normalizedOverrides, name) && ~isempty(normalizedOverrides.(name))
        displayOptions.(name) = normalizedOverrides.(name);
    end
end
maxJerk_deg_s3 = defaultMaxJerk_deg_s3;
if isfield(normalizedOverrides, "MaxJerk_deg_s3") && ~isempty(normalizedOverrides.MaxJerk_deg_s3)
    maxJerk_deg_s3 = normalizedOverrides.MaxJerk_deg_s3;
end
validateattributes(maxJerk_deg_s3, {'numeric'}, {'real', 'finite', 'positive', 'nonempty'});
if ~isscalar(maxJerk_deg_s3) && numel(maxJerk_deg_s3) ~= 2
    error("resolveAzElExampleOptions:InvalidMaxJerk", "MaxJerk_deg_s3 must be scalar or two-element.");
end
maxJerk_deg_s3 = reshape(double(maxJerk_deg_s3), 1, []);
if isscalar(maxJerk_deg_s3)
    maxJerk_deg_s3 = repmat(maxJerk_deg_s3, 1, 2);
end
logicalNames = ["PlotOutputs", "ShowWorkspace", "ShowKinematics", ...
    "ShowAnimation", "ShowSearchEdges", "ShowVisibilityGraphs", "ShowSweptSurfaces"];

for name = logicalNames
    displayOptions.(name) = azElInternal.normalizeLogicalScalar( ...
        displayOptions.(name), name, "resolveAzElExampleOptions:InvalidLogicalOption");
end
displayOptions.FigureVisible = lower(string(displayOptions.FigureVisible));
displayOptions.Title = string(displayOptions.Title);
if ~isscalar(displayOptions.FigureVisible) || ~any(displayOptions.FigureVisible == ["on", "off"])
    error("resolveAzElExampleOptions:InvalidFigureVisible", "FigureVisible must be 'on' or 'off'.");
end
if ~isscalar(displayOptions.Title)
    error("resolveAzElExampleOptions:InvalidTitle", "Title must be scalar text.");
end
validateattributes(displayOptions.FrameStride, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(displayOptions.Pause_s, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
displayCountNames = ["MaximumDisplayedSlicesPerObstacle", "MaximumDisplayedVisibilitySnapshots"];

for name = displayCountNames
    validateattributes(displayOptions.(name), {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
end

%% Section 2: Forward Only Public Planner Options

plannerOptions = planAzElMotion();
plannerNames = string(fieldnames(plannerOptions));

for name = intersect(string(fieldnames(scenarioDefaults)), plannerNames, "stable").'
    if ~isempty(scenarioDefaults.(name))
        plannerOptions.(name) = scenarioDefaults.(name);
    end
end
overrideNames = string(fieldnames(normalizedOverrides));
aliasNames = ["ShowKinematicPlot", "AnimationFrameStride", "AnimationPause_s", "MaxJerk_deg_s3"];
scenarioNames = string(fieldnames(scenarioDefaults));
unknownNames = setdiff(overrideNames, [plannerNames; displayNames; aliasNames.'], "stable");
unknownScenarioNames = setdiff(scenarioNames(:), [plannerNames(:); displayNames(:)], "stable");
unknownNames = unique( [unknownNames(:); unknownScenarioNames(:)], "stable");
if ~isempty(unknownNames)
    warning("resolveAzElExampleOptions:UnknownOptions", ...
        "Ignoring unknown example fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end

for name = intersect(overrideNames, plannerNames, "stable").'
    if ~isempty(normalizedOverrides.(name))
        plannerOptions.(name) = normalizedOverrides.(name);
    end
end
plotOptions = rmfield(displayOptions, "PlotOutputs");
displayOptions.JerkConstraintEnabled = true;
displayOptions.MaxJerk_deg_s3 = maxJerk_deg_s3;
displayOptions.ConfiguredFiniteMaxJerk_deg_s3 = maxJerk_deg_s3;
displayOptions.PlotOptions = plotOptions;
end

%% Section 3: Local Functions

function normalized = normalizeDisplayAliases(overrides)
% Map maintained example aliases to one plot-option vocabulary.
normalized = overrides;
aliases = [ ...
    "ShowKinematicPlot", "ShowKinematics"; "AnimationFrameStride", "FrameStride"; "AnimationPause_s", "Pause_s"];

for aliasIndex = 1:size(aliases, 1)
    oldName = aliases(aliasIndex, 1);
    newName = aliases(aliasIndex, 2);
    if isfield(normalized, oldName) && ~isfield(normalized, newName)
        normalized.(newName) = normalized.(oldName);
    end
end
end
