function [plannerOptions, displayOptions] = resolveExampleOptions(exampleOverrides, scenarioDefaults, defaultMaxJerk_units_s3)
%% Section 0: Header & Readme
% SYNTAX
%   [plannerOptions, displayOptions] = resolveExampleOptions(exampleOverrides, scenarioDefaults)
%   [plannerOptions, displayOptions] = resolveExampleOptions( ...
%       exampleOverrides, scenarioDefaults, defaultMaxJerk_units_s3)
%
% PURPOSE
%   - Resolve uniform example display/runtime controls and planner overrides.
%   - Accept the maintained main-branch display names through one mapping.
%
% INPUTS
%   - exampleOverrides (scalar struct or []; empty uses defaults)
%       Display controls and public planTrajectory options are accepted.
%   - scenarioDefaults (scalar partial planTrajectory options struct)
%   - defaultMaxJerk_units_s3 (positive two-element vector)
%       Optional default is [2.5 2.5]. MaxJerk_units_s3 can override it.
%       Examples use per-axis velocity and acceleration, so jerk must also
%       be a two-element [x y] limit.
%
% OUTPUTS
%   - plannerOptions (resolved planner options for the scenario)
%   - displayOptions (resolved example display/runtime controls)
%       PlotOptions contains only controls accepted by plotTrajectory.
%
% UNITS
%   - Pause_s is seconds. Planner values retain their documented units.
%

%% Section 1: Resolve Uniform Display Controls

% Keep display controls separate from planner controls. Display choices must not
% change route selection, collision checks, or motion feasibility.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
if nargin < 2 || isempty(scenarioDefaults)
    scenarioDefaults = struct();
end
if nargin < 3 || isempty(defaultMaxJerk_units_s3)
    defaultMaxJerk_units_s3 = [2.5 2.5];
end
if ~isstruct(exampleOverrides) || ~isscalar(exampleOverrides) || ~isstruct(scenarioDefaults) || ~isscalar(scenarioDefaults)
    error("resolveExampleOptions:InvalidOptions", "exampleOverrides and scenarioDefaults must be scalar structs.");
end
displayDefaults = struct();
displayDefaults.PlotOutputs                         = true;
displayDefaults.FigureVisible                       = "on";
displayDefaults.Title                               = "X/y motion plan";
displayDefaults.ShowWorkspace                       = true;
displayDefaults.ShowKinematics                      = true;
displayDefaults.ShowAnimation                       = true;
displayDefaults.ShowSearchEdges                     = true;
displayDefaults.ShowVisibilityGraphs                = true;
displayDefaults.Verbose                             = true;
displayDefaults.FrameStride                         = 4;
displayDefaults.Pause_s                             = 0.01;
displayDefaults.SaveAnimationGif                    = false;
displayDefaults.AnimationGifFile                    = "obstacleAvoidanceTrajectory.gif";
displayDefaults.AnimationGifDelay_s                 = 0.01;
displayDefaults.ShowSweptSurfaces                   = true;
displayDefaults.MaximumDisplayedSlicesPerObstacle   = 30;
displayDefaults.MaximumDisplayedVisibilitySnapshots = 30;
normalizedOverrides = normalizeDisplayAliases(exampleOverrides);
displayOptions      = displayDefaults;
displayNames        = string(fieldnames(displayDefaults));

% Apply scenario display defaults first. Caller values can replace them later.
for name = intersect(string(fieldnames(scenarioDefaults)), displayNames, "stable").'
    if ~isempty(scenarioDefaults.(name))
        displayOptions.(name) = scenarioDefaults.(name);
    end
end

% Apply each recognized caller display control.
for name = displayNames.'
    if isfield(normalizedOverrides, name) && ~isempty(normalizedOverrides.(name))
        displayOptions.(name) = normalizedOverrides.(name);
    end
end
maxJerk_units_s3 = defaultMaxJerk_units_s3;
if isfield(normalizedOverrides, "MaxJerk_units_s3") && ~isempty(normalizedOverrides.MaxJerk_units_s3)
    maxJerk_units_s3 = normalizedOverrides.MaxJerk_units_s3;
end
validateattributes(maxJerk_units_s3, {'numeric'}, {'real', 'finite', 'positive', 'nonempty'});
if ~isvector(maxJerk_units_s3) || numel(maxJerk_units_s3) ~= 2
    error("resolveExampleOptions:InvalidMaxJerk", "MaxJerk_units_s3 must be a two-element [x y] limit because examples use per-axis velocity and acceleration limits.");
end
maxJerk_units_s3 = reshape(double(maxJerk_units_s3), 1, []);
logicalNames = ["PlotOutputs", "ShowWorkspace", "ShowKinematics", ...
    "ShowAnimation", "ShowSearchEdges", "ShowVisibilityGraphs", ...
    "ShowSweptSurfaces", "SaveAnimationGif", "Verbose"];

% Convert each display toggle to one true or false value.
for name = logicalNames
    displayOptions.(name) = obstacleAvoidance.input.normalizeLogicalScalar(displayOptions.(name), name, "resolveExampleOptions:InvalidLogicalOption");
end
displayOptions.FigureVisible    = lower(string(displayOptions.FigureVisible));
displayOptions.Title            = string(displayOptions.Title);
displayOptions.AnimationGifFile = string(displayOptions.AnimationGifFile);
if ~isscalar(displayOptions.FigureVisible) || ~any(displayOptions.FigureVisible == ["on", "off"])
    error("resolveExampleOptions:InvalidFigureVisible", "FigureVisible must be 'on' or 'off'.");
end
if ~isscalar(displayOptions.Title)
    error("resolveExampleOptions:InvalidTitle", "Title must be scalar text.");
end
if ~isscalar(displayOptions.AnimationGifFile) || strlength(displayOptions.AnimationGifFile) == 0
    error("resolveExampleOptions:InvalidAnimationGifFile", "AnimationGifFile must be nonempty scalar text.");
end
validateattributes(displayOptions.FrameStride, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(displayOptions.Pause_s, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(displayOptions.AnimationGifDelay_s, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
displayCountNames = ["MaximumDisplayedSlicesPerObstacle", "MaximumDisplayedVisibilitySnapshots"];

% Require positive integer limits for both displayed-slice controls.
for name = displayCountNames
    validateattributes(displayOptions.(name), {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
end

%% Section 2: Forward Only Public Planner Options

% Get the maintained planner defaults before scenario values are applied.
% A list of public field names prevents display-only values from reaching the
% planner and producing an unknown-option warning.
plannerOptions = struct("GoalTimeMode", "earliestArrival", "FixedArrivalSearch", "spatial", "SampleTime_s", 0.05, ...
    "ConstraintTolerance", 1e-8, "CollisionClearanceTolerance_units", 1e-7, ...
    "ArrivalTimeTolerance_s", 1e-8, "WrapX", false, "WrapY", false, ...
    "MatchTargetVelocity",false,"MatchTargetAcceleration",false, ...
    "TemporalResolution_s",0.5,"MaxArrivalTrials",100,"PathLengthTimeAllowance_s",0.49);
plannerNames   = string(fieldnames(plannerOptions));

% Apply recognized scenario planner defaults. Ignore display-only fields here.
for name = intersect(string(fieldnames(scenarioDefaults)), plannerNames, "stable").'
    if ~isempty(scenarioDefaults.(name))
        plannerOptions.(name) = scenarioDefaults.(name);
    end
end
overrideNames        = string(fieldnames(normalizedOverrides));
aliasNames           = ["ShowKinematicPlot", "AnimationFrameStride", "AnimationPause_s", "MaxJerk_units_s3"];
scenarioNames        = string(fieldnames(scenarioDefaults));
unknownNames         = setdiff(overrideNames, [plannerNames; displayNames; aliasNames.'], "stable");
unknownScenarioNames = setdiff(scenarioNames(:), [plannerNames; displayNames(:)], "stable");
unknownNames         = unique([unknownNames(:); unknownScenarioNames(:)], "stable");
if ~isempty(unknownNames)
    warning("resolveExampleOptions:UnknownOptions", "Ignoring unknown example fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end

% Apply recognized caller planner values after scenario defaults.
for name = intersect(overrideNames, plannerNames, "stable").'
    if ~isempty(normalizedOverrides.(name))
        plannerOptions.(name) = normalizedOverrides.(name);
    end
end
plotOptions = rmfield(displayOptions, ["PlotOutputs", "Verbose"]);
displayOptions.JerkConstraintEnabled          = true;
displayOptions.MaxJerk_units_s3                 = maxJerk_units_s3;
displayOptions.ConfiguredFiniteMaxJerk_units_s3 = maxJerk_units_s3;
displayOptions.PlotOptions                    = plotOptions;
end


function normalized = normalizeDisplayAliases(overrides)
    % Map maintained example names to the names used by the plotting function.
    normalized = overrides;
    aliases    = [ ...
        "ShowKinematicPlot", "ShowKinematics"; "AnimationFrameStride", "FrameStride"; "AnimationPause_s", "Pause_s"];

    % Translate each supported old display name to its current field name.
    for aliasIndex = 1:size(aliases, 1)
        oldName = aliases(aliasIndex, 1);
        newName = aliases(aliasIndex, 2);
        if isfield(normalized, oldName) && ~isfield(normalized, newName)
            normalized.(newName) = normalized.(oldName);
        end
    end
end
