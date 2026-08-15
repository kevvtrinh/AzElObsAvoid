function [plannerOptions, jerkConfiguration] = ...
        resolveAzElExampleOptions(optionOverrides, plannerDefaults, ...
        defaultMaxJerk_deg_s3)
%% Section 0: Header & Readme
% SYNTAX
%   [plannerOptions, jerkConfiguration] = ...
%       resolveAzElExampleOptions(overrides, plannerDefaults, defaultJerk)
%**************************************************************************
% PURPOSE
%   - Give every maintained example one uniform jerk on/off interface.
%   - Keep example-only controls out of planAzElMotion options because jerk
%     is a physical limit, not a display or search option.
%   - Default every example to visible az/el planning, animation, and
%     kinematic figures while preserving explicit headless overrides.
%**************************************************************************
% INPUTS
%   - optionOverrides (scalar struct, optional; default struct())
%       Example-only controls include EnableJerkConstraint,
%       MaxJerk_deg_s3, FigureVisible, PlotOutputs, Title, ShowAnimation,
%       ShowKinematicPlot, and ShowVisibilityGraphs. Planner controls are
%       forwarded when known.
%   - plannerDefaults (scalar struct, optional; default struct())
%       Scenario defaults for planner and display controls.
%   - defaultMaxJerk_deg_s3 (positive scalar or two-element vector)
%       Finite jerk used when the constraint is enabled; default [2.5 2.5].
%**************************************************************************
% OUTPUTS
%   - plannerOptions (scalar struct)
%       Options safe to pass directly to planAzElMotion.
%   - jerkConfiguration (scalar struct)
%       JerkConstraintEnabled and MaxJerk_deg_s3 resolved for limits.
%**************************************************************************
% UNITS
%   - Jerk is degrees per second cubed.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(optionOverrides)
    optionOverrides = struct();
end
if nargin < 2 || isempty(plannerDefaults)
    plannerDefaults = struct();
end
if nargin < 3 || isempty(defaultMaxJerk_deg_s3)
    defaultMaxJerk_deg_s3 = [2.5 2.5];
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides) || ...
        ~isstruct(plannerDefaults) || ~isscalar(plannerDefaults)
    error("resolveAzElExampleOptions:InvalidOptions", ...
        "Overrides and plannerDefaults must be scalar structs.");
end

enableJerkConstraint = true;
if isfield(optionOverrides, "EnableJerkConstraint") && ...
        ~isempty(optionOverrides.EnableJerkConstraint)
    enableValue = optionOverrides.EnableJerkConstraint;
    validateattributes(enableValue, {'logical','numeric'}, ...
        {'real','finite','scalar'});
    if isnumeric(enableValue) && enableValue ~= 0 && enableValue ~= 1
        error("resolveAzElExampleOptions:InvalidEnableJerkConstraint", ...
            "EnableJerkConstraint must be logical or numeric 0 or 1.");
    end
    enableJerkConstraint = logical( ...
        enableValue);
end
maxJerk_deg_s3 = defaultMaxJerk_deg_s3;
if isfield(optionOverrides, "MaxJerk_deg_s3") && ...
        ~isempty(optionOverrides.MaxJerk_deg_s3)
    maxJerk_deg_s3 = optionOverrides.MaxJerk_deg_s3;
end
validateattributes(maxJerk_deg_s3, {'numeric'}, ...
    {'real','finite','positive','nonempty'});
if ~isscalar(maxJerk_deg_s3) && numel(maxJerk_deg_s3) ~= 2
    error("resolveAzElExampleOptions:InvalidMaxJerk", ...
        "MaxJerk_deg_s3 must be scalar or two-element.");
end
maxJerk_deg_s3 = reshape(double(maxJerk_deg_s3), 1, []);
if isscalar(maxJerk_deg_s3)
    maxJerk_deg_s3 = repmat(maxJerk_deg_s3, 1, 2);
end

figureVisible = fieldValue(optionOverrides, plannerDefaults, ...
    "FigureVisible", "on");
figureVisible = lower(string(figureVisible));
if ~isscalar(figureVisible) || ~any(figureVisible == ["on" "off"])
    error("resolveAzElExampleOptions:InvalidFigureVisible", ...
        "FigureVisible must be on or off.");
end
plotOutputs = logicalExampleControl(fieldValue( ...
    optionOverrides, plannerDefaults, "PlotOutputs", true), ...
    "PlotOutputs");
titleText = string(fieldValue(optionOverrides, plannerDefaults, ...
    "Title", "Azimuth/elevation motion plan"));
if ~isscalar(titleText)
    error("resolveAzElExampleOptions:InvalidTitle", ...
        "Title must be scalar text.");
end
showAnimation = logicalExampleControl(fieldValue( ...
    optionOverrides, plannerDefaults, "ShowAnimation", true), ...
    "ShowAnimation");
showKinematics = logicalExampleControl(fieldValue( ...
    optionOverrides, plannerDefaults, "ShowKinematicPlot", true), ...
    "ShowKinematicPlot");
showVisibilityGraphs = logicalExampleControl(fieldValue( ...
    optionOverrides, plannerDefaults, "ShowVisibilityGraphs", true), ...
    "ShowVisibilityGraphs");
showSweptSurfaces = logicalExampleControl(fieldValue( ...
    optionOverrides, plannerDefaults, "ShowSweptSurfaces", true), ...
    "ShowSweptSurfaces");
plotOptions = struct( ...
    "FigureVisible", figureVisible, "Title", titleText, ...
    "ShowAnimation", showAnimation, ...
    "ShowKinematics", showKinematics, ...
    "ShowVisibilityGraphs", showVisibilityGraphs, ...
    "FrameStride", fieldValue(optionOverrides, plannerDefaults, ...
        "AnimationFrameStride", 10), ...
    "Pause_s", fieldValue(optionOverrides, plannerDefaults, ...
        "AnimationPause_s", 0.001), ...
    "ShowSweptSurfaces", showSweptSurfaces, ...
    "MaximumDisplayedSlicesPerObstacle", fieldValue( ...
        optionOverrides, plannerDefaults, ...
        "MaximumDisplayedSlicesPerObstacle", 30), ...
    "MaximumDisplayedVisibilitySnapshots", fieldValue( ...
        optionOverrides, plannerDefaults, ...
        "MaximumDisplayedVisibilitySnapshots", 30));

%% Section 2: Forward Supported Planner Options

plannerSupported = string(fieldnames(planAzElMotion()));
plannerOptions = struct();
defaultNames = intersect(string(fieldnames(plannerDefaults)), ...
    plannerSupported, "stable");
for defaultNameIndex = 1:numel(defaultNames)
    name = defaultNames(defaultNameIndex);
    plannerOptions.(name) = plannerDefaults.(name);
end
exampleOnlyNames = ["EnableJerkConstraint" "MaxJerk_deg_s3" ...
    "FigureVisible" "PlotOutputs" "Title" "ShowAnimation" ...
    "ShowKinematicPlot" "ShowVisibilityGraphs" "ShowSweptSurfaces" ...
    "AnimationFrameStride" "AnimationPause_s" ...
    "MaximumDisplayedSlicesPerObstacle" ...
    "MaximumDisplayedVisibilitySnapshots"];
recognizedNames = [plannerSupported; exampleOnlyNames(:)];
unknownNames = unique([setdiff( ...
    string(fieldnames(plannerDefaults)), recognizedNames, "stable"); ...
    setdiff(string(fieldnames(optionOverrides)), ...
    recognizedNames, "stable")], "stable");
if ~isempty(unknownNames)
    warning("resolveAzElExampleOptions:UnknownOptions", ...
        "Ignoring unknown example option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
overrideNames = intersect(string(fieldnames(optionOverrides)), ...
    plannerSupported, "stable");
for overrideNameIndex = 1:numel(overrideNames)
    name = overrideNames(overrideNameIndex);
    if ~isempty(optionOverrides.(name))
        plannerOptions.(name) = optionOverrides.(name);
    end
end

%% Section 3: Assemble Example Configuration

resolvedJerk_deg_s3 = [Inf Inf];
if enableJerkConstraint
    resolvedJerk_deg_s3 = maxJerk_deg_s3;
end
jerkConfiguration = struct( ...
    "JerkConstraintEnabled", enableJerkConstraint, ...
    "MaxJerk_deg_s3", resolvedJerk_deg_s3, ...
    "ConfiguredFiniteMaxJerk_deg_s3", maxJerk_deg_s3, ...
    "FigureVisible", figureVisible, ...
    "PlotOutputs", plotOutputs, ...
    "Title", titleText, ...
    "ShowAnimation", showAnimation, ...
    "ShowKinematicPlot", showKinematics, ...
    "ShowVisibilityGraphs", showVisibilityGraphs, ...
    "PlotOptions", plotOptions);
end

%% Section 4: Local Functions

function value = fieldValue(overrides, defaults, name, fallback)
%% Section 0: Header & Readme
% SYNTAX
%   value = fieldValue(overrides, defaults, name, fallback)
%**************************************************************************
% PURPOSE
%   - Resolve one example-only control without forwarding it.
%**************************************************************************
% INPUTS
%   - overrides, defaults (scalar structs)
%   - name (scalar string)
%   - fallback (value of any supported option type)
%**************************************************************************
% OUTPUTS
%   - value (resolved option value)
%**************************************************************************
% UNITS
%   - Units are inherited from the named field.
%**************************************************************************
value = fallback;
if isfield(defaults, name) && ~isempty(defaults.(name))
    value = defaults.(name);
end
if isfield(overrides, name) && ~isempty(overrides.(name))
    value = overrides.(name);
end
end

function value = logicalExampleControl(value, name)
%% Section 0: Header & Readme
% SYNTAX
%   value = logicalExampleControl(value, name)
%**************************************************************************
% PURPOSE
%   - Validate and normalize one logical example control.
%**************************************************************************
% INPUTS
%   - value (scalar logical or binary numeric value)
%   - name (scalar text)
%**************************************************************************
% OUTPUTS
%   - value (logical scalar)
%**************************************************************************
% UNITS
%   - Values are dimensionless.
%**************************************************************************
if ~(islogical(value) && isscalar(value)) && ...
        ~(isnumeric(value) && isscalar(value) && ...
        isfinite(value) && any(value == [0 1]))
    error("resolveAzElExampleOptions:InvalidLogicalControl", ...
        "%s must be scalar logical or binary numeric.", name);
end
value = logical(value);
end
