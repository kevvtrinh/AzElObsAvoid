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
% EXAMPLE-ONLY OPTION FIELDS
%   - EnableJerkConstraint (logical scalar; default true)
%   - MaxJerk_deg_s3 (positive finite scalar or two-element vector)
%**************************************************************************
% OUTPUTS
%   - plannerOptions (scalar struct)
%       Options safe to pass directly to planAzElMotion.
%   - jerkConfiguration (scalar struct)
%       JerkConstraintEnabled and MaxJerk_deg_s3 resolved for limits.
%**************************************************************************
% UNITS
%   - Jerk is degrees per second cubed.
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
    validateattributes(optionOverrides.EnableJerkConstraint, ...
        {'logical','numeric'}, {'scalar'});
    enableJerkConstraint = logical( ...
        optionOverrides.EnableJerkConstraint);
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

exampleOnlyFields = intersect(fieldnames(optionOverrides), ...
    {'EnableJerkConstraint', 'MaxJerk_deg_s3'}, "stable");
if ~isempty(exampleOnlyFields)
    optionOverrides = rmfield(optionOverrides, exampleOnlyFields);
end
plotDefaults = struct( ...
    "FigureVisible", "on", ...
    "ShowAnimation", true, ...
    "ShowKinematicPlot", true);
plotDefaultNames = fieldnames(plotDefaults);
for index = 1:numel(plotDefaultNames)
    name = plotDefaultNames{index};
    if ~isfield(plannerDefaults, name) || isempty(plannerDefaults.(name))
        plannerDefaults.(name) = plotDefaults.(name);
    end
end

plannerOptions = plannerDefaults;
overrideNames = fieldnames(optionOverrides);
for index = 1:numel(overrideNames)
    name = overrideNames{index};
    if ~isempty(optionOverrides.(name))
        plannerOptions.(name) = optionOverrides.(name);
    end
end
resolvedJerk_deg_s3 = [Inf Inf];
if enableJerkConstraint
    resolvedJerk_deg_s3 = maxJerk_deg_s3;
end
jerkConfiguration = struct( ...
    "JerkConstraintEnabled", enableJerkConstraint, ...
    "MaxJerk_deg_s3", resolvedJerk_deg_s3, ...
    "ConfiguredFiniteMaxJerk_deg_s3", maxJerk_deg_s3);
end
