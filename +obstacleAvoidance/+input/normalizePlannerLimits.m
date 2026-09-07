function limits = normalizePlannerLimits(limits)
%% Section 0: Header & Readme
% SYNTAX
%   limits = obstacleAvoidance.input.normalizePlannerLimits(limits)
% PURPOSE
%   Resolve combined or per-axis physical limits and workspace intervals.
% INPUTS
%   limits: scalar struct with positive finite maxVelocity_units_s,
%   maxAcceleration_units_s2, and maxJerk_units_s3. All three must be scalars
%   (combined magnitudes) or two-element vectors ([x y]).
%   Optional xInterval_units and yInterval_units remain intervals.
% OUTPUTS
%   limits: double row vectors with workspace defaults filled in. A combined
%   limit L becomes [L/sqrt(2), L/sqrt(2)]; normalized vectors stay unchanged.
% UNITS
%   Coordinate units, units/s, units/s^2, and units/s^3 as indicated by the field names.

%% Section 1: Validate One Consistent Physical Limit Form

physicalNames = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"];
if ~isstruct(limits) || ~isscalar(limits) || ~all(isfield(limits, cellstr(physicalNames)))
    error("planTrajectory:InvalidLimits", "limits must contain velocity, acceleration, and jerk limits.");
end
limitSizes = zeros(1, numel(physicalNames));
for fieldIndex = 1:numel(physicalNames)
    fieldName = physicalNames(fieldIndex);
    validateattributes(limits.(fieldName), {'numeric'}, {'real', 'finite', 'positive', 'vector'}, "planTrajectory", fieldName);
    limitSizes(fieldIndex) = numel(limits.(fieldName));
    if ~any(limitSizes(fieldIndex) == [1 2])
        error("planTrajectory:InvalidLimits", "%s must be a combined scalar or a two-element [x y] limit.", fieldName);
    end
end
if any(limitSizes ~= limitSizes(1))
    error("planTrajectory:MixedLimitModes", "Velocity, acceleration, and jerk limits must all be combined scalars or all be two-element [x y] limits; mixing is not supported.");
end

%% Section 2: Allocate Combined Magnitudes Equally Between Axes

for fieldName = physicalNames
    value = double(limits.(fieldName)(:).');
    if isscalar(value)
        % Equal components have the requested hypotenuse. Keep this fixed
        % allocation for all solvers and continuous validation.
        value = repmat(value / sqrt(2), 1, 2);
    end
    limits.(fieldName) = value;
end

%% Section 3: Resolve Workspace Intervals Without Scaling

intervalDefaults = {"xInterval_units", [-180 180]; ...
    "yInterval_units", [-90 90]};
for intervalIndex = 1:size(intervalDefaults, 1)
    fieldName = intervalDefaults{intervalIndex, 1};
    if ~isfield(limits, fieldName) || isempty(limits.(fieldName))
        limits.(fieldName) = intervalDefaults{intervalIndex, 2};
    end
    validateattributes(limits.(fieldName), {'numeric'}, {'real', 'finite', 'vector', 'numel', 2, 'increasing'}, "planTrajectory", fieldName);
    limits.(fieldName) = double(limits.(fieldName)(:).');
end
end
