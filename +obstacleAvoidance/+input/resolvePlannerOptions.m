function options = resolvePlannerOptions(optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.input.resolvePlannerOptions()
%   options = obstacleAvoidance.input.resolvePlannerOptions(optionOverrides)
%
% PURPOSE
%   - Resolve and validate obstacle-planner search and motion options.
%
% INPUTS
%   optionOverrides (scalar struct, optional)
%     Omitted/empty fields use defaults. Unknown fields warn once and are ignored.
%
%   Option                            Default              Allowed values
%   GoalTimeMode                      "earliestArrival"    See modes below
%   MaximumSeedCount                  2                    Integers 1-5
%   MaximumTimeLayerCount             17                    Integers 2-65535
%   MaximumWaitRefinementIterations   16                    Integers 0-64
%   UnsupportedTimedTopologyPolicy   "fail"                See fallback below
%   WrapX, WrapY                     false                 Logical scalars
%     Each enabled axis uses its workspace interval width as the period.
%     Only obstacle-free fixed-position goals support wrapping.
%
%   Arrival modes:
%     earliestArrival  Minimize arrival time; break ties by shorter travel.
%     fixedArrival     Minimize travel at the specified arrival time.
%     Jerk is a hard limit in every mode.
%
%   Search limits:
%     Seeds beyond the first two are tried only after failure.
%     Time layers bound timed-motion segments + 1.
%     Wait iterations limit direct-wait bisection trials.
%
%   Fallback:
%     fail                   Report unsupported timed routes.
%     ruckigStopAtWaypoints   Allow stop-at-waypoint fallback for at most 2 segments.
%
% OUTPUTS
%   - options (scalar struct)
%       Fully populated, normalized, and validated planner options.
%
% UNITS
%   - Time fields use seconds and clearance fields use coordinate units.
%

%% Section 1: Resolve Defaults

defaults = struct();
defaults.GoalTimeMode                    = "earliestArrival";
defaults.SampleTime_s                    = 0.05;
defaults.UnsupportedTimedTopologyPolicy  = "fail";
defaults.WrapX                          = false;
defaults.WrapY                          = false;
defaults.MaximumSeedCount                = 2;
defaults.MaximumTimeLayerCount           = 17;
defaults.MaximumWaitRefinementIterations = 16;
defaults.ArrivalTimeTolerance_s          = 1e-3;
defaults.ConstraintTolerance             = 1e-7;
defaults.CollisionClearanceTolerance_units = 1e-7;
defaults.CollisionMinimumTimeStep_s      = 0.00025;
if nargin == 0 || isempty(optionOverrides)
    options = defaults;
    return;
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("planTrajectory:InvalidOptions", "optionOverrides must be a scalar struct.");
end
% Apply the required validation or transfer to each field name.
for fieldName = ["XInterval_units", "YInterval_units"]
    if isfield(optionOverrides, fieldName)
        replacementName = lower(extractBefore(fieldName, "Interval")) + "Interval_units";
        error("planTrajectory:WorkspaceLimitMoved", "%s has moved from options to limits.%s.", fieldName, replacementName);
    end
end
[options, unknownNames] = obstacleAvoidance.input.resolveOptions(defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("planTrajectory:UnknownOptions", "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end

%% Section 2: Normalize Public Values

textRules = {"GoalTimeMode", ...
    ["earliestArrival", "fixedArrival"], ...
    "planTrajectory:InvalidGoalTimeMode", ...
    "GoalTimeMode must be 'earliestArrival' or " + ...
    "'fixedArrival'."; ...
    "UnsupportedTimedTopologyPolicy", ...
    ["fail", "ruckigStopAtWaypoints"], ...
    "planTrajectory:InvalidUnsupportedTimedTopologyPolicy", ...
    "UnsupportedTimedTopologyPolicy must be 'fail' or " + ...
    "'ruckigStopAtWaypoints'."};
% Apply the required validation or transfer to each rule.
for ruleIndex = 1:size(textRules, 1)
    fieldName = textRules{ruleIndex, 1};
    options.(fieldName) = string(options.(fieldName));
    if ~isscalar(options.(fieldName)) || ~any(options.(fieldName) == textRules{ruleIndex, 2})
        error(textRules{ruleIndex, 3}, textRules{ruleIndex, 4});
    end
end
options.WrapX = obstacleAvoidance.input.normalizeLogicalScalar(options.WrapX, "WrapX", "planTrajectory:InvalidLogicalOption");
options.WrapY = obstacleAvoidance.input.normalizeLogicalScalar(options.WrapY, "WrapY", "planTrajectory:InvalidLogicalOption");

validateattributes(options.SampleTime_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
integerRules = {"MaximumSeedCount", 1, 5; ...
    "MaximumTimeLayerCount", 2, 65535; ...
    "MaximumWaitRefinementIterations", 0, 64};
% Apply the required validation or transfer to each rule.
for ruleIndex = 1:size(integerRules, 1)
    validateattributes(options.(integerRules{ruleIndex, 1}), {'numeric'}, {'real', 'finite', 'scalar', 'integer', '>=', integerRules{ruleIndex, 2}, '<=', integerRules{ruleIndex, 3}});
end
% Apply the required validation or transfer to each field name.
for fieldName = ["ArrivalTimeTolerance_s", "ConstraintTolerance", ...
        "CollisionMinimumTimeStep_s"]
    validateattributes(options.(fieldName), {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
end
validateattributes(options.CollisionClearanceTolerance_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
end
