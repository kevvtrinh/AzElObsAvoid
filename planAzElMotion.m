function result = planAzElMotion( ...
        obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = planAzElMotion()
%   options = planAzElMotion("hs3")
%   result = planAzElMotion( ...
%       obstacles, initialState, goalState, limits)
%   result = planAzElMotion( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Run the maintained Hermite-Simpson planner through one public entry
%     point with no alternate motion planner or fallback path.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle array, nested cells, or [])
%       Construct safety margins with makeAzElObstacleData exactly once.
%   - initialState (scalar struct)
%       Initial time, position, and supported derivatives.
%   - goalState (scalar struct)
%       Fixed or moving-goal state accepted by the HS3 planner.
%   - limits (scalar struct)
%       Physical and workspace limits with units in field names.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial HS3 options. PlannerMethod may be omitted or equal "hs3".
%   - plannerMethod (scalar text, defaults-only call)
%       The only supported value is "hs3".
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable success-or-failure planner result with HS3 diagnostics.
%   - options (scalar struct, defaults-only calls)
%       Resolved HS3 defaults with PlannerMethod equal "hs3".
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3. Histories are N-by-2 [azimuth elevation].
%**************************************************************************

%% Section 1: Resolve Defaults Requests

if nargin == 0
    result = azElPlannerMethods.hs3.plan();
    result.PlannerMethod = "hs3";
    return;
end

if nargin == 1
    if ~(ischar(obstacles) || ...
            (isstring(obstacles) && isscalar(obstacles)))
        error("planAzElMotion:MissingInputs", ...
            "A one-input call must name 'hs3'. Planning requires " + ...
            "obstacles, initialState, goalState, and limits.");
    end
    plannerMethod = lower(string(obstacles));
    if ~isscalar(plannerMethod) || plannerMethod ~= "hs3"
        error("planAzElMotion:InvalidPlannerMethod", ...
            "PlannerMethod must be 'hs3'; observed '%s'.", ...
            plannerMethod);
    end
    result = azElPlannerMethods.hs3.plan();
    result.PlannerMethod = "hs3";
    return;
end

%% Section 2: Resolve The HS3 Request

if nargin < 4
    error("planAzElMotion:MissingInputs", ...
        "obstacles, initialState, goalState, and limits are required.");
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("planAzElMotion:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end

if isfield(optionOverrides, "PlannerMethod") && ...
        ~isempty(optionOverrides.PlannerMethod)
    plannerMethod = lower(string(optionOverrides.PlannerMethod));
    if ~isscalar(plannerMethod) || plannerMethod ~= "hs3"
        error("planAzElMotion:InvalidPlannerMethod", ...
            "PlannerMethod must be 'hs3'; observed '%s'.", ...
            plannerMethod);
    end
end
if isfield(optionOverrides, "PlannerMethod")
    optionOverrides = rmfield(optionOverrides, "PlannerMethod");
end

%% Section 3: Run The HS3 Planner

result = azElPlannerMethods.hs3.plan( ...
    obstacles, initialState, goalState, limits, optionOverrides);
result.Options.PlannerMethod = "hs3";
result.SearchDiagnostics.PlannerMethod = "hs3";
end
