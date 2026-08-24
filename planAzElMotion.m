function result = planAzElMotion(obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = planAzElMotion()
%   options = planAzElMotion(plannerMethod)
%   result = planAzElMotion( ...
%       obstacles, initialState, goalState, limits)
%   result = planAzElMotion( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Run the compact planner directly, or preserve it as an immutable
%     baseline before bounded, independently validated HS3 improvement work.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle array, nested cells, or [])
%       Construct safety margins with makeAzElObstacleData exactly once.
%   - initialState (scalar struct)
%       time_s and 1-by-2 position_deg are required. Supported derivatives
%       retain the contract of the selected planner method.
%   - goalState (scalar struct)
%       Fixed or moving-goal state accepted by the selected planner method.
%   - limits (scalar struct)
%       Physical and workspace limits accepted by both planner methods.
%   - optionOverrides (scalar struct, optional; default struct())
%       PlannerMethod selects "corridorQuintic" or "hs3". All remaining
%       fields are forwarded unchanged to that method. Empty PlannerMethod
%       selects corridorQuintic.
%   - plannerMethod (scalar text, defaults-only call)
%       Returns the exact defaults for "corridorQuintic" or "hs3" plus the
%       normalized PlannerMethod field.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Selected planner result with the stable public contract. Its Options
%       record echoes the resolved PlannerMethod.
%   - options (scalar struct, defaults-only calls)
%       Method-specific defaults plus PlannerMethod. The zero-input call
%       returns corridorQuintic defaults for backward compatibility.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3. Histories are N-by-2 [azimuth elevation].
%**************************************************************************

%% Section 1: Resolve Defaults Requests

defaultPlannerMethod = "corridorQuintic";

if nargin == 0
    result = methodDefaults(defaultPlannerMethod);
    return;
end

if nargin == 1
    if ~(ischar(obstacles) || (isstring(obstacles) && isscalar(obstacles)))
        error("planAzElMotion:MissingInputs", ...
            "A one-input call must name a planner method. Planning requires obstacles, initialState, goalState, and limits.");
    end

    plannerMethod = normalizePlannerMethod(obstacles);
    result = methodDefaults(plannerMethod);
    return;
end

%% Section 2: Resolve The Selected Planner

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

plannerMethod = defaultPlannerMethod;
if isfield(optionOverrides, "PlannerMethod") && ...
        ~isempty(optionOverrides.PlannerMethod)
    plannerMethod = normalizePlannerMethod(optionOverrides.PlannerMethod);
end

methodOverrides = optionOverrides;
if isfield(methodOverrides, "PlannerMethod")
    methodOverrides = rmfield(methodOverrides, "PlannerMethod");
end

% MotionMethod belongs only to the corridor snapshot. A user can therefore
% switch a corridor defaults record to HS3 by changing just PlannerMethod.
if plannerMethod == "hs3" && isfield(methodOverrides, "MotionMethod")
    motionMethod = string(methodOverrides.MotionMethod);
    if ~isscalar(motionMethod) || motionMethod ~= "corridorQuintic"
        error("planAzElMotion:InvalidMotionMethod", ...
            "MotionMethod is a corridor compatibility field and must remain 'corridorQuintic' when PlannerMethod is 'hs3'.");
    end
    methodOverrides = rmfield(methodOverrides, "MotionMethod");
end

%% Section 3: Run The Selected Planner Composition

switch plannerMethod
    case "corridorQuintic"
        result = azElPlannerMethods.corridor.plan( ...
            obstacles, initialState, goalState, limits, methodOverrides);

    case "hs3"
        [hs3Options, compactOverrides] = ...
            azElPlannerMethods.hs3.resolvePlannerOptions(methodOverrides);
        compactBaseline = azElPlannerMethods.corridor.plan( ...
            obstacles, initialState, goalState, limits, compactOverrides);
        result = azElPlannerMethods.hs3.improve( ...
            compactBaseline, hs3Options);
end

% Echo the selected public composition after its engine has resolved all
% method-specific controls and retained rejected improvement evidence.
result.Options.PlannerMethod = plannerMethod;
result.SearchDiagnostics.PlannerMethod = plannerMethod;

end


function options = methodDefaults(plannerMethod)
% Return defaults from only the selected folder so the other can be removed.
switch plannerMethod
    case "corridorQuintic"
        options = azElPlannerMethods.corridor.plan();

    case "hs3"
        options = azElPlannerMethods.hs3.plan();
end

options.PlannerMethod = plannerMethod;
end


function plannerMethod = normalizePlannerMethod(value)
% Normalize the public selector while retaining two explicit method names.
plannerMethod = lower(string(value));
if ~isscalar(plannerMethod)
    error("planAzElMotion:InvalidPlannerMethod", ...
        "PlannerMethod must be scalar text: 'corridorQuintic' or 'hs3'.");
end

switch plannerMethod
    case "corridorquintic"
        plannerMethod = "corridorQuintic";

    case "hs3"
        plannerMethod = "hs3";

    otherwise
        error("planAzElMotion:InvalidPlannerMethod", ...
            "PlannerMethod must be 'corridorQuintic' or 'hs3'; observed '%s'.", ...
            plannerMethod);
end
end
