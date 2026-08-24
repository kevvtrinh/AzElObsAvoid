function result = plan(obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Compatibility Facade
% Preserve the method-qualified integration point while the public dispatcher
% owns composition of the compact baseline and optional HS3 improvement.

if nargin == 0
    [result, ~] = azElPlannerMethods.hs3.resolvePlannerOptions();
    return;
end
if nargin < 4
    error("planAzElMotion:MissingInputs", "obstacles, initialState, goalState, and limits are required.");
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    result = planAzElMotion(obstacles, initialState, goalState, limits, optionOverrides);
    return;
end
optionOverrides.PlannerMethod = "hs3";
result = planAzElMotion(obstacles, initialState, goalState, limits, optionOverrides);
end
