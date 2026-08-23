function result = planAzElMovingTargetIntercept(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   options = planAzElMovingTargetIntercept()
%   result = planAzElMovingTargetIntercept( ...
%       initialState, targetMotion, limits, options)
%   result = planAzElMovingTargetIntercept( ...
%       obstacles, initialState, targetMotion, limits, options)
%**************************************************************************
% PURPOSE
%   - Route a moving-target request to the adapter that belongs to the
%     selected planner method. Keeping both adapters with their planners
%     preserves the behavior of each source branch.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle array, optional; default [])
%   - initialState (scalar state struct)
%   - targetMotion (sampled moving-target struct)
%   - limits (scalar physical and workspace limits struct)
%   - options (scalar struct)
%       PlannerOptions.PlannerMethod selects "corridorQuintic" or "hs3".
%       The selector defaults to "corridorQuintic" and is removed before
%       the remaining planner options reach the selected adapter.
%**************************************************************************
% OUTPUTS
%   - result (scalar planAzElMotion-compatible result)
%       Options.PlannerMethod and SearchDiagnostics.PlannerMethod record
%       which complete method produced the result.
%   - options (zero-input call)
%       Corridor moving-target defaults with the public selector included.
%**************************************************************************

%% Section 1: Keep The Existing Public Call Forms

% A defaults request remains tied to the backward-compatible corridor
% method. The selector is added only at this public dispatch boundary.
if nargin == 0
    result = azElPlannerMethods.corridor.planMovingTargetIntercept();
    result.PlannerOptions.PlannerMethod = "corridorQuintic";
    return;
end

if nargin == 4
    optionIndex = 4;
elseif nargin == 5
    optionIndex = 5;
else
    error("planAzElMovingTargetIntercept:InvalidCall", ...
        "Use zero, four, or five inputs as documented.");
end

%% Section 2: Read And Remove The Public Method Selector

% Invalid outer or nested option records are intentionally forwarded to the
% corridor adapter so the established contract error remains authoritative.
% Only a well-formed, nonempty selector changes the destination method.
plannerMethod = "corridorQuintic";
backendArguments = varargin;
optionOverrides = varargin{optionIndex};

if isstruct(optionOverrides) && isscalar(optionOverrides) && ...
        isfield(optionOverrides, "PlannerOptions") && ...
        isstruct(optionOverrides.PlannerOptions) && ...
        isscalar(optionOverrides.PlannerOptions)
    plannerOptions = optionOverrides.PlannerOptions;

    if isfield(plannerOptions, "PlannerMethod") && ...
            ~isempty(plannerOptions.PlannerMethod)
        selectedDefaults = planAzElMotion(plannerOptions.PlannerMethod);
        plannerMethod = selectedDefaults.PlannerMethod;
    end

    % PlannerMethod belongs to this dispatcher, not either preserved
    % implementation. Removing it prevents a false unknown-option warning.
    if isfield(plannerOptions, "PlannerMethod")
        plannerOptions = rmfield(plannerOptions, "PlannerMethod");
    end

    optionOverrides.PlannerOptions = plannerOptions;
    backendArguments{optionIndex} = optionOverrides;
end

%% Section 3: Run Only The Selected Moving-Target Adapter

% The corridor branch used a chronological fixed-arrival search. The HS3
% branch used one moving-goal earliest-arrival solve. Dispatching to separate
% snapshots retains that material difference instead of blending algorithms.
switch plannerMethod
    case "corridorQuintic"
        result = azElPlannerMethods.corridor.planMovingTargetIntercept(backendArguments{:});

    case "hs3"
        result = azElPlannerMethods.hs3.planMovingTargetIntercept(backendArguments{:});
end

% Echo the method at stable top-level diagnostic locations. Also restore it
% in the adapter's nested option record so a saved result is reproducible.
result.Options.PlannerMethod = plannerMethod;
result.SearchDiagnostics.PlannerMethod = plannerMethod;
result.Intercept.Options.PlannerOptions.PlannerMethod = plannerMethod;
end
