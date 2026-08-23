function validation = validateAzElTrajectory(trajectory, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   validation = validateAzElTrajectory()
%   validation = validateAzElTrajectory(result)
%   validation = validateAzElTrajectory( ...
%       trajectory, obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Route independent trajectory validation to the validator preserved
%     with the method that produced the motion.
%   - Keep legacy explicit calls on the default corridor validator.
%**************************************************************************
% INPUTS
%   - trajectory (scalar candidate or planner-result struct)
%   - obstacles (canonical protected obstacle array, explicit form)
%   - initialState, goalState (normalized states, explicit form)
%   - limits (normalized physical limits, explicit form)
%   - options (resolved method options, explicit form)
%       PlannerMethod optionally selects "corridorQuintic" or "hs3".
%       A one-input planner result supplies this field through result.Options.
%**************************************************************************
% OUTPUTS
%   - validation (selected method's stable validation record)
%**************************************************************************

%% Section 1: Preserve The Public Call Contract

if nargin == 0
    validation = azElPlannerMethods.corridor.validateTrajectory();
    return;
end

if nargin ~= 1 && nargin ~= 6
    error("validateAzElTrajectory:InvalidCall", ...
        "Use one planner result or all six explicit validation inputs.");
end

%% Section 2: Identify The Validator That Owns The Motion

% Older caller-created trajectories have no selector, so they retain the
% corridor validator that was public when the combined branch was created.
plannerMethod = "corridorQuintic";

if nargin == 1
    if isstruct(trajectory) && isscalar(trajectory) && ...
            isfield(trajectory, "Options") && ...
            isstruct(trajectory.Options) && ...
            isscalar(trajectory.Options) && ...
            isfield(trajectory.Options, "PlannerMethod") && ...
            ~isempty(trajectory.Options.PlannerMethod)
        methodDefaults = planAzElMotion(trajectory.Options.PlannerMethod);
        plannerMethod = methodDefaults.PlannerMethod;
    end
elseif isstruct(options) && isscalar(options) && ...
        isfield(options, "PlannerMethod") && ...
        ~isempty(options.PlannerMethod)
    methodDefaults = planAzElMotion(options.PlannerMethod);
    plannerMethod = methodDefaults.PlannerMethod;
end

%% Section 3: Run Only The Selected Independent Validator

switch plannerMethod
    case "corridorQuintic"
        if nargin == 1
            validation = azElPlannerMethods.corridor.validateTrajectory(trajectory);
        else
            validation = azElPlannerMethods.corridor.validateTrajectory( ...
                trajectory, obstacles, initialState, goalState, limits, options);
        end

    case "hs3"
        if nargin == 1
            validation = azElPlannerMethods.hs3.validateTrajectory(trajectory);
        else
            validation = azElPlannerMethods.hs3.validateTrajectory( ...
                trajectory, obstacles, initialState, goalState, limits, options);
        end
end
end
