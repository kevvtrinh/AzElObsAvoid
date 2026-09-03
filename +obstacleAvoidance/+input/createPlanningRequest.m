function request = createPlanningRequest( ...
        obstacles, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   request = obstacleAvoidance.input.createPlanningRequest( ...
%       obstacles, initialState, goalState, limits)
%   request = obstacleAvoidance.input.createPlanningRequest( ...
%       obstacles, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Resolve planner options and normalize one complete planning request.
%   - Give later stages one stable record instead of separate raw inputs.
%**************************************************************************
% INPUTS
%   - obstacles (supported obstacle input or [])
%       Static or time-varying protected obstacle geometry.
%   - initialState, goalState (scalar structs)
%       Initial motion and requested terminal motion or target policy.
%   - limits (scalar struct)
%       Physical and workspace limits with units in field names.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial public planner options. Empty fields use defaults.
%**************************************************************************
% OUTPUTS
%   - request (scalar struct)
%       Normalized obstacles, states, limits, and resolved options.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Options

% Options control normalization rules such as workspace and time policies.
% Resolve them first so every input is interpreted under the same choices.

if nargin < 4
    error("createPlanningRequest:MissingInputs", ...
        "obstacles, initialState, goalState, and limits are required.");
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
options = obstacleAvoidance.input.resolvePlannerOptions(optionOverrides);
obstacleAvoidance.input.throwIfCancellationRequested(options);

%% Section 2: Normalize Physical Inputs

% Search and motion stages require consistent state shapes, obstacle records,
% units, and workspace limits. Normalize those forms once at this boundary.

[obstacles, initialState, goalState, limits] = ...
    obstacleAvoidance.input.normalizePlannerRequest( ...
    obstacles, initialState, goalState, limits, options);

%% Section 3: Assemble The Request

% Keep physical meaning separate from later prepared geometry. Obstacle
% histories remain the original normalized inputs in this record.

request = struct( ...
    "obstacles", obstacles, ...
    "initialState", initialState, ...
    "goalState", goalState, ...
    "limits", limits, ...
    "options", options);
end
