function [feasible, message, reason] = checkPlannerEndpoints( ...
        obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [feasible, message, reason] = ...
%       obstacleAvoidance.input.checkPlannerEndpoints( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Check endpoint geometry, dynamics, timing, and workspace requirements
%     before route search or trajectory optimization begins.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected-obstacle array)
%       Prepared static or time-varying obstacle histories.
%   - initialState, goalState, limits (normalized scalar structs)
%       Endpoint request and physical workspace or motion limits.
%   - options (resolved scalar struct)
%       Goal-time, tolerance, and azimuth-wrap policies.
%**************************************************************************
% OUTPUTS
%   - feasible (logical scalar)
%       True only when all endpoint requirements pass.
%   - message, reason (string scalars)
%       Empty on success; otherwise actionable and machine-readable failure.
%**************************************************************************
% UNITS
%   - Position and workspace intervals are degrees; time is seconds;
%     derivatives use deg/s and deg/s^2.
%**************************************************************************

%% Section 1: Run The Established Endpoint Checks

% Keep the prior internal entry point as a compatibility alias during the
% plain-language migration so error identifiers and numerical order remain
% unchanged in this restructuring pass.
[feasible, message, reason] = ...
    obstacleAvoidance.input.validatePlannerEndpoints( ...
    obstacles, initialState, goalState, limits, options);
end
