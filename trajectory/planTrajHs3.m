function trajectory = planTrajHs3( ...
        initialState, terminalState, limits, optionOverrides, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   options = planTrajHs3()
%   trajectory = planTrajHs3(initialState, terminalState, limits)
%   trajectory = planTrajHs3( ...
%       initialState, terminalState, limits, optionOverrides)
%   trajectory = planTrajHs3( ...
%       initialState, terminalState, limits, optionOverrides, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Provide the public trajectory-planning entry point for HS3 motion.
%**************************************************************************
% INPUTS
%   - initialState (scalar struct)
%       Initial time, position, velocity, and acceleration state.
%   - terminalState (scalar struct)
%       Terminal position, derivatives, and time policy.
%   - limits (scalar struct)
%       Coordinate and derivative limits in caller-consistent units.
%   - optionOverrides (scalar struct, optional; default struct())
%       Partial HS3 planning-option overrides; empty fields use defaults.
%   - pathConstraints (scalar struct, optional; default empty)
%       Optional affine path constraints accepted by the HS3 implementation.
%**************************************************************************
% OUTPUTS
%   - trajectory (scalar struct)
%       Stable success-or-failure motion record. Zero inputs return defaults.
%**************************************************************************
% UNITS
%   - Time and coordinate units are caller-defined and must be consistent.
%**************************************************************************

if nargin == 0
    trajectory = hs3Engine.solve();
elseif nargin == 3
    trajectory = hs3Engine.solve(initialState, terminalState, limits);
elseif nargin == 4
    trajectory = hs3Engine.solve( ...
        initialState, terminalState, limits, optionOverrides);
else
    trajectory = hs3Engine.solve( ...
        initialState, terminalState, limits, optionOverrides, pathConstraints);
end
end
