function result = planTrajRuckig( ...
        initialState, terminalState, limits, options, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   options = planTrajRuckig()
%   result = planTrajRuckig(initialState, terminalState, limits)
%   result = planTrajRuckig(initialState, terminalState, limits, options)
%   result = planTrajRuckig( ...
%       initialState, terminalState, limits, options, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Provide the public trajectory-planning entry point for Ruckig motion.
%**************************************************************************
% INPUTS
%   - initialState (scalar struct)
%       Initial time, position, velocity, and acceleration state.
%   - terminalState (scalar struct)
%       Terminal position, derivatives, and time policy.
%   - limits (scalar struct)
%       Coordinate and derivative limits in caller-consistent units.
%   - options (scalar struct, optional; default struct())
%       Partial Ruckig planning-option overrides; empty fields use defaults.
%   - pathConstraints (scalar struct, optional; default empty)
%       Nonempty affine path constraints return the documented unsupported
%       result from the Ruckig implementation.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable success, unsupported, or failure record. Zero inputs return
%       defaults; invalid requirements throw identified errors.
%**************************************************************************
% UNITS
%   - Time and coordinate units are caller-defined and must be consistent.
%**************************************************************************

if nargin == 0
    result = ruckigEngine.solve();
elseif nargin == 3
    result = ruckigEngine.solve(initialState, terminalState, limits);
elseif nargin == 4
    result = ruckigEngine.solve(initialState, terminalState, limits, options);
else
    result = ruckigEngine.solve( ...
        initialState, terminalState, limits, options, pathConstraints);
end
end
