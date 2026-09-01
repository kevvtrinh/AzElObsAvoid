function [candidate, diagnostics, restart] = planTrajBmtp( ...
        seed, regions_deg, coverage, initialState, goalState, limits, ...
        options, legacyRestart) %#ok<INUSD>
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics, restart] = planTrajBmtp( ...
%       seed, regions_deg, coverage, initialState, goalState, limits, options)
%   [candidate, diagnostics, restart] = planTrajBmtp( ...
%       seed, regions_deg, coverage, initialState, goalState, limits, ...
%       options, legacyRestart)
%**************************************************************************
% PURPOSE
%   - Provide the public trajectory-planning entry point for BMTP motion.
%   - Preserve one-release call compatibility for retired restart state.
%**************************************************************************
% INPUTS
%   - seed (scalar struct)
%       Static topology seed with N-by-2 position_deg and normalized tau.
%   - regions_deg (R-by-1 cell array)
%       Finite convex exclusion polygons in degrees.
%   - coverage (scalar struct)
%       Caller-owned geometry-coverage evidence required by the BMTP engine.
%   - initialState, goalState (scalar structs)
%       Normalized rest endpoint states with time_s and position_deg.
%   - limits (scalar struct)
%       Normalized workspace and derivative limits in degrees and seconds.
%   - options (scalar struct)
%       Resolved BMTP planning options.
%   - legacyRestart (scalar struct, optional; deprecated and ignored)
%       Supplying the former restart input warns once and changes no behavior.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Stable success-or-failure BMTP motion record.
%   - diagnostics (scalar struct)
%       BMTP solver and certificate evidence.
%   - restart (scalar struct)
%       Deprecated empty compatibility record with zero control points and
%       SegmentTime_s=NaN. Requesting this output warns once.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s powers.
%**************************************************************************

if nargin ~= 7 && nargin ~= 8
    error("planTrajBmtp:InvalidCall", ...
        "Use seven or eight inputs as documented.");
end
if nargin == 8 || nargout >= 3
    warning("planTrajBmtp:DeprecatedRestart", ...
        "BMTP restart state is deprecated and ignored. The maintained " + ...
        "planner always performs a seed-derived cold solve.");
end
[candidate, diagnostics] = bmtpEngine.solve( ...
    seed, regions_deg, coverage, initialState, goalState, limits, options);
restart = struct( ...
    "ControlPoint_deg", zeros(0, 0, 2), "SegmentTime_s", NaN);
end
