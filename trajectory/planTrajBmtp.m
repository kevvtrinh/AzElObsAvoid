function [candidate, diagnostics, restart] = planTrajBmtp( ...
        seed, regions_deg, coverage, initialState, goalState, limits, ...
        options, warmStart)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics, restart] = planTrajBmtp( ...
%       seed, regions_deg, coverage, initialState, goalState, limits, options)
%   [candidate, diagnostics, restart] = planTrajBmtp( ...
%       seed, regions_deg, coverage, initialState, goalState, limits, ...
%       options, warmStart)
%**************************************************************************
% PURPOSE
%   - Provide the public trajectory-planning entry point for BMTP motion.
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
%   - warmStart (scalar struct, optional; default struct())
%       Compatible certified control net and segment duration.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Stable success-or-failure BMTP motion record.
%   - diagnostics (scalar struct)
%       BMTP solver and certificate evidence.
%   - restart (scalar struct)
%       Certified parent control net and segment duration for refinement.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s powers.
%**************************************************************************

if nargin == 7
    [candidate, diagnostics, restart] = bmtpEngine.solve( ...
        seed, regions_deg, coverage, initialState, goalState, limits, options);
elseif nargin == 8
    [candidate, diagnostics, restart] = bmtpEngine.solve( ...
        seed, regions_deg, coverage, initialState, goalState, limits, ...
        options, warmStart);
else
    error("planTrajBmtp:InvalidCall", ...
        "Use seven or eight inputs as documented.");
end
end
