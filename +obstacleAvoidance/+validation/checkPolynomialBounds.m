function [bounds, dynamics, checks] = checkPolynomialBounds( ...
        polynomial, time_s, position_deg, velocity_deg_s, ...
        acceleration_deg_s2, jerk_deg_s3, initialState, goalState, ...
        limits, options, stateTolerance)
%% Section 0: Header & Readme
% SYNTAX
%   [bounds, dynamics, checks] = ...
%       obstacleAvoidance.validation.checkPolynomialBounds( ...
%       polynomial, time_s, position_deg, velocity_deg_s, ...
%       acceleration_deg_s2, jerk_deg_s3, initialState, goalState, ...
%       limits, options, stateTolerance)
%**************************************************************************
% PURPOSE
%   - Check polynomial format, continuity, histories, dynamics, and limits.
%   - Route every scalar continuous limit through the certified range check.
%**************************************************************************
% INPUTS
%   - polynomial (scalar struct)
%       Piecewise power-polynomial representation of the complete motion.
%   - time_s (N-by-1 numeric vector)
%       Sample times in seconds.
%   - position_deg, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3
%       N-by-2 sampled motion histories in [azimuth elevation] order.
%   - initialState, goalState, limits, options (scalar structs)
%       Normalized request, physical limits, and resolved check controls.
%   - stateTolerance (nonnegative finite scalar)
%       Absolute endpoint and history consistency allowance.
%**************************************************************************
% OUTPUTS
%   - bounds (scalar struct)
%       Continuous position, velocity, acceleration, and jerk results.
%   - dynamics (scalar struct)
%       Polynomial derivative-consistency result and residual.
%   - checks (scalar struct)
%       Format, time-base, continuity, endpoint, and history results.
%**************************************************************************
% UNITS
%   - Position is degrees; derivatives use deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Check The Complete Polynomial Motion

% Sampled histories alone cannot exclude between-sample violations. Delegate
% structural and derivative checks to the established polynomial validator,
% supplying the certified continuous scalar range test explicitly.

[bounds, dynamics, checks] = ...
    obstacleAvoidance.validation.validatePolynomialTrajectory( ...
    polynomial, time_s, position_deg, velocity_deg_s, ...
    acceleration_deg_s2, jerk_deg_s3, initialState, goalState, limits, ...
    options, stateTolerance, ...
    @obstacleAvoidance.validation.checkPolynomialRange);
end
