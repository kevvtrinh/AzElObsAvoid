function [time_s, position_deg, velocity_deg_s, ...
        acceleration_deg_s2, jerk_deg_s3] = evaluatePlannerPolynomial( ...
        polynomial, time_s, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
%       jerk_deg_s3] = obstacleAvoidance.planner.evaluatePlannerPolynomial(polynomial, time_s)
%   [time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
%       jerk_deg_s3] = obstacleAvoidance.planner.evaluatePlannerPolynomial( ...
%       polynomial, time_s, segmentIndex)
%**************************************************************************
% PURPOSE
%   - Evaluate shared ascending-power segment records at absolute times.
%**************************************************************************
% INPUTS
%   - polynomial (scalar normalized planner polynomial struct)
%       Coefficient arrays use N-by-D-by-P shape and ascending powers.
%   - time_s (numeric vector)
%       Absolute evaluation times. The output uses a numeric column.
%   - segmentIndex (numeric scalar or vector, optional; default [])
%       Select an exact segment for each time. Empty values select segments
%       from polynomial start-time records. A scalar applies to all times.
%**************************************************************************
% OUTPUTS
%   - time_s (N-by-1 numeric column)
%       Normalized requested times.
%   - position_deg through jerk_deg_s3 (N-by-D numeric arrays)
%       Evaluated motion histories for every modeled coordinate.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************

%% Section 1: Translate The Polynomial Data

% HS3 stores coefficients by segment, axis, and power. The shared evaluator
% uses a dimension-neutral record. This block only rearranges fields; it does
% not resample or approximate the continuous motion.

enginePolynomial = struct( ...
    "SegmentCount", polynomial.SegmentCount, ...
    "SegmentStartTime", polynomial.SegmentStartTime_s, ...
    "SegmentDuration", polynomial.SegmentDuration_s, ...
    "FinalTime", polynomial.FinalTime_s, ...
    "positionPower", polynomial.positionPower_deg, ...
    "velocityPower", polynomial.velocityPower_deg_s, ...
    "accelerationPower", polynomial.accelerationPower_deg_s2, ...
    "jerkPower", polynomial.jerkPower_deg_s3);

%% Section 2: Evaluate Through The Dimension-Neutral Engine

% Explicit query times evaluate the same polynomial at caller-selected
% instants. Omitting them uses the standard grid including segment boundaries.

if nargin < 3
    segmentIndex = [];
end
[time_s, position_deg, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3] = ...
    hs3Internal.polynomial.evaluateTrajectoryPolynomial(enginePolynomial, time_s, segmentIndex);
end
