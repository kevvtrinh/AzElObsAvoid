function [polynomial, terminalState] = createJerkPolynomial(initialState, relativeBreak_s, segmentJerk_units_s3, segmentAcceleration_units_s2)
%% Section 0: Header & Readme
% SYNTAX: [polynomial, terminalState] = motionCore.createJerkPolynomial(initialState, breaks, jerk, acceleration)
% PURPOSE: Integrate exact event intervals into the shared physical-unit polynomial.
% INPUTS: Initial time_s, position_units, velocity_units_s, acceleration_units_s2;
%   increasing relative breaks from zero and one jerk row per interval.
%   Optional acceleration rows reset acceleration for second-order switching control.
% OUTPUTS: Cubic polynomial and analytically integrated terminal state.
% UNITS: Coordinate units, seconds, and physical derivatives; any axis count.

%% Section 1: Integrate The Supplied Control Without Resampling
dimensionCount = numel(initialState.position_units);
relativeBreak_s    = double(relativeBreak_s(:));
segmentDuration_s  = diff(relativeBreak_s);
segmentJerk_units_s3 = double(segmentJerk_units_s3);
segmentCount       = numel(segmentDuration_s);
partitionValid     = relativeBreak_s(1) == 0 && all(segmentDuration_s > 0) && isequal(size(segmentJerk_units_s3), [segmentCount, dimensionCount]);
if ~partitionValid
    error("createMotionRecord:InvalidEventWord", "Breaks must increase from zero and jerk must be N-by-D.");
end
positionPower_units        = zeros(segmentCount, dimensionCount, 4);
velocityPower_units_s      = zeros(segmentCount, dimensionCount, 3);
accelerationPower_units_s2 = zeros(segmentCount, dimensionCount, 2);
position_units             = initialState.position_units;
velocity_units_s           = initialState.velocity_units_s;
acceleration_units_s2      = initialState.acceleration_units_s2;
hasAccelerationControl = nargin >= 4 && ~isempty(segmentAcceleration_units_s2);
for segmentIndex = 1:segmentCount
    if hasAccelerationControl
        acceleration_units_s2 = segmentAcceleration_units_s2(segmentIndex, :);
    end
    step_s      = segmentDuration_s(segmentIndex);
    jerk_units_s3 = segmentJerk_units_s3(segmentIndex, :);
    positionPower_units(segmentIndex, :, :) = reshape([position_units; velocity_units_s * step_s; acceleration_units_s2 * step_s ^ 2 / 2; jerk_units_s3 * step_s ^ 3 / 6].', 1, dimensionCount, 4);
    velocityPower_units_s(segmentIndex, :, :) = reshape([velocity_units_s; acceleration_units_s2 * step_s; jerk_units_s3 * step_s ^ 2 / 2].', 1, dimensionCount, 3);
    accelerationPower_units_s2(segmentIndex, :, :) = reshape([acceleration_units_s2; jerk_units_s3 * step_s].', 1, dimensionCount, 2);
    position_units        = position_units + velocity_units_s * step_s + acceleration_units_s2 * step_s ^ 2 / 2 + jerk_units_s3 * step_s ^ 3 / 6;
    velocity_units_s      = velocity_units_s + acceleration_units_s2 * step_s + jerk_units_s3 * step_s ^ 2 / 2;
    acceleration_units_s2 = acceleration_units_s2 + jerk_units_s3 * step_s;
end
terminalState = struct("position_units", position_units, ...
    "velocity_units_s", velocity_units_s, ...
    "acceleration_units_s2", acceleration_units_s2);
initialTime_s = initialState.time_s;
finalTime_s   = initialTime_s + relativeBreak_s(end);
polynomial    = struct("Degree", 3, "SegmentCount", segmentCount, ...
    "SegmentStartTime_s", initialTime_s + relativeBreak_s(1:end - 1), ...
    "SegmentDuration_s", segmentDuration_s, ...
    "SegmentBreakTau", relativeBreak_s / relativeBreak_s(end), ...
    "FinalTime_s", finalTime_s, "positionPower_units", positionPower_units, ...
    "velocityPower_units_s", velocityPower_units_s, ...
    "accelerationPower_units_s2", accelerationPower_units_s2, ...
    "jerkPower_units_s3", reshape(segmentJerk_units_s3, ...
    segmentCount, dimensionCount, 1), "TerminalState", terminalState);
end
