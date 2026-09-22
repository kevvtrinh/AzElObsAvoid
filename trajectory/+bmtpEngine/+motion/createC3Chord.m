function [controlPoint_units, segmentTime_s, powerCoefficients_units] = ...
    createC3Chord(start_units, goal_units, limits)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, powerCoefficients_units] = ...
%       bmtpEngine.motion.createC3Chord(start_units, goal_units, limits)
%**************************************************************************
% PURPOSE
%   - Create straight-line motion that starts and ends at rest. Smooth the
%     jerk changes so position, velocity, acceleration, and jerk stay
%     continuous across segment joins (C3), with zero jerk at both ends.
%**************************************************************************
% INPUTS
%   - start_units (1-by-2 numeric row)
%       Straight-line start position.
%   - goal_units (1-by-2 numeric row)
%       Goal position, different from the start.
%   - limits (scalar struct)
%       Positive per-axis velocity, acceleration, and jerk limits.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-6-by-2 numeric array)
%       Six Bezier control points for each degree-five position segment.
%       The third dimension selects x or y.
%   - segmentTime_s (positive numeric column)
%       Duration of each motion segment.
%   - powerCoefficients_units (S-by-2-by-6 numeric array)
%       The same positions as c0 + c1 x u + ... + c5 x u^5, where u runs
%       from 0 to 1 within each segment. Coefficients are ordered c0 to c5.
%**************************************************************************
% UNITS
%   - Position is coordinate units and duration is seconds.
%**************************************************************************

%% Section 1: Calculate Straight-Line Progress Within The Motion Limits

% Progress runs from 0 at start to 1 at goal. Divide each axis limit by its
% travel distance, then use the tighter limit so both axes remain allowed.
% An axis with zero travel contributes Inf and does not restrict progress.
axisDistance_units = abs(goal_units - start_units);
scalarLimits = struct( ...
    'maxVelocity_units_s',      min(limits.maxVelocity_units_s ./ axisDistance_units) * [1, 1], ...
    'maxAcceleration_units_s2', min(limits.maxAcceleration_units_s2 ./ axisDistance_units) * [1, 1], ...
    'maxJerk_units_s3',         min(limits.maxJerk_units_s3 ./ axisDistance_units) * [1, 1]);
[~, originalPhaseTime_s, originalMotionPhases] = bmtpEngine.motion.createJerkLimitedChord( ...
    [0, 0], [1, 0], scalarLimits, 5);

%% Section 2: Place Segment Joins For The Smoothed Motion

% Average the original motion over a short time window, twice. This keeps
% the displacement, speed, acceleration, and jerk limits while adding
% 2 x window width to the duration. Use the shortest phase with nonzero
% jerk; a short hold or long cruise should not set the smoothing width.
phaseHasNonzeroJerk  = originalMotionPhases.Jerk_units_s3(:, 1) ~= 0;
smoothingWidth_s     = min(originalPhaseTime_s(phaseHasNonzeroJerk)) / 10;
originalBreakTimes_s = [0; cumsum(originalPhaseTime_s)];
segmentBreakTimes_s = unique( ...
    [originalBreakTimes_s; originalBreakTimes_s + smoothingWidth_s; originalBreakTimes_s + 2 * smoothingWidth_s]);

% Remove times separated only by rounding error. Cap that tolerance using
% the smoothing width so a long cruise cannot erase short ramp segments.
joinTimeTolerance_s = min(64 * eps(max(1, max(abs(segmentBreakTimes_s)))), smoothingWidth_s / 64);
segmentBreakTimes_s = segmentBreakTimes_s([true; diff(segmentBreakTimes_s) > joinTimeTolerance_s]);

segmentTime_s           = diff(segmentBreakTimes_s);
powerCoefficients_units = zeros(numel(segmentTime_s), 2, 6);
controlPoint_units      = zeros(numel(segmentTime_s), 6, 2);
progress               = 0;
progressRate_s1         = 0;
progressAcceleration_s2 = 0;
displacement_units      = goal_units - start_units;

%% Section 3: Integrate The Smoothed Jerk Into Position

% Between these joins the smoothed jerk is quadratic. Its values at the
% start, middle, and end determine that quadratic exactly. Integrating
% three times gives degree-five position, with velocity and acceleration.
for segmentIndex = 1:numel(segmentTime_s)
    segmentDuration_s = segmentTime_s(segmentIndex);
    sampleTimes_s     = segmentBreakTimes_s(segmentIndex) + segmentDuration_s * [0, 0.5, 1];
    % After two averages, jerk is this second difference of the original
    % progress rate divided by the smoothing width squared.
    sampledProgressJerk_s3 = (evaluateUnsmoothedProgressRate(sampleTimes_s) - ...
        2 * evaluateUnsmoothedProgressRate(sampleTimes_s - smoothingWidth_s) + ...
        evaluateUnsmoothedProgressRate(sampleTimes_s - 2 * smoothingWidth_s)) / smoothingWidth_s ^ 2;
    progressJerkCoefficients_s3 = [sampledProgressJerk_s3(1), ...
        4 * sampledProgressJerk_s3(2) - 3 * sampledProgressJerk_s3(1) - sampledProgressJerk_s3(3), ...
        2 * sampledProgressJerk_s3(1) + 2 * sampledProgressJerk_s3(3) - 4 * sampledProgressJerk_s3(2)];
    progressCoefficients = [progress, progressRate_s1 * segmentDuration_s, ...
        progressAcceleration_s2 * segmentDuration_s ^ 2 / 2, ...
        progressJerkCoefficients_s3 .* segmentDuration_s ^ 3 ./ [6, 24, 60]];

    % position = start + displacement x progress, applied to x and y.
    segmentPositionCoefficients_units = displacement_units.' * progressCoefficients;
    segmentPositionCoefficients_units(:, 1) = segmentPositionCoefficients_units(:, 1) + start_units.';

    powerCoefficients_units(segmentIndex, :, :) = segmentPositionCoefficients_units;
    controlPoint_units(segmentIndex, :, :)      = bmtpEngine.motion.powerToBernstein(segmentPositionCoefficients_units.');

    % Start the next segment at this segment's ending state (u = 1).
    progress               = sum(progressCoefficients);
    progressRate_s1         = sum((1:5) .* progressCoefficients(2:6)) / segmentDuration_s;
    progressAcceleration_s2 = sum((1:4) .* (2:5) .* progressCoefficients(3:6)) / segmentDuration_s ^ 2;
end

%% Section 4: Nested Function

    function sampledProgressRate_s1 = evaluateUnsmoothedProgressRate(requestedTimes_s)
        % Evaluate the original progress rate: v = v0 + a0 x t + j x t^2 / 2.
        % Before departure and after arrival, the vehicle is at rest.
        % This nested function reads the original phase states above.
        sampledProgressRate_s1 = zeros(size(requestedTimes_s));
        for sampleIndex = 1:numel(requestedTimes_s)
            if requestedTimes_s(sampleIndex) <= 0 || requestedTimes_s(sampleIndex) >= originalBreakTimes_s(end)
                continue
            end
            phaseIndex         = find(originalBreakTimes_s <= requestedTimes_s(sampleIndex), 1, 'last');
            phaseElapsedTime_s = requestedTimes_s(sampleIndex) - originalBreakTimes_s(phaseIndex);
            sampledProgressRate_s1(sampleIndex) = originalMotionPhases.Velocity_units_s(phaseIndex, 1) + ...
                phaseElapsedTime_s * originalMotionPhases.Acceleration_units_s2(phaseIndex, 1) + ...
                phaseElapsedTime_s ^ 2 * originalMotionPhases.Jerk_units_s3(phaseIndex, 1) / 2;
        end
    end
end
