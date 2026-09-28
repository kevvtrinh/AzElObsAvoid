function [controlPoint_units, segmentTime_s, powerCoefficients_units] = ...
    createC3Chord(start_units, goal_units, limits)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, powerCoefficients_units] = ...
%       bmtpEngine.motion.createC3Chord(start_units, goal_units, limits)
%**************************************************************************
% PURPOSE
%   - Build a straight-line trip from rest to rest. Start with a motion
%     limited by jerk, then smooth its abrupt jerk changes. Position and
%     its first three time derivatives join continuously (C3); jerk is
%     zero at departure and arrival.
%**************************************************************************
% INPUTS
%   - start_units (1-by-2 numeric row)
%       Starting [x, y] position.
%   - goal_units (1-by-2 numeric row)
%       Ending [x, y] position, different from the start.
%   - limits (scalar struct)
%       Positive maxVelocity_units_s, maxAcceleration_units_s2, and
%       maxJerk_units_s3 for each coordinate axis.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-6-by-2 numeric array)
%       Six Bezier controls for each of S degree-five segments. The last
%       dimension selects x or y.
%   - segmentTime_s (positive numeric column)
%       Duration of each of the S segments, in travel order.
%   - powerCoefficients_units (S-by-2-by-6 numeric array)
%       The same positions as c0 + c1 x u + ... + c5 x u^5. Each segment's
%       local fraction u runs from 0 to 1; coefficients run from c0 to c5.
%**************************************************************************
% UNITS
%   - Positions and position coefficients: coordinate units; durations:
%     seconds. Limits use coordinate units/s, units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Calculate Straight-Line Progress Within The Motion Limits

% Progress is 0 at start and 1 at goal. Convert each axis limit into a
% limit on progress, then use the smaller value. For 10 units of x travel
% at 2 units/s, progress may rise by at most 2 / 10 = 0.2 each second.
% An axis with no travel places no limit on progress.
axisDistance_units = abs(goal_units - start_units);
scalarLimits = struct( ...
    'maxVelocity_units_s',      min(limits.maxVelocity_units_s ./ axisDistance_units) * [1, 1], ...
    'maxAcceleration_units_s2', min(limits.maxAcceleration_units_s2 ./ axisDistance_units) * [1, 1], ...
    'maxJerk_units_s3',         min(limits.maxJerk_units_s3 ./ axisDistance_units) * [1, 1]);
[~, originalPhaseTime_s, originalMotionPhases] = bmtpEngine.motion.createJerkLimitedChord( ...
    [0, 0], [1, 0], scalarLimits, 5);

%% Section 2: Place Segment Joins For The Smoothed Motion

% Average the original motion over a short time window twice. Averaging
% softens each jerk jump without exceeding the original derivative limits.
% The trip takes 2 x window width longer. Set the width from the shortest
% phase with nonzero jerk so a cruise or hold does not choose the width.
phaseHasNonzeroJerk  = originalMotionPhases.Jerk_units_s3(:, 1) ~= 0;
smoothingWidth_s     = min(originalPhaseTime_s(phaseHasNonzeroJerk)) / 10;
originalBreakTimes_s = [0; cumsum(originalPhaseTime_s)];
segmentBreakTimes_s = unique( ...
    [originalBreakTimes_s; originalBreakTimes_s + smoothingWidth_s; originalBreakTimes_s + 2 * smoothingWidth_s]);

% Remove joins separated only by floating-point rounding. Keep the
% tolerance far below the window width so short smoothing ramps survive.
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

% Between joins, smoothed jerk is quadratic in local fraction u. Its
% values at u = 0, 0.5, and 1 determine that quadratic exactly. Integrate
% it three times to obtain the degree-five position for this segment.
for segmentIndex = 1:numel(segmentTime_s)
    segmentDuration_s = segmentTime_s(segmentIndex);
    sampleTimes_s     = segmentBreakTimes_s(segmentIndex) + segmentDuration_s * [0, 0.5, 1];
    % The second difference of the original progress rate gives the jerk
    % after two averages. Divide by the window width squared to keep its
    % physical units: progress per second cubed.
    sampledProgressJerk_s3 = (evaluateUnsmoothedProgressRate(sampleTimes_s) - ...
        2 * evaluateUnsmoothedProgressRate(sampleTimes_s - smoothingWidth_s) + ...
        evaluateUnsmoothedProgressRate(sampleTimes_s - 2 * smoothingWidth_s)) / smoothingWidth_s ^ 2;
    % Fit j(u) = a + b x u + c x u^2 through the three jerk samples.
    progressJerkCoefficients_s3 = [sampledProgressJerk_s3(1), ...
        4 * sampledProgressJerk_s3(2) - 3 * sampledProgressJerk_s3(1) - sampledProgressJerk_s3(3), ...
        2 * sampledProgressJerk_s3(1) + 2 * sampledProgressJerk_s3(3) - 4 * sampledProgressJerk_s3(2)];
    % The integration factors 6, 24, and 60 turn jerk's constant, linear,
    % and quadratic terms into position's cubic, quartic, and quintic terms.
    progressCoefficients = [progress, progressRate_s1 * segmentDuration_s, ...
        progressAcceleration_s2 * segmentDuration_s ^ 2 / 2, ...
        progressJerkCoefficients_s3 .* segmentDuration_s ^ 3 ./ [6, 24, 60]];

    % position = start + displacement x progress, applied to x and y.
    segmentPositionCoefficients_units = displacement_units.' * progressCoefficients;
    segmentPositionCoefficients_units(:, 1) = segmentPositionCoefficients_units(:, 1) + start_units.';

    powerCoefficients_units(segmentIndex, :, :) = segmentPositionCoefficients_units;
    controlPoint_units(segmentIndex, :, :)      = bmtpEngine.motion.powerToBernstein(segmentPositionCoefficients_units.');

    % Carry this segment's state at u = 1 into the next segment so the
    % position, speed, and acceleration meet at the join.
    progress               = sum(progressCoefficients);
    progressRate_s1         = sum((1:5) .* progressCoefficients(2:6)) / segmentDuration_s;
    progressAcceleration_s2 = sum((1:4) .* (2:5) .* progressCoefficients(3:6)) / segmentDuration_s ^ 2;
end

%% Section 4: Nested Function

    function sampledProgressRate_s1 = evaluateUnsmoothedProgressRate(requestedTimes_s)
        % Within each original phase, progress rate is
        % v(t) = v0 + a0 x t + j x t^2 / 2, measured from that phase's start.
        % Before departure and after arrival, return zero: the trip is at
        % rest. This nested function uses the phase states computed above.
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
