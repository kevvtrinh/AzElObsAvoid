function [controls_units, durations_s, powers_units] = createC3Chord(start_units, goal_units, limits)
%% Section 0: Header & Readme
% SYNTAX
%   [controls_units, durations_s, powers_units] = ...
%       bmtpEngine.createC3Chord(start_units, goal_units, limits)
%**************************************************************************
% PURPOSE
%   - Smooth a rest-to-rest chord into C3 quintic spans with zero end jerk.
%**************************************************************************
% INPUTS
%   - start_units (1-by-2 numeric row)
%       Chord start point.
%   - goal_units (1-by-2 numeric row)
%       Distinct chord endpoint.
%   - limits (scalar struct)
%       Positive per-axis velocity, acceleration, and jerk limits.
%**************************************************************************
% OUTPUTS
%   - controls_units (S-by-6-by-2 numeric array)
%       Quintic Bezier controls, one page per coordinate.
%   - durations_s (positive numeric column)
%       Physical duration of each quintic span.
%   - powers_units (S-by-2-by-6 numeric array)
%       Matching normalized ascending-power position coefficients.
%**************************************************************************
% UNITS
%   - Position is coordinate units and duration is seconds.
%**************************************************************************

%% Section 1: Solve The Scalar Jerk-Limited Progress Profile
% Convolution preserves derivative bounds and total displacement. Two causal
% box averages add twice the smoothing width to the original motion duration.
chordLength_units = abs(goal_units - start_units);
scalarLimits      = struct( ...
    'maxVelocity_units_s',      min(limits.maxVelocity_units_s ./ chordLength_units) * [1, 1], ...
    'maxAcceleration_units_s2', min(limits.maxAcceleration_units_s2 ./ chordLength_units) * [1, 1], ...
    'maxJerk_units_s3',         min(limits.maxJerk_units_s3 ./ chordLength_units) * [1, 1]);
[~, originalTimes_s, phases] = bmtpEngine.createJerkLimitedChord( ...
    [0, 0], [1, 0], scalarLimits, 5);

%% Section 2: Place The Smoothed Knot Mesh
% Smooth on the shortest jerk-ramp scale. Zero-jerk holds and cruises may
% be arbitrarily short or long without changing the physical ramp width.
rampPhase      = phases.Jerk_units_s3(:, 1) ~= 0;
width_s        = min(originalTimes_s(rampPhase)) / 10;
sourceBreaks_s = [0; cumsum(originalTimes_s)];
breaks_s       = unique([sourceBreaks_s; sourceBreaks_s + width_s; sourceBreaks_s + 2 * width_s]);

% Merge only arithmetic duplicates that are also negligible relative to
% the smoothing kernel; a long cruise must not erase short ramp knots.
knotTolerance_s = min(64 * eps(max(1, max(abs(breaks_s)))), width_s / 64);
breaks_s        = breaks_s([true; diff(breaks_s) > knotTolerance_s]);

durations_s             = diff(breaks_s);
powers_units            = zeros(numel(durations_s), 2, 6);
controls_units          = zeros(numel(durations_s), 6, 2);
progress                = 0;
progressRate_s1         = 0;
progressAcceleration_s2 = 0;
direction_units         = goal_units - start_units;

%% Section 3: Integrate Quadratic Jerk Into Quintic Position
for spanIndex = 1:numel(durations_s)
    duration_s    = durations_s(spanIndex);
    sampleTimes_s = breaks_s(spanIndex) + duration_s * [0, 0.5, 1];
    jerk_s3      = (originalVelocity(sampleTimes_s) - ...
        2 * originalVelocity(sampleTimes_s - width_s) + ...
        originalVelocity(sampleTimes_s - 2 * width_s)) / width_s ^ 2;
    jerkPower_s3 = [jerk_s3(1), ...
        4 * jerk_s3(2) - 3 * jerk_s3(1) - jerk_s3(3), ...
        2 * jerk_s3(1) + 2 * jerk_s3(3) - 4 * jerk_s3(2)];
    progressPower = [progress, progressRate_s1 * duration_s, ...
        progressAcceleration_s2 * duration_s ^ 2 / 2, ...
        jerkPower_s3 .* duration_s ^ 3 ./ [6, 24, 60]];
    physicalPower_units       = direction_units.' * progressPower;
    physicalPower_units(:, 1) = physicalPower_units(:, 1) + start_units.';

    powers_units(spanIndex, :, :)   = physicalPower_units;
    controls_units(spanIndex, :, :) = bmtpEngine.powerToBernstein(physicalPower_units.');
    progress                = sum(progressPower);
    progressRate_s1         = sum((1:5) .* progressPower(2:6)) / duration_s;
    progressAcceleration_s2 = sum((1:4) .* (2:5) .* progressPower(3:6)) / duration_s ^ 2;
end

%% Section 4: Local Functions

    function velocity = originalVelocity(times_s)
        % Evaluate the unsmoothed analytic phase velocity at each time.
        % Times outside the original motion contribute exactly zero velocity.
        velocity = zeros(size(times_s));
        for sampleIndex = 1:numel(times_s)
            if times_s(sampleIndex) <= 0 || times_s(sampleIndex) >= sourceBreaks_s(end)
                continue
            end
            phaseIndex = find(sourceBreaks_s <= times_s(sampleIndex), 1, 'last');
            elapsed_s  = times_s(sampleIndex) - sourceBreaks_s(phaseIndex);
            velocity(sampleIndex) = phases.Velocity_units_s(phaseIndex, 1) + ...
                elapsed_s * phases.Acceleration_units_s2(phaseIndex, 1) + ...
                elapsed_s ^ 2 * phases.Jerk_units_s3(phaseIndex, 1) / 2;
        end
    end
end
