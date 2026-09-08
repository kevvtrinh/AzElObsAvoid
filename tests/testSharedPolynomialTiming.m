function tests = testSharedPolynomialTiming
%% Section 0: Header & Readme
% SYNTAX: tests = testSharedPolynomialTiming
% PURPOSE: Preserve common-clock arithmetic while supporting unequal clocks.
% INPUTS: None.
% OUTPUTS: MATLAB tests against the preceding scalar implementation.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
end

function testScalarPolynomialsAndDerivativeBoundsRemainExact(testCase)
    previous = rng(8710); cleanup = onCleanup(@() rng(previous));
    limits = struct('maxVelocity_units_s', [2 3], 'maxAcceleration_units_s2', [1 2], 'maxJerk_units_s3', [4 7]);
    for count = [1 3 12]
        for degree = [7 8]
            controls = rand(count, degree + 1, 2) + 25;
            for origin = [0 7]
                expected = createPowerPolynomialReference(controls, 0.7, origin);
                actual = bmtpEngine.createPowerPolynomial(controls, 0.7, origin);
                verifyEqual(testCase, actual, expected);
            end
            [common, individual] = bmtpEngine.findRequiredSegmentTime(controls, limits);
            verifyEqual(testCase, common, findRequiredSegmentTimeReference(controls, limits));
            for segment = 1:count
                verifyEqual(testCase, individual(segment), findRequiredSegmentTimeReference(controls(segment, :, :), limits));
            end
        end
    end
end

function testUnequalDurationsPreserveCubicDerivatives(testCase)
    durations = [0.3; 1.7; 0.8];
    starts = [0; cumsum(durations(1:end - 1))];
    powers = zeros(3, 2, 9);
    controls = zeros(3, 9, 2);
    for segment = 1:3
        origin = starts(segment);
        powers(segment, 1, 1:4) = [origin^3, 3*origin^2*durations(segment), 3*origin*durations(segment)^2, durations(segment)^3];
        powers(segment, 2, 1:3) = [origin^2, 2*origin*durations(segment), durations(segment)^2];
        for bernstein = 0:8
            for power = 0:bernstein
                controls(segment, bernstein + 1, :) = controls(segment, bernstein + 1, :) + reshape(powers(segment, :, power + 1), 1, 1, 2) * nchoosek(bernstein, power) / nchoosek(8, power);
            end
        end
    end
    polynomial = bmtpEngine.createPowerPolynomial(controls, durations, 7, powers);
    verifyEqual(testCase, polynomial.positionPower_units, powers);
    verifyEqual(testCase, polynomial.SegmentStartTime_s, 7 + starts);
    verifyEqual(testCase, polynomial.FinalTime_s, 7 + sum(durations));
    time = linspace(0, sum(durations) - 1e-5, 31).';
    [~, position, velocity, acceleration, jerk] = motionCore.evaluatePolynomial(polynomial, 7 + time);
    verifyEqual(testCase, position, [time.^3 time.^2], 'AbsTol', 1e-12);
    verifyEqual(testCase, velocity, [3*time.^2 2*time], 'AbsTol', 1e-12);
    verifyEqual(testCase, acceleration, [6*time 2+zeros(size(time))], 'AbsTol', 1e-12);
    verifyEqual(testCase, jerk, [6+zeros(size(time)) zeros(size(time))], 'AbsTol', 1e-12);
end

function testSharedRecordReplacesTheGeneralSolverExporter(testCase)
    controls = zeros(3, 9, 2);
    for segment = 1:3
        controls(segment, :, 1) = linspace(segment - 1, segment, 9);
        controls(segment, :, 2) = linspace(0, 0.1 * segment, 9);
    end
    prepared = struct('ControlPoint_units', controls, 'SegmentTime_s', 0.7, 'MotionCertificate', struct('MaximumViolation', 0));
    for origin = [0 7]
        initial = struct('time_s', origin, 'position_units', [0 0], 'velocity_units_s', [0 0], 'acceleration_units_s2', [0 0]);
        empty = bmtpEngine.createMotionRecord(struct(), initial, [], [], 0.1, "sharedExport");
        request = struct('InitialState', initial, 'Options', struct('SampleTime_s', 0.1));
        expected = createMotionOutputReference(empty, request, prepared);
        actual = bmtpEngine.createMotionRecord(empty, initial, expected.Polynomial, [], 0.1, "sharedExport");
        verifyEqual(testCase, actual.Polynomial, expected.Polynomial);
        verifyEqual(testCase, actual.TrajectoryDuration_s, expected.TrajectoryDuration_s);
        verifyEqual(testCase, actual.IntegratedSquaredJerk_units2_s5, expected.IntegratedSquaredJerk_units2_s5);
        % Relative-grid translation creates redundant points a few ulps apart.
        % Require both grids to cover each other, preserving every event;
        % evaluate the identical reference polynomial on the shared grid so
        % one-sided jerk at a join is compared at the same physical instant.
        timeDifferences = abs(actual.time_s - expected.time_s.');
        timeTolerance = 16 * eps(max(1, expected.Polynomial.FinalTime_s));
        verifyLessThanOrEqual(testCase, max(min(timeDifferences, [], 1)), timeTolerance);
        verifyLessThanOrEqual(testCase, max(min(timeDifferences, [], 2)), timeTolerance);
        verifyTrue(testCase, all(ismember(expected.Polynomial.SegmentStartTime_s, actual.time_s)));
        [~, position, velocity, acceleration, jerk] = motionCore.evaluatePolynomial(expected.Polynomial, actual.time_s);
        verifyEqual(testCase, actual.position_units, position);
        verifyEqual(testCase, actual.velocity_units_s, velocity);
        verifyEqual(testCase, actual.acceleration_units_s2, acceleration);
        verifyEqual(testCase, actual.jerk_units_s3, jerk);
    end
end
