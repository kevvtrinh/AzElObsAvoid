function tests = testOffsetTranslationBatch
%% Section 0: Header & Readme
% SYNTAX: tests = testOffsetTranslationBatch
% PURPOSE: Compare batched polynomial translations with the frozen scalar engine.
% INPUTS: None.
% OUTPUTS: Deterministic MATLAB function tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
end

function testNonuniformKnotsAndShiftedClocks(testCase)
    rng(281, 'twister');
    for caseIndex = 1:12
        dimensionCount = 1 + mod(caseIndex, 3);
        initialState = struct('time_s', 1e3 * (caseIndex - 1), ...
            'position_units', randn(1, dimensionCount), ...
            'velocity_units_s', zeros(1, dimensionCount), ...
            'acceleration_units_s2', zeros(1, dimensionCount));
        goalState = initialState;
        goalState.time_s = initialState.time_s + 20;
        goalState.position_units = initialState.position_units + 4 * randn(1, dimensionCount);
        limits = struct('maxVelocity_units_s', 2 * ones(1, dimensionCount), ...
            'maxAcceleration_units_s2', ones(1, dimensionCount), ...
            'maxJerk_units_s3', 3 * ones(1, dimensionCount));
        options = struct('GoalTimeMode', 'fixedArrival', 'SampleTime_s', 0.2);
        base = bmtpEngine.createDirectMotion(initialState, goalState, limits, options);
        verifyTrue(testCase, base.Success);
        knotTime_s = initialState.time_s + [0; 0.7; 4.3; 13.1; 20];
        offsets_units = [0; randn(3, 1); 0];
        velocities_units_s = [0; NaN; 0; NaN; 0];
        for axisIndex = 1:dimensionCount
            reference = createOffsetSplineMotionReference(base, knotTime_s, offsets_units, axisIndex, initialState, 0.2, 'comparison', velocities_units_s);
            candidate = bmtpEngine.createOffsetSplineMotion(base, knotTime_s, offsets_units, axisIndex, initialState, 0.2, 'comparison', velocities_units_s);
            verifyEqual(testCase, candidate, reference);
            % A quintic base exercises higher-degree source coefficients too.
            reference = createOffsetSplineMotionReference(reference, knotTime_s, -offsets_units / 2, axisIndex, initialState, 0.2, 'comparison');
            candidate = bmtpEngine.createOffsetSplineMotion(candidate, knotTime_s, -offsets_units / 2, axisIndex, initialState, 0.2, 'comparison');
            verifyEqual(testCase, candidate, reference);
        end
    end
end
