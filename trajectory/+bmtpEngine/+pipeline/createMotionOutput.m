function candidate = createMotionOutput(candidate, solverRequest, preparedMotion)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.pipeline.createMotionOutput(candidate, solverRequest, preparedMotion)
%**************************************************************************
% PURPOSE
%   - Fill the candidate with its polynomial, sampled motion history,
%     arrival time, path length, and integrated squared jerk.
%**************************************************************************
% INPUTS
%   - candidate (scalar struct)
%       Empty candidate record whose stable fields are filled in.
%   - solverRequest (scalar struct)
%       Checked inputs supplying the start time and output sample spacing.
%   - preparedMotion (scalar struct)
%       Final curve controls, segment durations, coefficients, and motion checks.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Polynomial, sampled histories, and motion measures.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Create The Polynomial And Output Samples

polynomial = bmtpEngine.motion.createPowerPolynomial(preparedMotion.ControlPoint_units, ...
    preparedMotion.SegmentTime_s, solverRequest.InitialState.time_s, preparedMotion.GivenPower_units, ...
    preparedMotion.FinalTime_s);
sampledMotion = samplePolynomial(polynomial, solverRequest.Options.SampleTime_s);

%% Section 2: Measure Arrival, Path Length, And Jerk Cost

candidate.ArrivalTime_s        = polynomial.FinalTime_s;
candidate.TrajectoryDuration_s = polynomial.FinalTime_s - solverRequest.InitialState.time_s;
candidate.MotionLength_units   = 0;
% Path length = segment duration x integral(speed(u), u = 0..1),
% where u = elapsed time / segment duration.
% This measures the curve itself rather than distances between output samples.
for segmentIndex = 1:polynomial.SegmentCount
    velocityCoefficients_units_s = squeeze(polynomial.velocityPower_units_s(segmentIndex, :, :));
    speedAtFractionHandle = @(segmentFraction) hypot( ...
        polyval(fliplr(velocityCoefficients_units_s(1, :)), segmentFraction), ...
        polyval(fliplr(velocityCoefficients_units_s(2, :)), segmentFraction));
    candidate.MotionLength_units = candidate.MotionLength_units + ...
        polynomial.SegmentDuration_s(segmentIndex) * ...
        integral(speedAtFractionHandle, 0, 1, 'AbsTol', 1e-11, 'RelTol', 1e-11);
end
candidate.IntegratedSquaredJerk_units2_s5 = integratedSquaredJerk(polynomial);
candidate.MaximumConstraintViolation     = preparedMotion.MotionProof.MaximumViolation;

%% Section 3: Store The Motion Histories And Polynomial

for fieldName = ["time_s", "position_units", "velocity_units_s", ...
        "acceleration_units_s2", "jerk_units_s3"]
    candidate.(fieldName) = sampledMotion.(fieldName);
end
candidate.Polynomial = polynomial;
end

%% Section 4: Local Functions

function sampledMotion = samplePolynomial(polynomial, sampleTime_s)
    % Use regularly spaced times plus every segment start and the final time.
    % This includes segment joins even when the sample spacing skips them.
    initialTime_s    = polynomial.SegmentStartTime_s(1);
    motionDuration_s = polynomial.FinalTime_s - initialTime_s;
    % Adding a large start time can round two nearby times to the same value.
    % Remove duplicates after that addition so exported times stay distinct.
    sampleTimes_s = unique([initialTime_s + (0:sampleTime_s:motionDuration_s).'; ...
        polynomial.SegmentStartTime_s; polynomial.FinalTime_s]);
    [time_s, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3] = ...
        bmtpEngine.motion.evaluatePolynomial(polynomial, sampleTimes_s);
    sampledMotion = struct( ...
        "time_s",                time_s, ...
        "position_units",        position_units, ...
        "velocity_units_s",      velocity_units_s, ...
        "acceleration_units_s2", acceleration_units_s2, ...
        "jerk_units_s3",         jerk_units_s3);
end

function integratedJerkCost_units2_s5 = integratedSquaredJerk(polynomial)
    % Square each axis's jerk polynomial and integrate every power exactly.
    % For powers i and j, integral(u^(i+j), 0..1) = 1 / (i+j+1).
    % Multiplying by segment duration converts the fraction integral to seconds.
    jerkCoefficients_units_s3 = permute(polynomial.jerkPower_units_s3, [3, 1, 2]);
    coefficientIndices       = (1:size(jerkCoefficients_units_s3, 1)).';
    powerProductIntegrals    = 1 ./ (coefficientIndices + coefficientIndices.' - 1);
    segmentJerkCost_units2_s6 = sum(jerkCoefficients_units_s3 .* ...
        pagemtimes(powerProductIntegrals, jerkCoefficients_units_s3), 1);
    integratedJerkCost_units2_s5 = sum(polynomial.SegmentDuration_s(:) .* ...
        reshape(segmentJerkCost_units2_s6, polynomial.SegmentCount, 2), "all");
end
