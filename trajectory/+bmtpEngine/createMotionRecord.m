function [candidate, terminalState] = createMotionRecord(candidate, initialState, relativeBreak_s, segmentJerk_units_s3, sampleStep_s, seedSource)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.createMotionRecord( ...
%       struct(), initialState, [], [], sampleStep_s, seedSource)
%   [candidate, terminalState] = ...
%       bmtpEngine.createMotionRecord( ...
%       candidate, initialState, relativeBreak_s, segmentJerk_units_s3, ...
%       sampleStep_s, seedSource)
%
% PURPOSE
%   - Create the shared candidate record and exactly integrate a
%     sequence of constant-jerk intervals without changing its time partition.
%
% INPUTS
%   - candidate (scalar struct)
%       Empty creates the common record; nonempty preserves caller metadata.
%   - initialState (scalar struct)
%       Requires one-by-D position, velocity, and acceleration fields.
%   - relativeBreak_s (N-plus-one vector, scalar polynomial struct, or empty)
%       Exact relative event times from zero through the motion duration,
%       or an already assembled finite polynomial. Empty returns only the
%       stable record.
%   - segmentJerk_units_s3 (N-by-D numeric)
%       Constant jerk on each corresponding event interval.
%   - sampleStep_s (positive scalar)
%       Output-history spacing; exact event times are always retained.
%   - seedSource (scalar text)
%       Input-derived construction label copied to the candidate.
%
% OUTPUTS
%   - candidate (scalar struct)
%       Stable exact-motion record with polynomial and sampled histories.
%   - terminalState (scalar struct)
%       Analytically integrated terminal position, velocity, and acceleration.
%
% UNITS
%   - Position is coordinate units and time is seconds. Derivatives use units/s,
%     units/s^2, and units/s^3. Histories are N-by-D.
%

%% Section 1: Create The Stable Record

dimensionCount = numel(initialState.position_units);
terminalState  = struct();
terminalState.position_units        = zeros(1, dimensionCount);
terminalState.velocity_units_s      = zeros(1, dimensionCount);
terminalState.acceleration_units_s2 = zeros(1, dimensionCount);
if isempty(fieldnames(candidate))
    candidate = struct("Success", false, "OptimizerFeasible", false, ...
        "Message", "Exact motion was not constructed.", ...
        "TerminationReason", "notRun", "SeedIndex", 1, ...
        "SeedSource", string(seedSource), ...
        "ArrivalTime_s", NaN, "TrajectoryDuration_s", NaN, ...
        "MinimumAxisDuration_s", NaN(1, dimensionCount), ...
        "StraightProgressMinimumDuration_s", NaN, ...
        "UsedStraightProgress", false, "MotionLength_units", NaN, ...
        "IntegratedSquaredJerk_units2_s5", NaN, ...
        "MaximumConstraintViolation", Inf, "time_s", zeros(0, 1), ...
        "position_units", zeros(0, dimensionCount), ...
        "velocity_units_s", zeros(0, dimensionCount), ...
        "acceleration_units_s2", zeros(0, dimensionCount), ...
        "jerk_units_s3", zeros(0, dimensionCount), "Polynomial", struct(), ...
        "SeedCorridorBoundary_units", zeros(0, 2), ...
        "SeedCorridor", struct([]), ...
        "PlaneCertificate", emptyPlaneCertificate(), ...
        "SolverDiagnostics", struct());
end
candidate.SeedSource = string(seedSource);
if isempty(relativeBreak_s)
    return;
end

%% Section 2: Integrate The Exact Event Word

if isstruct(relativeBreak_s)
    polynomial     = relativeBreak_s;
    requiredFields = {'Degree', 'SegmentCount', 'SegmentStartTime_s', ...
        'SegmentDuration_s', 'FinalTime_s', 'jerkPower_units_s3', 'TerminalState'};
    polynomialIsComplete = isscalar(polynomial) && all(isfield(polynomial, requiredFields));
    if polynomialIsComplete
        degree        = polynomial.Degree;
        degreeIsValid = isnumeric(degree) && isscalar(degree) && isfinite(degree) && degree >= 0 && degree == floor(degree);
    else
        degreeIsValid = false;
    end
    if ~polynomialIsComplete || ~degreeIsValid
        error("createMotionRecord:InvalidPolynomial", "An assembled polynomial must be scalar, complete, and have finite degree.");
    end
    segmentCount      = polynomial.SegmentCount;
    segmentDuration_s = polynomial.SegmentDuration_s;
    finalTime_s       = polynomial.FinalTime_s;
    terminalState     = polynomial.TerminalState;
else
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
    % Process each segment while assembling the complete motion or interval result.
    for segmentIndex = 1:segmentCount
        step_s      = segmentDuration_s(segmentIndex);
        jerk_units_s3 = segmentJerk_units_s3(segmentIndex, :);
        [positionPower_units(segmentIndex, :, :), ...
            velocityPower_units_s(segmentIndex, :, :), accelerationPower_units_s2(segmentIndex, :, :)] = bmtpEngine.createConstantJerkPowerCoefficients(position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3, step_s);
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

%% Section 3: Sample And Update Shared Quality Fields

initialTime_s = initialState.time_s;
sampleTime_s  = (initialTime_s:sampleStep_s:finalTime_s).';
sampleTime_s  = unique([sampleTime_s; polynomial.SegmentStartTime_s; finalTime_s]);
[sampleTime_s, position_units, velocity_units_s, acceleration_units_s2, ...
    jerk_units_s3] = bmtpEngine.evaluatePolynomial(polynomial, sampleTime_s);
candidate.ArrivalTime_s        = finalTime_s;
candidate.TrajectoryDuration_s = finalTime_s - initialTime_s;
candidate.time_s               = sampleTime_s;
candidate.position_units         = position_units;
candidate.velocity_units_s       = velocity_units_s;
candidate.acceleration_units_s2  = acceleration_units_s2;
candidate.jerk_units_s3          = jerk_units_s3;
candidate.Polynomial           = polynomial;
candidate.MotionLength_units     = sum(vecnorm(diff(position_units), 2, 2));
if polynomial.Degree <= 3
    jerk = reshape(polynomial.jerkPower_units_s3, segmentCount, dimensionCount);
    candidate.IntegratedSquaredJerk_units2_s5 = sum(segmentDuration_s .* sum(jerk .^ 2, 2));
else
    integratedSquaredJerk_units2_s5 = 0;
    % Process each segment while assembling the complete motion or interval result.
    for segmentIndex = 1:segmentCount
        % Evaluate each coordinate axis and combine its limiting result.
        for axisIndex = 1:dimensionCount
            jerkPower                     = reshape(polynomial.jerkPower_units_s3(segmentIndex, axisIndex, :), 1, []);
            squaredPower                  = conv(jerkPower, jerkPower);
            integratedSquaredJerk_units2_s5 = integratedSquaredJerk_units2_s5 + segmentDuration_s(segmentIndex) * sum(squaredPower ./ (1:numel(squaredPower)));
        end
    end
    candidate.IntegratedSquaredJerk_units2_s5 = integratedSquaredJerk_units2_s5;
end
end

%% Section 4: Local Functions

function certificate = emptyPlaneCertificate()
    % Initialize an empty separation certificate.
    emptyPlane = struct();
    emptyPlane.Active        = false;
    emptyPlane.Normal        = zeros(2, 2);
    emptyPlane.Offset_units    = zeros(1, 2);
    emptyPlane.SignedGap_units = NaN;
    emptyPlane.Verified      = false;
    emptyPlane.ExitFlag      = NaN;
    certificate = struct("Kind", "", "Passed", false, ...
        "ExactRegionCount", 0, "SolverRegionCount", 0, ...
        "Regions_units", {cell(0, 1)}, ...
        "Planes", repmat(emptyPlane, 0, 0), ...
        "RequiredGap_units", NaN, "RoundoffReserve_units", NaN, ...
        "MinimumSignedGap_units", NaN, "CoveragePassed", false, ...
        "Coverage", struct(), "AllPairCount", 0, ...
        "VerifiedPairCount", 0, "ReusedPairCount", 0, ...
        "AnalyticPairCount", 0, "ConicPairCount", 0);
end
