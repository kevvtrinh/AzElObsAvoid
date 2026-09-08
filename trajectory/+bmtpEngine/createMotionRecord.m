function [candidate, terminalState] = createMotionRecord(candidate, initialState, relativeBreak_s, segmentJerk_units_s3, sampleStep_s, seedSource)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.createMotionRecord( ...
%       struct(), initialState, [], [], sampleStep_s, seedSource)
%   [candidate, terminalState] = ...
%       bmtpEngine.createMotionRecord( ...
%       candidate, initialState, relativeBreak_s, segmentJerk_units_s3, ...
%       sampleStep_s, seedSource)
% PURPOSE
%   - Create the shared candidate record and exactly integrate a
%     sequence of constant-jerk intervals without changing its time partition.
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
% OUTPUTS
%   - candidate (scalar struct)
%       Stable exact-motion record with polynomial and sampled histories.
%       MotionLength_units integrates polynomial speed independently of samples.
%   - terminalState (scalar struct)
%       Analytically integrated terminal position, velocity, and acceleration.
% UNITS
%   - Position is coordinate units and time is seconds. Derivatives use units/s,
%     units/s^2, and units/s^3. Histories are N-by-D.

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
    [polynomial, terminalState] = motionCore.createJerkPolynomial(initialState, relativeBreak_s, segmentJerk_units_s3);
    segmentCount = polynomial.SegmentCount;
    segmentDuration_s = polynomial.SegmentDuration_s;
    finalTime_s = polynomial.FinalTime_s;
end

%% Section 3: Sample And Update Shared Quality Fields

initialTime_s = initialState.time_s;
sampleTime_s  = (initialTime_s:sampleStep_s:finalTime_s).';
sampleTime_s  = unique([sampleTime_s; polynomial.SegmentStartTime_s; finalTime_s]);
[sampleTime_s, position_units, velocity_units_s, acceleration_units_s2, ...
    jerk_units_s3] = motionCore.evaluatePolynomial(polynomial, sampleTime_s);
candidate.ArrivalTime_s        = finalTime_s;
candidate.TrajectoryDuration_s = finalTime_s - initialTime_s;
candidate.time_s               = sampleTime_s;
candidate.position_units         = position_units;
candidate.velocity_units_s       = velocity_units_s;
candidate.acceleration_units_s2  = acceleration_units_s2;
candidate.jerk_units_s3          = jerk_units_s3;
candidate.Polynomial           = polynomial;
candidate.MotionLength_units     = bmtpEngine.measurePolynomialLength(polynomial);
if polynomial.Degree <= 3
    jerk = reshape(polynomial.jerkPower_units_s3, segmentCount, dimensionCount);
    candidate.IntegratedSquaredJerk_units2_s5 = sum(segmentDuration_s .* sum(jerk .^ 2, 2));
else
    coefficients = permute(polynomial.jerkPower_units_s3, [3 1 2]);
    order = (1:size(coefficients, 1)).';
    gram = 1 ./ (order + order.' - 1);
    segmentCosts = sum(coefficients .* pagemtimes(gram, coefficients), 1);
    candidate.IntegratedSquaredJerk_units2_s5 = sum(segmentDuration_s(:) .* reshape(segmentCosts, segmentCount, dimensionCount), 'all');
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
