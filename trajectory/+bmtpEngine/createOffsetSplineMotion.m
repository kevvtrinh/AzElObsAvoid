function candidate = createOffsetSplineMotion(baseMotion, knotTime_s, knotOffset_units, axisIndex, initialState, sampleStep_s, seedSource, knotVelocity_units_s)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.createOffsetSplineMotion( ...
%       baseMotion, knotTime_s, knotOffset_units, axisIndex, ...
%       initialState, sampleStep_s, seedSource)
%   candidate = bmtpEngine.createOffsetSplineMotion(baseMotion, knotTime_s, ...
%       knotOffset_units, axisIndex, initialState, sampleStep_s, seedSource, knotVelocity_units_s)
%
% PURPOSE
%   - Add a minimum-integrated-jerk scalar offset to one motion coordinate.
%   - Keep the motion duration unchanged and represent the offset with fifth-degree polynomials.
%
% INPUTS
%   - baseMotion (scalar trajectory-engine result struct)
%       Requires a complete Polynomial and stable motion-result fields.
%   - knotTime_s, knotOffset_units (matching numeric column vectors)
%       Absolute increasing knot times and prescribed scalar offsets.
%   - axisIndex (positive integer scalar)
%       Coordinate receiving the additive offset.
%   - initialState (scalar struct)
%       Requires time_s, position_units, velocity_units_s, and
%       acceleration_units_s2.
%   - sampleStep_s (positive numeric scalar)
%       Requested output-history spacing in seconds.
%   - seedSource (scalar text)
%       Input-driven construction label copied to the motion record.
%   - knotVelocity_units_s (optional vector matching knotTime_s)
%       Prescribed offset velocities. NaN leaves an interior velocity free.
%       Omitted or empty leaves all interior velocities free; endpoints stay zero.
%
% OUTPUTS
%   - candidate (scalar trajectory-engine result struct)
%       Motion record containing the composite polynomial and histories.
%
% UNITS
%   - Position and offsets are coordinate units; time is seconds; derivatives use
%     units/s, units/s^2, and units/s^3. Histories are N-by-D.
%

%% Section 1: Validate The Clock And Offset Knots

if nargin < 7 || nargin > 8 || ~isstruct(baseMotion) || ~isscalar(baseMotion) || ~isfield(baseMotion, "Polynomial")
    error("createOffsetSplineMotion:InvalidCall", "Seven or eight inputs and a scalar baseMotion.Polynomial are required.");
end
knotTime_s     = double(knotTime_s(:));
knotOffset_units = double(knotOffset_units(:));
dimensionCount = size(baseMotion.Polynomial.positionPower_units, 2);
knotsAreValid  = numel(knotTime_s) >= 2 && numel(knotOffset_units) == numel(knotTime_s) && all(isfinite(knotTime_s)) && all(isfinite(knotOffset_units)) && all(diff(knotTime_s) > 0);
validateattributes(axisIndex, {'numeric'}, {'real', 'finite', 'scalar', 'integer', '>=', 1, '<=', dimensionCount});
validateattributes(sampleStep_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
seedSource = string(seedSource);
if ~knotsAreValid || ~isscalar(seedSource)
    error("createOffsetSplineMotion:InvalidKnots", "Knot times must increase and match finite offsets; seedSource is scalar.");
end
if nargin < 8 || isempty(knotVelocity_units_s)
    knotVelocity_units_s = NaN(size(knotTime_s));
end
validateattributes(knotVelocity_units_s, {'numeric'}, {'real', 'vector', 'numel', numel(knotTime_s)});
knotVelocity_units_s = double(knotVelocity_units_s(:));
if any(isinf(knotVelocity_units_s)) || any(isfinite(knotVelocity_units_s([1 end])) & knotVelocity_units_s([1 end]) ~= 0)
    error("createOffsetSplineMotion:InvalidKnotVelocity", "Offset velocities must be finite or NaN, with zero or unspecified endpoint values.");
end
clockTolerance_s = 1024 * eps(max(1, max(abs(knotTime_s))));
baseStartTime_s  = baseMotion.Polynomial.SegmentStartTime_s(1);
baseFinalTime_s  = baseMotion.Polynomial.FinalTime_s;
if abs(knotTime_s(1) - baseStartTime_s) > clockTolerance_s || abs(knotTime_s(end) - baseFinalTime_s) > clockTolerance_s
    error("createOffsetSplineMotion:ClockMismatch", "The first and final offset knots must match the base motion clock.");
end

%% Section 2: Create And Compose The Minimum-Jerk Spline

lateral    = createMinimumJerkSpline(knotTime_s, knotOffset_units, knotVelocity_units_s);
break_s    = unique([baseMotion.Polynomial.SegmentStartTime_s; baseMotion.Polynomial.FinalTime_s; lateral.SegmentStartTime_s; lateral.FinalTime_s]);
polynomial = combinePolynomials(baseMotion.Polynomial, lateral, break_s, axisIndex);
candidate  = bmtpEngine.createMotionRecord(baseMotion, initialState, polynomial, [], sampleStep_s, seedSource);
end

%% Section 3: Local Functions

function polynomial = createMinimumJerkSpline(knotTime_s, knotPosition_units, knotVelocity_units_s)
    % Solve the quadratic minimum-jerk quintic Hermite interpolation system.
    segmentDuration_s = diff(knotTime_s);
    knotCount         = numel(knotTime_s);
    stateCount        = 3 * knotCount;
    hessian           = zeros(stateCount);
    coefficientMap    = zeros(6, 6, knotCount - 1);
    jerkMap           = [zeros(3), diag([6, 24, 60])];
    moment            = [1, 1 / 2, 1 / 3; 1 / 2, 1 / 3, 1 / 4; ...
        1 / 3, 1 / 4, 1 / 5];
    % Process each segment while assembling the complete motion or interval result.
    for segmentIndex = 1:knotCount - 1
        duration_s = segmentDuration_s(segmentIndex);
        coefficientMap(:, :, segmentIndex) = quinticHermiteMap(duration_s);
        localHessian = coefficientMap(:, :, segmentIndex).' * jerkMap.' * moment * jerkMap * coefficientMap(:, :, segmentIndex) / duration_s ^ 5;
        rows         = 3 * segmentIndex - 2:3 * segmentIndex + 3;
        hessian(rows, rows) = hessian(rows, rows) + localHessian;
    end
    knotState = zeros(stateCount, 1);
    knotState(1:3:end) = knotPosition_units;
    isFixed = mod((1:stateCount).' - 1, 3) == 0;
    isFixed([2, 3, stateCount - 1, stateCount]) = true;
    % A prescribed interior velocity can make a waypoint a true turning point.
    % Acceleration remains free, so adjacent pieces still join smoothly.
    velocityRows = 3 * find(isfinite(knotVelocity_units_s)) - 1;
    knotState(velocityRows) = knotVelocity_units_s(isfinite(knotVelocity_units_s));
    isFixed(velocityRows) = true;
    knotState(~isFixed) = -hessian(~isFixed, ~isFixed) \ (hessian(~isFixed, isFixed) * knotState(isFixed));
    positionPower_units = zeros(knotCount - 1, 1, 6);
    % Process each segment while assembling the complete motion or interval result.
    for segmentIndex = 1:knotCount - 1
        rows = 3 * segmentIndex - 2:3 * segmentIndex + 3;
        positionPower_units(segmentIndex, 1, :) = coefficientMap(:, :, segmentIndex) * knotState(rows);
    end
    polynomial = struct("SegmentCount", knotCount - 1, ...
        "SegmentStartTime_s", knotTime_s(1:end - 1), ...
        "SegmentDuration_s", segmentDuration_s, ...
        "FinalTime_s", knotTime_s(end), ...
        "positionPower_units", positionPower_units);
end

function map = quinticHermiteMap(duration_s)
    % Map endpoint position, velocity, and acceleration to quintic powers.
    h   = duration_s;
    map = [1, 0, 0, 0, 0, 0; 0, h, 0, 0, 0, 0; ...
        0, 0, h ^ 2 / 2, 0, 0, 0; ...
        -10, -6 * h, -1.5 * h ^ 2, 10, -4 * h, h ^ 2 / 2; ...
        15, 8 * h, 1.5 * h ^ 2, -15, 7 * h, -h ^ 2; ...
        -6, -3 * h, -h ^ 2 / 2, 6, -3 * h, h ^ 2 / 2];
end

function polynomial = combinePolynomials(direct, lateral, break_s, axisIndex)
    % Split at both sets of breakpoints, then add the offset polynomial.
    duration_s        = diff(break_s);
    segmentCount      = numel(duration_s);
    dimensionCount    = size(direct.positionPower_units, 2);
    coefficientCount  = max(size(direct.positionPower_units, 3), size(lateral.positionPower_units, 3));
    positionPower_units = zeros(segmentCount, dimensionCount, coefficientCount);
    % Process each segment while assembling the complete motion or interval result.
    for segmentIndex = 1:segmentCount
        startTime_s  = break_s(segmentIndex);
        step_s       = duration_s(segmentIndex);
        directPower  = translatePolynomial(direct, startTime_s, step_s, coefficientCount);
        lateralPower = translatePolynomial(lateral, startTime_s, step_s, coefficientCount);
        directPower(axisIndex, :) = directPower(axisIndex, :) + lateralPower;
        positionPower_units(segmentIndex, :, :) = reshape(directPower, 1, dimensionCount, coefficientCount);
    end
    durationScale_s          = reshape(duration_s, [], 1, 1);
    velocityPower_units_s      = positionPower_units(:, :, 2:end) .* reshape(1:coefficientCount - 1, 1, 1, []) ./ durationScale_s;
    accelerationPower_units_s2 = velocityPower_units_s(:, :, 2:end) .* reshape(1:coefficientCount - 2, 1, 1, []) ./ durationScale_s;
    jerkPower_units_s3         = accelerationPower_units_s2(:, :, 2:end) .* reshape(1:coefficientCount - 3, 1, 1, []) ./ durationScale_s;
    polynomial               = struct("Degree", coefficientCount - 1, ...
        "SegmentCount", segmentCount, ...
        "SegmentStartTime_s", break_s(1:end - 1), ...
        "SegmentDuration_s", duration_s, ...
        "SegmentBreakTau", (break_s - break_s(1)) / ...
            (break_s(end) - break_s(1)), ...
        "FinalTime_s", break_s(end), ...
        "positionPower_units", positionPower_units, ...
        "velocityPower_units_s", velocityPower_units_s, ...
        "accelerationPower_units_s2", accelerationPower_units_s2, ...
        "jerkPower_units_s3", jerkPower_units_s3, ...
        "TerminalState", direct.TerminalState);
end

function power = translatePolynomial(polynomial, startTime_s, duration_s, outputCount)
    % Re-express a segment on a normalized subinterval.
    sourceIndex      = min(polynomial.SegmentCount, 1 + sum(startTime_s >= polynomial.SegmentStartTime_s(2:end)));
    sourceDuration_s = polynomial.SegmentDuration_s(sourceIndex);
    sourceTau        = (startTime_s - polynomial.SegmentStartTime_s(sourceIndex)) / sourceDuration_s;
    durationRatio    = duration_s / sourceDuration_s;
    source           = reshape(polynomial.positionPower_units(sourceIndex, :, :), size(polynomial.positionPower_units, 2), []);
    power            = zeros(size(source, 1), outputCount);
    % Process each target power needed to complete translate polynomial.
    for targetPower = 0:size(source, 2) - 1
        % Process each source power needed to complete translate polynomial.
        for sourcePower = targetPower:size(source, 2) - 1
            power(:, targetPower + 1) = power(:, targetPower + 1) + source(:, sourcePower + 1) * nchoosek(sourcePower, targetPower) * sourceTau ^ (sourcePower - targetPower) * durationRatio ^ targetPower;
        end
    end
end
