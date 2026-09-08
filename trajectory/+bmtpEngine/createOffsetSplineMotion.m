function candidate = createOffsetSplineMotion(baseMotion, knotTime_s, knotOffset_units, axisIndex, initialState, sampleStep_s, seedSource, knotVelocity_units_s)
%% Section 0: Header & Readme
% SYNTAX: candidate = bmtpEngine.createOffsetSplineMotion(baseMotion, knotTime_s, knotOffset_units, axisIndex, initialState, sampleStep_s, seedSource, knotVelocity_units_s)
% PURPOSE: Add a minimum-integrated-jerk quintic offset to one coordinate without changing duration.
% INPUTS: baseMotion requires a complete Polynomial. Increasing absolute knotTime_s
%   and knotOffset_units are matching columns; axisIndex selects the offset coordinate.
%   initialState contains time, position, velocity, and acceleration; sampleStep_s is positive.
%   seedSource labels the construction. Optional knotVelocity_units_s matches the knots:
%   NaN leaves an interior velocity free; omitted/empty leaves all interior velocities free.
%   Endpoint offset velocities remain zero.
% OUTPUTS: candidate motion record with composite polynomial and histories.
% UNITS: Coordinate units, seconds, physical derivatives; histories N-by-D.

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
    coefficientCount  = max(size(direct.positionPower_units, 3), size(lateral.positionPower_units, 3));
    positionPower_units = translatePolynomial(direct, break_s(1:end - 1), duration_s, coefficientCount);
    lateralPower_units = translatePolynomial(lateral, break_s(1:end - 1), duration_s, coefficientCount);
    positionPower_units(:, axisIndex, :) = positionPower_units(:, axisIndex, :) + lateralPower_units;
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
    % Translate every requested subinterval together. The power sums retain
    % their scalar order; only independent output spans are batched.
    sourceIndex      = min(polynomial.SegmentCount, 1 + sum(startTime_s >= polynomial.SegmentStartTime_s(2:end).', 2));
    sourceDuration_s = polynomial.SegmentDuration_s(sourceIndex);
    sourceTau        = (startTime_s - polynomial.SegmentStartTime_s(sourceIndex)) ./ sourceDuration_s;
    durationRatio    = duration_s ./ sourceDuration_s;
    source           = polynomial.positionPower_units(sourceIndex, :, :);
    power            = zeros(numel(startTime_s), size(source, 2), outputCount);
    for targetPower = 0:size(source, 3) - 1
        for sourcePower = targetPower:size(source, 3) - 1
            power(:, :, targetPower + 1) = power(:, :, targetPower + 1) + source(:, :, sourcePower + 1) * nchoosek(sourcePower, targetPower) .* sourceTau .^ (sourcePower - targetPower) .* durationRatio .^ targetPower;
        end
    end
end
