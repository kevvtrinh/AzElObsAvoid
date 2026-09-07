function [bounds, dynamics, checks] = validatePolynomialTrajectory(polynomial, time_s, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3, initialState, goalState, limits, options, tolerance, rangeCheck)
%% Section 0: Header & Readme
% SYNTAX
%   [bounds, dynamics, checks] = ...
%       obstacleAvoidance.validation.validatePolynomialTrajectory( ...
%       polynomial, time_s, position_units, velocity_units_s, ...
%       acceleration_units_s2, jerk_units_s3, initialState, goalState, limits, ...
%       options, tolerance, rangeCheck)
%
% PURPOSE
%   - Validate a degree-neutral piecewise polynomial independently.
%   - Check its time base, derivative chain, knots, endpoints, sampled
%     histories, and continuous physical bounds.
%
% INPUTS
%   - polynomial (scalar planner polynomial struct)
%       Ascending-power position coefficients have degree at least three;
%       each derivative array has one fewer coefficient.
%   - time_s and sampled histories (numeric arrays)
%       Returned absolute times and N-by-2 position through jerk histories.
%   - initialState, goalState, limits, options (scalar structs)
%       Resolved inputs governing endpoints, bounds, and x wrapping.
%   - tolerance (nonnegative finite scalar)
%       Absolute state and polynomial consistency tolerance.
%   - rangeCheck (function handle)
%       Complete polynomial range certificate for normalized time [0, 1].
%
% OUTPUTS
%   - bounds, dynamics, checks (scalar structs)
%       Stable continuous-bound, derivative, format, and history checks.
%
% UNITS
%   - Position is coordinate units. Derivatives use units/s, units/s^2, and units/s^3.
%     Time is seconds.
%

%% Section 1: Validate The Polynomial Representation

checks   = createEmptyChecks();
bounds   = createBounds(false(1, 4));
dynamics = struct();
dynamics.Consistent      = false;
dynamics.MaximumResidual = Inf;
requiredFields = {'SegmentCount', 'SegmentStartTime_s', ...
    'SegmentDuration_s', 'FinalTime_s', 'positionPower_units', ...
    'velocityPower_units_s', 'accelerationPower_units_s2', ...
    'jerkPower_units_s3'};
if ~isstruct(polynomial) || ~isscalar(polynomial) || ~all(isfield(polynomial, requiredFields))
    return;
end
segmentCount       = polynomial.SegmentCount;
segmentDuration_s  = double(polynomial.SegmentDuration_s(:));
segmentStartTime_s = double(polynomial.SegmentStartTime_s(:));
countIsValid       = isnumeric(segmentCount) && isscalar(segmentCount) && isfinite(segmentCount) && segmentCount >= 1 && segmentCount == fix(segmentCount);
durationIsValid    = isnumeric(polynomial.SegmentDuration_s) && any(numel(segmentDuration_s) == [1 segmentCount]) && all(isfinite(segmentDuration_s)) && all(segmentDuration_s > 0);
finalTimeIsValid   = isnumeric(polynomial.FinalTime_s) && isscalar(polynomial.FinalTime_s) && isfinite(polynomial.FinalTime_s);
if ~(countIsValid && durationIsValid && finalTimeIsValid)
    return;
end
if isscalar(segmentDuration_s)
    segmentDuration_s = repmat(segmentDuration_s, segmentCount, 1);
end
powerArrays = {polynomial.positionPower_units, ...
    polynomial.velocityPower_units_s, ...
    polynomial.accelerationPower_units_s2, polynomial.jerkPower_units_s3};
positionCoefficientCount = size(powerArrays{1}, 3);
arraysAreValid           = numel(segmentStartTime_s) == segmentCount && all(isfinite(segmentStartTime_s));
% Process each derivative order needed to verify polynomial trajectory.
for derivativeOrder = 0:3
    array          = powerArrays{derivativeOrder + 1};
    arraysAreValid = arraysAreValid && isnumeric(array) && positionCoefficientCount >= 4 && size(array, 1) == segmentCount && size(array, 2) == 2 && size(array, 3) == positionCoefficientCount - derivativeOrder && all(isfinite(array), "all");
end
checks.FormatValid = arraysAreValid;
if ~checks.FormatValid
    return;
end

%% Section 2: Check Time, Bounds, And The Derivative Chain

expectedStartTime_s    = segmentStartTime_s(1) + [0; cumsum(segmentDuration_s(1:end - 1))];
expectedFinalTime_s    = segmentStartTime_s(1) + sum(segmentDuration_s);
timeTolerance_s        = max(tolerance, 100 * eps(max(1, max(abs([expectedStartTime_s; expectedFinalTime_s])))));
sampleTimeIsConsistent = ~isempty(time_s) && all(isfinite(time_s)) && abs(time_s(1) - segmentStartTime_s(1)) <= timeTolerance_s && abs(time_s(end) - expectedFinalTime_s) <= timeTolerance_s && all(time_s >= segmentStartTime_s(1) - timeTolerance_s) && all(time_s <= expectedFinalTime_s + timeTolerance_s);
checks.TimeBaseConsistent = sampleTimeIsConsistent && max(abs(segmentStartTime_s - expectedStartTime_s)) <= timeTolerance_s && abs(polynomial.FinalTime_s - expectedFinalTime_s) <= timeTolerance_s;
checks.InitialTimeMatched = abs(segmentStartTime_s(1) - initialState.time_s) <= timeTolerance_s;
lowerLimits = {[limits.xInterval_units(1), ...
    limits.yInterval_units(1)], -limits.maxVelocity_units_s, -limits.maxAcceleration_units_s2, -limits.maxJerk_units_s3};
upperLimits = {[limits.xInterval_units(2), ...
    limits.yInterval_units(2)], limits.maxVelocity_units_s, limits.maxAcceleration_units_s2, limits.maxJerk_units_s3};
within = true(1, 4);
wrapAxes = [options.WrapX, options.WrapY];
% Process each segment while assembling the complete motion or interval result.
for segmentIndex = 1:segmentCount
    % Evaluate each coordinate axis and combine its limiting result.
    for axisIndex = 1:2
        % Process each derivative order needed to verify polynomial trajectory.
        for derivativeOrder = 0:3
            shouldCheck = derivativeOrder > 0 || ~wrapAxes(axisIndex);
            if shouldCheck && within(derivativeOrder + 1)
                power = reshape(powerArrays{derivativeOrder + 1}(segmentIndex, axisIndex, :), [], 1);
                within(derivativeOrder + 1) = rangeCheck(power, lowerLimits{derivativeOrder + 1}(axisIndex), upperLimits{derivativeOrder + 1}(axisIndex), tolerance);
            end
        end
    end
end
bounds           = createBounds(within);
durationScale_s  = reshape(segmentDuration_s, [], 1, 1);
dynamicsResidual = zeros(0, 1);
% Process each derivative order needed to verify polynomial trajectory.
for derivativeOrder = 0:2
    source           = powerArrays{derivativeOrder + 1};
    derivative       = source(:, :, 2:end) .* reshape(1:size(source, 3) - 1, 1, 1, []) ./ durationScale_s;
    dynamicsResidual = [dynamicsResidual; ...
        derivative(:) - powerArrays{derivativeOrder + 2}(:)]; %#ok<AGROW>
end
maximumDynamicsResidual = max(abs(dynamicsResidual));
dynamics                = struct("Consistent", maximumDynamicsResidual <= tolerance, ...
    "MaximumResidual", maximumDynamicsResidual);

%% Section 3: Check Knots, Endpoints, And Sampled Histories

maximumSegmentContinuityResidual = 0;
if segmentCount > 1
    previousSegmentIndex = (1:segmentCount - 1).';
    previousEndTime_s    = segmentStartTime_s(1:end - 1) + segmentDuration_s(1:end - 1);
    [~, previousPosition_units, previousVelocity_units_s, ...
        previousAcceleration_units_s2] = bmtpEngine.evaluatePolynomial(polynomial, previousEndTime_s, previousSegmentIndex);
    nextInitialState = [reshape(powerArrays{1}(2:end, :, 1), [], 2), ...
        reshape(powerArrays{2}(2:end, :, 1), [], 2), reshape(powerArrays{3}(2:end, :, 1), [], 2)];
    previousFinalState = [previousPosition_units, previousVelocity_units_s, ...
        previousAcceleration_units_s2];
    maximumSegmentContinuityResidual = max(abs(previousFinalState - nextInitialState), [], "all");
end
checks.MaximumSegmentContinuityResidual = maximumSegmentContinuityResidual;
checks.SegmentContinuity                = maximumSegmentContinuityResidual <= tolerance;
initialPolynomialState = [reshape(powerArrays{1}(1, :, 1), 1, 2), ...
    reshape(powerArrays{2}(1, :, 1), 1, 2), reshape(powerArrays{3}(1, :, 1), 1, 2)];
terminalPolynomialState = [sum(reshape(powerArrays{1}(end, :, :), 2, []), 2).', ...
    sum(reshape(powerArrays{2}(end, :, :), 2, []), 2).', sum(reshape(powerArrays{3}(end, :, :), 2, []), 2).'];
goalPosition_units      = obstacleAvoidance.input.goalPositionAtTime(goalState, polynomial.FinalTime_s);
expectedEndpointState = [initialState.position_units, ...
    initialState.velocity_units_s, initialState.acceleration_units_s2, ...
    goalPosition_units, goalState.velocity_units_s, goalState.acceleration_units_s2];
checks.EndpointStatesMatched = max(abs([initialPolynomialState, terminalPolynomialState] - expectedEndpointState)) <= tolerance;
[~, polynomialPosition_units, polynomialVelocity_units_s, ...
    polynomialAcceleration_units_s2, polynomialJerk_units_s3] = bmtpEngine.evaluatePolynomial(polynomial, time_s);
historySizesMatch = isequal(size(position_units), size(polynomialPosition_units)) && isequal(size(velocity_units_s), size(polynomialVelocity_units_s)) && isequal(size(acceleration_units_s2), size(polynomialAcceleration_units_s2)) && isequal(size(jerk_units_s3), size(polynomialJerk_units_s3));
if historySizesMatch
    checks.MaximumHistoryResidual = max(abs([ position_units - polynomialPosition_units, velocity_units_s - polynomialVelocity_units_s, acceleration_units_s2 - polynomialAcceleration_units_s2, jerk_units_s3 - polynomialJerk_units_s3]), [], "all");
    checks.HistoryConsistent      = checks.MaximumHistoryResidual <= tolerance;
end
checks.Valid = checks.FormatValid && checks.InitialTimeMatched && checks.TimeBaseConsistent && checks.SegmentContinuity && checks.EndpointStatesMatched && checks.HistoryConsistent;
end

%% Section 4: Local Functions

function checks = createEmptyChecks()
    % Initialize checks for invalid polynomial inputs.
    checks = struct();
    checks.Valid                            = false;
    checks.FormatValid                      = false;
    checks.InitialTimeMatched               = false;
    checks.TimeBaseConsistent               = false;
    checks.SegmentContinuity                = false;
    checks.EndpointStatesMatched            = false;
    checks.HistoryConsistent                = false;
    checks.MaximumSegmentContinuityResidual = Inf;
    checks.MaximumHistoryResidual           = Inf;
end

function bounds = createBounds(within)
    % Assemble continuous checks in public field order.
    bounds = struct("Valid", all(within), ...
        "PositionWithinLimits", within(1), ...
        "VelocityWithinLimits", within(2), ...
        "AccelerationWithinLimits", within(3), ...
        "JerkWithinLimits", within(4));
end
