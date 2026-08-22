function motion = buildQuinticBsplinePrototype( ...
        route_deg, initialState, goalState, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = buildQuinticBsplinePrototype()
%   motion = buildQuinticBsplinePrototype( ...
%       route_deg, initialState, goalState, limits)
%   motion = buildQuinticBsplinePrototype( ...
%       route_deg, initialState, goalState, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Construct and independently validate one research quintic B-spline
%     motion using the maintained polynomial trajectory contract.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 numeric array, N >= 2)
%       Ordered [azimuth elevation] geometric route with distinct neighbors.
%   - initialState (scalar struct)
%       time_s and position_deg are required. This prototype supports only
%       zero velocity_deg_s and acceleration_deg_s2 endpoint states.
%   - goalState (scalar struct)
%       time_s is the latest arrival and position_deg is fixed. This
%       prototype supports only zero terminal velocity and acceleration.
%   - limits (scalar struct)
%       Positive maxVelocity_deg_s, maxAcceleration_deg_s2, and
%       maxJerk_deg_s3 are required. Workspace intervals are optional.
%   - optionOverrides (scalar struct, optional; default struct())
%       .ControlPointOffsets_deg is (N-2)-by-2 (default zeros).
%       .SpanWeights is an (N-1)-vector of positive relative durations
%       (default ones). .SampleTime_s is positive (default 0.05 seconds).
%**************************************************************************
% OUTPUTS
%   - motion (scalar struct)
%       Stable prototype result with B-spline parameters, exact quintic
%       span polynomials, sampled histories, continuity, and Validation.
%       Invalid contracts throw; an unvalidated motion returns Success=false.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3. Histories are N-by-2 [azimuth elevation].
%**************************************************************************

%% Section 1: Validate Inputs And Resolve Prototype Controls

defaults = struct( ...
    "ControlPointOffsets_deg", zeros(0, 2), ...
    "SpanWeights", zeros(0, 1), ...
    "SampleTime_s", 0.05);
if nargin == 0
    motion = defaults;
    return;
end
if nargin < 4
    error("buildQuinticBsplinePrototype:MissingInputs", ...
        "route_deg, initialState, goalState, and limits are required.");
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
[options, unknownNames] = azElInternal.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("buildQuinticBsplinePrototype:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
validateattributes(route_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2});
route_deg = double(route_deg);
if size(route_deg, 1) < 2 || any(vecnorm(diff(route_deg), 2, 2) <= 0)
    error("buildQuinticBsplinePrototype:InvalidRoute", ...
        "route_deg must have at least two rows and distinct neighbors.");
end
initialState = normalizeState(initialState, "initialState");
goalState = normalizeState(goalState, "goalState");
limits = normalizeLimits(limits);
if goalState.time_s <= initialState.time_s
    error("buildQuinticBsplinePrototype:InvalidTimeWindow", ...
        "goalState.time_s must be greater than initialState.time_s.");
end
endpointTolerance = 1e-10;
endpointDerivatives = [initialState.velocity_deg_s, ...
    initialState.acceleration_deg_s2, goalState.velocity_deg_s, ...
    goalState.acceleration_deg_s2];
if max(abs(endpointDerivatives)) > endpointTolerance
    error("buildQuinticBsplinePrototype:UnsupportedEndpointState", ...
        "The prototype currently requires zero endpoint velocity and " + ...
        "acceleration; the largest requested magnitude is %.6g.", ...
        max(abs(endpointDerivatives)));
end
if max(abs(route_deg(1, :) - initialState.position_deg)) > ...
        endpointTolerance || ...
        max(abs(route_deg(end, :) - goalState.position_deg)) > ...
        endpointTolerance
    error("buildQuinticBsplinePrototype:RouteEndpointMismatch", ...
        "The first and final route rows must match the requested positions.");
end
interiorCount = size(route_deg, 1) - 2;
if isempty(options.ControlPointOffsets_deg)
    options.ControlPointOffsets_deg = zeros(interiorCount, 2);
end
validateattributes(options.ControlPointOffsets_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2});
if size(options.ControlPointOffsets_deg, 1) ~= interiorCount
    error("buildQuinticBsplinePrototype:OffsetCountMismatch", ...
        "ControlPointOffsets_deg must have %d rows; observed %d.", ...
        interiorCount, size(options.ControlPointOffsets_deg, 1));
end
spanCount = size(route_deg, 1) - 1;
if isempty(options.SpanWeights)
    options.SpanWeights = ones(spanCount, 1);
end
validateattributes(options.SpanWeights, {'numeric'}, ...
    {'real', 'finite', 'vector', 'positive'});
options.SpanWeights = double(options.SpanWeights(:));
if numel(options.SpanWeights) ~= spanCount
    error("buildQuinticBsplinePrototype:SpanWeightCountMismatch", ...
        "SpanWeights must contain %d values; observed %d.", ...
        spanCount, numel(options.SpanWeights));
end
validateattributes(options.SampleTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});

%% Section 2: Assemble The Open Quintic B-Spline

degree = 5;
controlPoint_deg = [ ...
    repmat(route_deg(1, :), 3, 1); ...
    route_deg(2:end - 1, :) + options.ControlPointOffsets_deg; ...
    repmat(route_deg(end, :), 3, 1)];
baseSpanDuration_s = options.SpanWeights / mean(options.SpanWeights);
basePolynomial = convertToPolynomial( ...
    controlPoint_deg, degree, initialState.time_s, baseSpanDuration_s);

%% Section 3: Apply A Deterministic Continuous Kinematic Time Scale

[peakVelocity_deg_s, peakAcceleration_deg_s2, peakJerk_deg_s3] = ...
    continuousDerivativePeaks(basePolynomial);
velocityScale = max( ...
    peakVelocity_deg_s ./ limits.maxVelocity_deg_s);
accelerationScale = sqrt(max( ...
    peakAcceleration_deg_s2 ./ limits.maxAcceleration_deg_s2));
jerkScale = nthroot(max( ...
    peakJerk_deg_s3 ./ limits.maxJerk_deg_s3), 3);
minimumSpanDuration_s = 1e-3;
minimumDurationScale = minimumSpanDuration_s / ...
    min(baseSpanDuration_s);
durationScale = max([velocityScale, accelerationScale, ...
    jerkScale, minimumDurationScale]);
% The small factor prevents equality-roundoff from appearing as a violation.
durationRoundoffScale = 1 + 64 * eps;
spanDuration_s = baseSpanDuration_s * ...
    durationScale * durationRoundoffScale;
polynomial = convertToPolynomial( ...
    controlPoint_deg, degree, initialState.time_s, spanDuration_s);

%% Section 4: Sample And Independently Validate The Motion

[time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
    jerk_deg_s3] = samplePolynomial(polynomial, options.SampleTime_s);
continuity = continuityDiagnostics(polynomial);
motion = emptyMotion(options);
motion.MotionSource = "quinticBsplinePrototype";
motion.FinalTime_s = polynomial.FinalTime_s;
motion.MotionDuration_s = polynomial.FinalTime_s - initialState.time_s;
motion.IntegratedSquaredJerk_deg2_s5 = ...
    integratedSquaredJerk(polynomial);
motion.time_s = time_s;
motion.position_deg = position_deg;
motion.velocity_deg_s = velocity_deg_s;
motion.acceleration_deg_s2 = acceleration_deg_s2;
motion.jerk_deg_s3 = jerk_deg_s3;
motion.Polynomial = polynomial;
motion.Route_deg = route_deg;
motion.ControlPoint_deg = controlPoint_deg;
motion.KnotTime_s = knotTime(initialState.time_s, spanDuration_s, degree);
motion.SpanDuration_s = spanDuration_s;
motion.Continuity = continuity;
relativeTimingParameterCount = max(0, spanCount - 1);
motion.RepresentationDiagnostics = struct( ...
    "Degree", degree, ...
    "SpanCount", spanCount, ...
    "ControlPointOffsetParameterCount", 2 * interiorCount, ...
    "RelativeTimingParameterCount", relativeTimingParameterCount, ...
    "TotalParameterCount", ...
    2 * interiorCount + relativeTimingParameterCount, ...
    "InteriorRouteInterpolated", false, ...
    "MaintainedValidatorCompatible", true);
validatorOptions = planAzElMotion();
validatorOptions.GoalTimeMode = "earliestArrival";
validatorOptions.SampleTime_s = options.SampleTime_s;
validation = validateAzElTrajectory( ...
    motion, [], initialState, goalState, limits, validatorOptions);
motion.Validation = validation;
motion.Success = validation.Passed && continuity.C3Continuous;
if motion.Success
    motion.Message = ...
        "The quintic B-spline prototype passed independent validation.";
    motion.TerminationReason = "prototypeValidated";
else
    motion.Message = "The quintic B-spline prototype failed: " + ...
        validation.Message;
    motion.TerminationReason = "prototypeValidationFailed";
end
end

%% Section 5: Local Functions

function state = normalizeState(state, inputName)
%% Section 0: Header & Readme
% SYNTAX
%   state = normalizeState(state, inputName)
%**************************************************************************
% PURPOSE
%   - Normalize one fixed-position prototype endpoint state.
%**************************************************************************
% INPUTS
%   - state (scalar struct)
%   - inputName (scalar text)
%**************************************************************************
% OUTPUTS
%   - state (normalized scalar struct)
%**************************************************************************
% UNITS
%   - Position is degrees; derivatives use seconds.
%**************************************************************************
if ~isstruct(state) || ~isscalar(state) || ...
        ~all(isfield(state, ["time_s", "position_deg"]))
    error("buildQuinticBsplinePrototype:InvalidState", ...
        "%s must be scalar with time_s and position_deg.", inputName);
end
validateattributes(state.time_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(state.position_deg, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2});
state.time_s = double(state.time_s);
state.position_deg = reshape(double(state.position_deg), 1, 2);
derivativeNames = ["velocity_deg_s", "acceleration_deg_s2"];
for derivativeName = derivativeNames
    if ~isfield(state, derivativeName) || isempty(state.(derivativeName))
        state.(derivativeName) = [0 0];
    end
    validateattributes(state.(derivativeName), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2});
    state.(derivativeName) = ...
        reshape(double(state.(derivativeName)), 1, 2);
end
end

function limits = normalizeLimits(limits)
%% Section 0: Header & Readme
% SYNTAX
%   limits = normalizeLimits(limits)
%**************************************************************************
% PURPOSE
%   - Normalize physical and workspace limits used by the prototype.
%**************************************************************************
% INPUTS
%   - limits (scalar struct)
%**************************************************************************
% OUTPUTS
%   - limits (normalized scalar struct)
%**************************************************************************
% UNITS
%   - Workspace is degrees; derivatives use seconds.
%**************************************************************************
requiredNames = ["maxVelocity_deg_s", "maxAcceleration_deg_s2", ...
    "maxJerk_deg_s3"];
if ~isstruct(limits) || ~isscalar(limits) || ...
        ~all(isfield(limits, requiredNames))
    error("buildQuinticBsplinePrototype:InvalidLimits", ...
        "limits must contain positive two-axis velocity, acceleration, " + ...
        "and jerk fields.");
end
for limitName = requiredNames
    validateattributes(limits.(limitName), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2, 'positive'});
    limits.(limitName) = reshape(double(limits.(limitName)), 1, 2);
end
if ~isfield(limits, "azimuthInterval_deg") || ...
        isempty(limits.azimuthInterval_deg)
    limits.azimuthInterval_deg = [-180 180];
end
if ~isfield(limits, "elevationInterval_deg") || ...
        isempty(limits.elevationInterval_deg)
    limits.elevationInterval_deg = [-90 90];
end
intervalNames = ["azimuthInterval_deg", "elevationInterval_deg"];
for intervalName = intervalNames
    validateattributes(limits.(intervalName), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2, 'increasing'});
    limits.(intervalName) = ...
        reshape(double(limits.(intervalName)), 1, 2);
end
end

function polynomial = convertToPolynomial( ...
        controlPoint_deg, degree, initialTime_s, spanDuration_s)
%% Section 0: Header & Readme
% SYNTAX
%   polynomial = convertToPolynomial( ...
%       controlPoint_deg, degree, initialTime_s, spanDuration_s)
%**************************************************************************
% PURPOSE
%   - Convert every B-spline knot span to the maintained quintic schema.
%**************************************************************************
% INPUTS
%   - controlPoint_deg (M-by-2 numeric array)
%   - degree (integer scalar; required value 5)
%   - initialTime_s (finite scalar)
%   - spanDuration_s ((M-degree)-vector of positive durations)
%**************************************************************************
% OUTPUTS
%   - polynomial (scalar maintained polynomial struct)
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use seconds.
%**************************************************************************
spanDuration_s = double(spanDuration_s(:));
spanCount = numel(spanDuration_s);
segmentStartTime_s = initialTime_s + ...
    [0; cumsum(spanDuration_s(1:end - 1))];
knots_s = knotTime(initialTime_s, spanDuration_s, degree);
localSamples = linspace(0, 1, degree + 1).';
vandermonde = localSamples.^(0:degree);
positionPower_deg = zeros(spanCount, 2, degree + 1);
velocityPower_deg_s = zeros(spanCount, 2, degree);
accelerationPower_deg_s2 = zeros(spanCount, 2, degree - 1);
jerkPower_deg_s3 = zeros(spanCount, 2, degree - 2);
for segmentIndex = 1:spanCount
    sampleTime_s = segmentStartTime_s(segmentIndex) + ...
        localSamples * spanDuration_s(segmentIndex);
    basisValues = zeros(degree + 1, size(controlPoint_deg, 1));
    for sampleIndex = 1:degree + 1
        basisValues(sampleIndex, :) = bsplineBasis( ...
            knots_s, degree, sampleTime_s(sampleIndex), ...
            size(controlPoint_deg, 1));
    end
    positionPower = vandermonde \ (basisValues * controlPoint_deg);
    positionPower = positionPower.';
    velocityPower = positionPower(:, 2:end) .* (1:degree) / ...
        spanDuration_s(segmentIndex);
    accelerationPower = velocityPower(:, 2:end) .* (1:degree - 1) / ...
        spanDuration_s(segmentIndex);
    jerkPower = accelerationPower(:, 2:end) .* (1:degree - 2) / ...
        spanDuration_s(segmentIndex);
    positionPower_deg(segmentIndex, :, :) = ...
        reshape(positionPower, 1, 2, degree + 1);
    velocityPower_deg_s(segmentIndex, :, :) = ...
        reshape(velocityPower, 1, 2, degree);
    accelerationPower_deg_s2(segmentIndex, :, :) = ...
        reshape(accelerationPower, 1, 2, degree - 1);
    jerkPower_deg_s3(segmentIndex, :, :) = ...
        reshape(jerkPower, 1, 2, degree - 2);
end
terminalPosition_deg = sum(reshape( ...
    positionPower_deg(end, :, :), 2, []), 2).';
terminalVelocity_deg_s = sum(reshape( ...
    velocityPower_deg_s(end, :, :), 2, []), 2).';
terminalAcceleration_deg_s2 = sum(reshape( ...
    accelerationPower_deg_s2(end, :, :), 2, []), 2).';
polynomial = struct( ...
    "SegmentCount", spanCount, ...
    "SegmentStartTime_s", segmentStartTime_s, ...
    "SegmentDuration_s", spanDuration_s, ...
    "FinalTime_s", initialTime_s + sum(spanDuration_s), ...
    "positionPower_deg", positionPower_deg, ...
    "velocityPower_deg_s", velocityPower_deg_s, ...
    "accelerationPower_deg_s2", accelerationPower_deg_s2, ...
    "jerkPower_deg_s3", jerkPower_deg_s3, ...
    "TerminalState", struct( ...
    "position_deg", terminalPosition_deg, ...
    "velocity_deg_s", terminalVelocity_deg_s, ...
    "acceleration_deg_s2", terminalAcceleration_deg_s2));
end

function knots_s = knotTime(initialTime_s, spanDuration_s, degree)
%% Section 0: Header & Readme
% SYNTAX
%   knots_s = knotTime(initialTime_s, spanDuration_s, degree)
%**************************************************************************
% PURPOSE
%   - Build one open knot vector with simple interior physical-time knots.
%**************************************************************************
% INPUTS
%   - initialTime_s (finite scalar)
%   - spanDuration_s (positive vector)
%   - degree (positive integer scalar)
%**************************************************************************
% OUTPUTS
%   - knots_s (row vector)
%**************************************************************************
% UNITS
%   - Knot values are seconds.
%**************************************************************************
spanBoundary_s = initialTime_s + [0; cumsum(spanDuration_s(:))];
knots_s = [ ...
    repmat(spanBoundary_s(1), 1, degree + 1), ...
    spanBoundary_s(2:end - 1).', ...
    repmat(spanBoundary_s(end), 1, degree + 1)];
end

function basis = bsplineBasis(knots_s, degree, time_s, controlCount)
%% Section 0: Header & Readme
% SYNTAX
%   basis = bsplineBasis(knots_s, degree, time_s, controlCount)
%**************************************************************************
% PURPOSE
%   - Evaluate all nonzero-safe Cox-de Boor basis values at one time.
%**************************************************************************
% INPUTS
%   - knots_s (nondecreasing row vector)
%   - degree (nonnegative integer scalar)
%   - time_s (finite scalar)
%   - controlCount (positive integer scalar)
%**************************************************************************
% OUTPUTS
%   - basis (1-by-controlCount numeric row)
%**************************************************************************
% UNITS
%   - Basis values are dimensionless; knots and time are seconds.
%**************************************************************************
endpointTolerance_s = 32 * eps(max(1, abs(knots_s(end))));
if time_s >= knots_s(end) - endpointTolerance_s
    basis = zeros(1, controlCount);
    basis(end) = 1;
    return;
end
baseCount = numel(knots_s) - 1;
basis = zeros(1, baseCount);
for basisIndex = 1:baseCount
    basis(basisIndex) = time_s >= knots_s(basisIndex) && ...
        time_s < knots_s(basisIndex + 1);
end
for recursionDegree = 1:degree
    nextCount = baseCount - recursionDegree;
    nextBasis = zeros(1, nextCount);
    for basisIndex = 1:nextCount
        leftDenominator_s = knots_s(basisIndex + recursionDegree) - ...
            knots_s(basisIndex);
        rightDenominator_s = ...
            knots_s(basisIndex + recursionDegree + 1) - ...
            knots_s(basisIndex + 1);
        leftValue = 0;
        rightValue = 0;
        if leftDenominator_s > 0
            leftValue = (time_s - knots_s(basisIndex)) / ...
                leftDenominator_s * basis(basisIndex);
        end
        if rightDenominator_s > 0
            rightValue = ...
                (knots_s(basisIndex + recursionDegree + 1) - time_s) / ...
                rightDenominator_s * basis(basisIndex + 1);
        end
        nextBasis(basisIndex) = leftValue + rightValue;
    end
    basis = nextBasis;
end
basis = basis(1:controlCount);
end

function [peakVelocity_deg_s, peakAcceleration_deg_s2, ...
        peakJerk_deg_s3] = continuousDerivativePeaks(polynomial)
%% Section 0: Header & Readme
% SYNTAX
%   [peakVelocity_deg_s, peakAcceleration_deg_s2, ...
%       peakJerk_deg_s3] = continuousDerivativePeaks(polynomial)
%**************************************************************************
% PURPOSE
%   - Bound complete-span derivative peaks through Bernstein convex hulls.
%**************************************************************************
% INPUTS
%   - polynomial (scalar maintained polynomial struct)
%**************************************************************************
% OUTPUTS
%   - peakVelocity_deg_s, peakAcceleration_deg_s2, peakJerk_deg_s3
%       1-by-2 conservative absolute derivative peaks.
%**************************************************************************
% UNITS
%   - Derivatives use degrees and seconds.
%**************************************************************************
peakVelocity_deg_s = derivativePeak( ...
    polynomial.velocityPower_deg_s);
peakAcceleration_deg_s2 = derivativePeak( ...
    polynomial.accelerationPower_deg_s2);
peakJerk_deg_s3 = derivativePeak(polynomial.jerkPower_deg_s3);
end

function peak = derivativePeak(powerArray)
%% Section 0: Header & Readme
% SYNTAX
%   peak = derivativePeak(powerArray)
%**************************************************************************
% PURPOSE
%   - Bound one two-axis piecewise polynomial over every normalized span.
%**************************************************************************
% INPUTS
%   - powerArray (segment-by-2-by-coefficient numeric array)
%**************************************************************************
% OUTPUTS
%   - peak (1-by-2 nonnegative row)
%**************************************************************************
% UNITS
%   - Units are inherited from powerArray.
%**************************************************************************
peak = [0 0];
for segmentIndex = 1:size(powerArray, 1)
    for axisIndex = 1:2
        powerCoefficient = reshape( ...
            powerArray(segmentIndex, axisIndex, :), [], 1);
        bernsteinCoefficient = ...
            azElInternal.powerToBernstein(powerCoefficient);
        peak(axisIndex) = max(peak(axisIndex), ...
            max(abs(bernsteinCoefficient)));
    end
end
end

function [time_s, position_deg, velocity_deg_s, ...
        acceleration_deg_s2, jerk_deg_s3] = ...
        samplePolynomial(polynomial, sampleTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [time_s, position_deg, velocity_deg_s, ...
%       acceleration_deg_s2, jerk_deg_s3] = ...
%       samplePolynomial(polynomial, sampleTime_s)
%**************************************************************************
% PURPOSE
%   - Sample every knot plus a uniform display and validation time grid.
%**************************************************************************
% INPUTS
%   - polynomial (scalar maintained polynomial struct)
%   - sampleTime_s (positive scalar)
%**************************************************************************
% OUTPUTS
%   - time_s and N-by-2 position through jerk histories.
%**************************************************************************
% UNITS
%   - Time is seconds; derivatives use degrees and seconds.
%**************************************************************************
initialTime_s = polynomial.SegmentStartTime_s(1);
uniformTime_s = (initialTime_s:sampleTime_s:polynomial.FinalTime_s).';
time_s = unique([uniformTime_s; polynomial.SegmentStartTime_s; ...
    polynomial.FinalTime_s]);
[time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
    jerk_deg_s3] = azElInternal.evaluateAzElPolynomial(polynomial, time_s);
end

function continuity = continuityDiagnostics(polynomial)
%% Section 0: Header & Readme
% SYNTAX
%   continuity = continuityDiagnostics(polynomial)
%**************************************************************************
% PURPOSE
%   - Measure position-through-jerk residuals at every interior knot.
%**************************************************************************
% INPUTS
%   - polynomial (scalar maintained polynomial struct)
%**************************************************************************
% OUTPUTS
%   - continuity (scalar struct)
%**************************************************************************
% UNITS
%   - Residual fields use the derivative unit named by each field.
%**************************************************************************
maximumResidual = zeros(1, 4);
arrays = {polynomial.positionPower_deg, ...
    polynomial.velocityPower_deg_s, ...
    polynomial.accelerationPower_deg_s2, ...
    polynomial.jerkPower_deg_s3};
for segmentIndex = 1:polynomial.SegmentCount - 1
    for derivativeIndex = 1:numel(arrays)
        leftValue = sum(reshape( ...
            arrays{derivativeIndex}(segmentIndex, :, :), 2, []), 2).';
        rightValue = reshape( ...
            arrays{derivativeIndex}(segmentIndex + 1, :, 1), 1, 2);
        maximumResidual(derivativeIndex) = max( ...
            maximumResidual(derivativeIndex), ...
            max(abs(leftValue - rightValue)));
    end
end
continuityTolerance = 1e-7;
continuity = struct( ...
    "MaximumPositionResidual_deg", maximumResidual(1), ...
    "MaximumVelocityResidual_deg_s", maximumResidual(2), ...
    "MaximumAccelerationResidual_deg_s2", maximumResidual(3), ...
    "MaximumJerkResidual_deg_s3", maximumResidual(4), ...
    "C3Continuous", all(maximumResidual <= continuityTolerance));
end

function cost_deg2_s5 = integratedSquaredJerk(polynomial)
%% Section 0: Header & Readme
% SYNTAX
%   cost_deg2_s5 = integratedSquaredJerk(polynomial)
%**************************************************************************
% PURPOSE
%   - Integrate squared two-axis jerk exactly over every polynomial span.
%**************************************************************************
% INPUTS
%   - polynomial (scalar maintained polynomial struct)
%**************************************************************************
% OUTPUTS
%   - cost_deg2_s5 (nonnegative scalar)
%**************************************************************************
% UNITS
%   - Integrated squared jerk is square degrees per seconds to the fifth.
%**************************************************************************
cost_deg2_s5 = 0;
for segmentIndex = 1:polynomial.SegmentCount
    for axisIndex = 1:2
        jerkPower = reshape( ...
            polynomial.jerkPower_deg_s3(segmentIndex, axisIndex, :), ...
            1, []);
        squaredPower = conv(jerkPower, jerkPower);
        normalizedIntegral = sum(squaredPower ./ (1:numel(squaredPower)));
        cost_deg2_s5 = cost_deg2_s5 + ...
            polynomial.SegmentDuration_s(segmentIndex) * ...
            normalizedIntegral;
    end
end
end

function motion = emptyMotion(options)
%% Section 0: Header & Readme
% SYNTAX
%   motion = emptyMotion(options)
%**************************************************************************
% PURPOSE
%   - Define one stable prototype schema for success and validation failure.
%**************************************************************************
% INPUTS
%   - options (resolved prototype options)
%**************************************************************************
% OUTPUTS
%   - motion (scalar prototype motion struct)
%**************************************************************************
% UNITS
%   - Motion fields use degrees and seconds.
%**************************************************************************
emptyPolynomial = struct( ...
    "SegmentCount", 0, "SegmentStartTime_s", zeros(0, 1), ...
    "SegmentDuration_s", zeros(0, 1), "FinalTime_s", NaN, ...
    "positionPower_deg", zeros(0, 2, 6), ...
    "velocityPower_deg_s", zeros(0, 2, 5), ...
    "accelerationPower_deg_s2", zeros(0, 2, 4), ...
    "jerkPower_deg_s3", zeros(0, 2, 3), ...
    "TerminalState", struct( ...
    "position_deg", [NaN NaN], "velocity_deg_s", [NaN NaN], ...
    "acceleration_deg_s2", [NaN NaN]));
emptyRepresentation = struct( ...
    "Degree", 5, "SpanCount", 0, ...
    "ControlPointOffsetParameterCount", 0, ...
    "RelativeTimingParameterCount", 0, ...
    "TotalParameterCount", 0, ...
    "InteriorRouteInterpolated", false, ...
    "MaintainedValidatorCompatible", true);
motion = struct( ...
    "Success", false, "Message", "The prototype was not constructed.", ...
    "TerminationReason", "notRun", "MotionSource", "", ...
    "FinalTime_s", NaN, "MotionDuration_s", NaN, ...
    "IntegratedSquaredJerk_deg2_s5", NaN, ...
    "time_s", zeros(0, 1), "position_deg", zeros(0, 2), ...
    "velocity_deg_s", zeros(0, 2), ...
    "acceleration_deg_s2", zeros(0, 2), ...
    "jerk_deg_s3", zeros(0, 2), "Polynomial", emptyPolynomial, ...
    "Route_deg", zeros(0, 2), "ControlPoint_deg", zeros(0, 2), ...
    "KnotTime_s", zeros(1, 0), "SpanDuration_s", zeros(0, 1), ...
    "SeedCorridorBoundary_deg", zeros(0, 2), ...
    "SeedCorridor", struct([]), ...
    "RepresentationDiagnostics", emptyRepresentation, ...
    "Continuity", struct( ...
    "MaximumPositionResidual_deg", NaN, ...
    "MaximumVelocityResidual_deg_s", NaN, ...
    "MaximumAccelerationResidual_deg_s2", NaN, ...
    "MaximumJerkResidual_deg_s3", NaN, "C3Continuous", false), ...
    "Validation", validateAzElTrajectory(), "Options", options);
end
