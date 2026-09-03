function candidate = createDelayedMotion( ...
        baseMotion, initialState, breakTime_s, sampleStep_s, seedSource)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.createDelayedMotion( ...
%       baseMotion, initialState, breakTime_s, sampleStep_s, seedSource)
%**************************************************************************
% PURPOSE
%   - Prepend a stationary dwell and repartition an exact cubic motion.
%**************************************************************************
% INPUTS
%   - baseMotion (scalar trajectory-engine result struct)
%       Requires a complete piecewise-cubic Polynomial.
%   - initialState (scalar struct)
%       Requires time_s, position_deg, velocity_deg_s, and
%       acceleration_deg_s2.
%   - breakTime_s (numeric vector)
%       Absolute increasing output breaks from initial time through arrival.
%   - sampleStep_s (positive finite scalar)
%       Requested output-history spacing in seconds.
%   - seedSource (scalar text)
%       Input-driven construction label copied to the motion record.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar trajectory-engine result struct)
%       Exact delayed motion containing polynomial and sampled histories.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3. Histories are N-by-D.
%**************************************************************************

%% Section 1: Validate And Normalize The Requested Partition

if nargin ~= 5 || ~isstruct(baseMotion) || ~isscalar(baseMotion) || ...
        ~isfield(baseMotion, "Polynomial")
    error("createDelayedMotion:InvalidCall", ...
        "Five inputs and a scalar baseMotion.Polynomial are required.");
end
polynomial = baseMotion.Polynomial;
if ~isfield(polynomial, "Degree") || polynomial.Degree > 3
    error("createDelayedMotion:UnsupportedDegree", ...
        "baseMotion.Polynomial must have degree at most three.");
end
breakTime_s = unique(sort(double(breakTime_s(:))));
validateattributes(sampleStep_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
seedSource = string(seedSource);
clockScale_s = max(1, max(abs([initialState.time_s; ...
    polynomial.FinalTime_s])));
clockTolerance_s = 1024 * eps(clockScale_s);
partitionIsValid = numel(breakTime_s) >= 2 && ...
    all(isfinite(breakTime_s)) && all(diff(breakTime_s) > 0) && ...
    abs(breakTime_s(1) - initialState.time_s) <= clockTolerance_s && ...
    abs(breakTime_s(end) - polynomial.FinalTime_s) <= clockTolerance_s && ...
    isscalar(seedSource);
if ~partitionIsValid
    error("createDelayedMotion:InvalidPartition", ...
        "breakTime_s must increase from initialState.time_s through arrival.");
end

%% Section 2: Reconstruct Every Dwell And Motion Span

duration_s = diff(breakTime_s);
segmentCount = numel(duration_s);
dimensionCount = numel(initialState.position_deg);
positionPower_deg = zeros(segmentCount, dimensionCount, 4);
velocityPower_deg_s = zeros(segmentCount, dimensionCount, 3);
accelerationPower_deg_s2 = zeros(segmentCount, dimensionCount, 2);
jerkPower_deg_s3 = zeros(segmentCount, dimensionCount, 1);
baseStartTime_s = polynomial.SegmentStartTime_s(1);
for segmentIndex = 1:segmentCount
    startTime_s = breakTime_s(segmentIndex);
    midpoint_s = mean(breakTime_s(segmentIndex:segmentIndex + 1));
    step_s = duration_s(segmentIndex);
    if midpoint_s < baseStartTime_s
        position_deg = initialState.position_deg;
        velocity_deg_s = initialState.velocity_deg_s;
        acceleration_deg_s2 = initialState.acceleration_deg_s2;
        jerk_deg_s3 = zeros(1, dimensionCount);
    else
        [~, position_deg, velocity_deg_s, acceleration_deg_s2] = ...
            bmtpEngine.evaluatePolynomial(polynomial, startTime_s);
        [~, ~, ~, ~, jerk_deg_s3] = ...
            bmtpEngine.evaluatePolynomial(polynomial, midpoint_s);
    end
    [positionPower_deg(segmentIndex, :, :), ...
        velocityPower_deg_s(segmentIndex, :, :), ...
        accelerationPower_deg_s2(segmentIndex, :, :)] = ...
        bmtpEngine.createConstantJerkPowerCoefficients( ...
        position_deg, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3, step_s);
    jerkPower_deg_s3(segmentIndex, :, 1) = jerk_deg_s3;
end
finalTime_s = breakTime_s(end);
delayedPolynomial = struct( ...
    "Degree", 3, "SegmentCount", segmentCount, ...
    "SegmentStartTime_s", breakTime_s(1:end - 1), ...
    "SegmentDuration_s", duration_s, ...
    "SegmentBreakTau", (breakTime_s - breakTime_s(1)) / ...
        (finalTime_s - breakTime_s(1)), ...
    "FinalTime_s", finalTime_s, ...
    "positionPower_deg", positionPower_deg, ...
    "velocityPower_deg_s", velocityPower_deg_s, ...
    "accelerationPower_deg_s2", accelerationPower_deg_s2, ...
    "jerkPower_deg_s3", jerkPower_deg_s3, ...
    "TerminalState", polynomial.TerminalState);
candidate = bmtpEngine.createMotionRecord( ...
    baseMotion, initialState, delayedPolynomial, [], sampleStep_s, seedSource);
end
