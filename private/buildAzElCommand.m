function [command, waits] = buildAzElCommand(routePosition_deg, ...
        initialState, goalState, limits, options, arrivalTime_s, ...
        waitDuration_s)
%% Section 0: Header & Readme
% SYNTAX
%   [command, waits] = buildAzElCommand(routePosition_deg, initialState, ...
%       goalState, limits, options, arrivalTime_s, waitDuration_s)
%**************************************************************************
% PURPOSE
%   - Construct one coherent C2 piecewise-quintic command through a spatial
%     route while carrying a shared velocity through safe interior turns.
%**************************************************************************
% INPUTS
%   - routePosition_deg (N-by-2 numeric)
%       Start, internally selected route points, and terminal position.
%   - initialState (scalar complete-state struct)
%   - goalState (scalar complete-state struct)
%   - limits (scalar physical-limit struct)
%   - options (scalar resolved mission-option struct)
%   - arrivalTime_s (finite scalar)
%   - waitDuration_s (nonnegative scalar)
%**************************************************************************
% OUTPUTS
%   - command (scalar struct)
%       Continuous unwrapped knot state and wrapped display position.
%   - waits (struct column)
%       Explicit stationary intervals.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

%% Section 1: Normalize Route & Timing
routePosition_deg = double(routePosition_deg);
if size(routePosition_deg, 2) ~= 2 || size(routePosition_deg, 1) < 2 || ...
        any(~isfinite(routePosition_deg), "all")
    error("buildAzElCommand:InvalidRoute", ...
        "routePosition_deg must be a finite N-by-2 array with N >= 2.");
end
routePosition_deg(1, :) = initialState.position_deg;
routePosition_deg(end, :) = goalState.position_deg;
routePosition_deg = removeAdjacentDuplicates(routePosition_deg);

startTime_s = initialState.time_s;
departureTime_s = startTime_s + waitDuration_s;
motionDuration_s = arrivalTime_s - departureTime_s;
if motionDuration_s <= 0
    error("buildAzElCommand:NonpositiveMotionDuration", ...
        "arrivalTime_s must be later than departure time.");
end
if waitDuration_s > options.arrivalTolerance_s && ...
        (norm(initialState.velocity_deg_s) > ...
            options.velocityTolerance_deg_s || ...
        norm(initialState.acceleration_deg_s2) > ...
            options.accelerationTolerance_deg_s2)
    error("buildAzElCommand:InfeasibleInitialWait", ...
        "A stationary initial wait requires zero initial rate and acceleration.");
end

isRestToRest = all(abs([ ...
    initialState.velocity_deg_s, goalState.velocity_deg_s, ...
    initialState.acceleration_deg_s2, ...
    goalState.acceleration_deg_s2]) <= 1e-12);
if size(routePosition_deg, 1) == 2 && isRestToRest
    [command, waits] = buildRestToRestSCurveCommand( ...
        routePosition_deg, initialState, limits, options, ...
        arrivalTime_s, waitDuration_s);
    return;
end

%% Section 2: Allocate Segment Durations
segmentDelta_deg = diff(routePosition_deg, 1, 1);
rateTime_s = max(abs(segmentDelta_deg) ./ ...
    limits.maxVelocity_deg_s, [], 2);
accelerationTime_s = max(sqrt(2 .* abs(segmentDelta_deg) ./ ...
    limits.maxAcceleration_deg_s2), [], 2);
scaledDistance = vecnorm(segmentDelta_deg ./ ...
    limits.maxVelocity_deg_s, 2, 2);
durationWeight = max([rateTime_s, 0.5 .* accelerationTime_s, ...
    scaledDistance], [], 2);
durationWeight = max(durationWeight, ...
    1e-6 .* max(1, max(durationWeight)));
segmentDuration_s = motionDuration_s .* durationWeight ./ ...
    sum(durationWeight);
motionKnotTime_s = departureTime_s + [0; cumsum(segmentDuration_s)];
motionKnotTime_s(end) = arrivalTime_s;

%% Section 3: Assign Shared Interior Derivatives
routeCount = size(routePosition_deg, 1);
velocity_deg_s = zeros(routeCount, 2);
acceleration_deg_s2 = zeros(routeCount, 2);
velocity_deg_s(1, :) = initialState.velocity_deg_s;
velocity_deg_s(end, :) = goalState.velocity_deg_s;
acceleration_deg_s2(1, :) = initialState.acceleration_deg_s2;
acceleration_deg_s2(end, :) = goalState.acceleration_deg_s2;

for routeIndex = 2:(routeCount - 1)
    previousDelta_deg = routePosition_deg(routeIndex, :) - ...
        routePosition_deg(routeIndex - 1, :);
    nextDelta_deg = routePosition_deg(routeIndex + 1, :) - ...
        routePosition_deg(routeIndex, :);
    previousDuration_s = motionKnotTime_s(routeIndex) - ...
        motionKnotTime_s(routeIndex - 1);
    nextDuration_s = motionKnotTime_s(routeIndex + 1) - ...
        motionKnotTime_s(routeIndex);
    velocity_deg_s(routeIndex, :) = carriedTurnVelocity( ...
        previousDelta_deg, nextDelta_deg, previousDuration_s, ...
        nextDuration_s, limits.maxVelocity_deg_s);
end

%% Section 4: Prepend An Explicit Hold
waits = repmat(struct( ...
    "startTime_s", 0, ...
    "endTime_s", 0, ...
    "position_deg", [0, 0]), 0, 1);
if waitDuration_s > options.arrivalTolerance_s
    knotTime_s = [startTime_s; motionKnotTime_s];
    unwrappedPosition_deg = [initialState.position_deg; routePosition_deg];
    velocity_deg_s = [initialState.velocity_deg_s; velocity_deg_s];
    acceleration_deg_s2 = [initialState.acceleration_deg_s2; ...
        acceleration_deg_s2];
    waits(1, 1) = struct( ...
        "startTime_s", startTime_s, ...
        "endTime_s", departureTime_s, ...
        "position_deg", initialState.position_deg);
else
    knotTime_s = motionKnotTime_s;
    unwrappedPosition_deg = routePosition_deg;
end

%% Section 5: Assemble Wrapped & Unwrapped Histories
displayPosition_deg = unwrappedPosition_deg;
if options.azimuthWrap
    displayPosition_deg(:, 1) = wrapAzimuth( ...
        unwrappedPosition_deg(:, 1), limits.azimuth_deg);
end
command = struct( ...
    "interpolation", "quinticHermite", ...
    "time_s", knotTime_s, ...
    "position_deg", displayPosition_deg, ...
    "unwrappedPosition_deg", unwrappedPosition_deg, ...
    "velocity_deg_s", velocity_deg_s, ...
    "acceleration_deg_s2", acceleration_deg_s2, ...
    "azimuthWrap", options.azimuthWrap, ...
    "azimuthDisplayRange_deg", limits.azimuth_deg);
end

function [command, waits] = buildRestToRestSCurveCommand( ...
        routePosition_deg, initialState, limits, options, ...
        arrivalTime_s, waitDuration_s)
%% Section 0: Header & Readme
% SYNTAX
%   [command, waits] = buildRestToRestSCurveCommand( ...
%       routePosition_deg, initialState, limits, options, ...
%       arrivalTime_s, waitDuration_s)
%**************************************************************************
% PURPOSE
%   - Construct a direct C2 rest-to-rest command from symmetric constant-
%     jerk, constant-acceleration, and constant-rate phases.
%   - Approach the double-integrator time bound without imposing a jerk
%     limit that is absent from the mission contract.
%**************************************************************************
% INPUTS
%   - routePosition_deg (2-by-2 numeric)
%   - initialState (scalar complete-state struct)
%   - limits (scalar physical-limit struct)
%   - options (scalar resolved mission-option struct)
%   - arrivalTime_s (finite scalar)
%   - waitDuration_s (nonnegative scalar)
%**************************************************************************
% OUTPUTS
%   - command (scalar piecewise-quintic command)
%   - waits (struct column)
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

departureTime_s = initialState.time_s + waitDuration_s;
motionDuration_s = arrivalTime_s - departureTime_s;
displacement_deg = routePosition_deg(2, :) - ...
    routePosition_deg(1, :);

axisProfile = repmat(emptyAxisProfile(), 1, 2);
localKnotTime_s = [0; motionDuration_s];
for axisIndex = 1:2
    axisProfile(axisIndex) = makeAxisProfile( ...
        displacement_deg(axisIndex), motionDuration_s, ...
        limits.maxVelocity_deg_s(axisIndex), ...
        limits.maxAcceleration_deg_s2(axisIndex));
    localKnotTime_s = [localKnotTime_s; ...
        axisProfile(axisIndex).boundaryTime_s]; %#ok<AGROW>
end
localKnotTime_s = coalesceKnotTimes( ...
    localKnotTime_s, motionDuration_s);
localKnotTime_s(1) = 0;
localKnotTime_s(end) = motionDuration_s;

knotCount = numel(localKnotTime_s);
unwrappedPosition_deg = zeros(knotCount, 2);
velocity_deg_s = zeros(knotCount, 2);
acceleration_deg_s2 = zeros(knotCount, 2);
for axisIndex = 1:2
    [relativePosition_deg, axisVelocity_deg_s, ...
        axisAcceleration_deg_s2] = evaluateAxisProfile( ...
        axisProfile(axisIndex), localKnotTime_s);
    unwrappedPosition_deg(:, axisIndex) = ...
        routePosition_deg(1, axisIndex) + relativePosition_deg;
    velocity_deg_s(:, axisIndex) = axisVelocity_deg_s;
    acceleration_deg_s2(:, axisIndex) = axisAcceleration_deg_s2;
end
unwrappedPosition_deg(1, :) = routePosition_deg(1, :);
unwrappedPosition_deg(end, :) = routePosition_deg(2, :);
velocity_deg_s([1, end], :) = 0;
acceleration_deg_s2([1, end], :) = 0;
knotTime_s = departureTime_s + localKnotTime_s;

waits = repmat(struct( ...
    "startTime_s", 0, ...
    "endTime_s", 0, ...
    "position_deg", [0, 0]), 0, 1);
if waitDuration_s > options.arrivalTolerance_s
    knotTime_s = [initialState.time_s; knotTime_s];
    unwrappedPosition_deg = [routePosition_deg(1, :); ...
        unwrappedPosition_deg];
    velocity_deg_s = [0, 0; velocity_deg_s];
    acceleration_deg_s2 = [0, 0; acceleration_deg_s2];
    waits(1, 1) = struct( ...
        "startTime_s", initialState.time_s, ...
        "endTime_s", departureTime_s, ...
        "position_deg", routePosition_deg(1, :));
end

displayPosition_deg = unwrappedPosition_deg;
if options.azimuthWrap
    displayPosition_deg(:, 1) = wrapAzimuth( ...
        unwrappedPosition_deg(:, 1), limits.azimuth_deg);
end
command = struct( ...
    "interpolation", "quinticHermite", ...
    "time_s", knotTime_s, ...
    "position_deg", displayPosition_deg, ...
    "unwrappedPosition_deg", unwrappedPosition_deg, ...
    "velocity_deg_s", velocity_deg_s, ...
    "acceleration_deg_s2", acceleration_deg_s2, ...
    "azimuthWrap", options.azimuthWrap, ...
    "azimuthDisplayRange_deg", limits.azimuth_deg);
end

function profile = makeAxisProfile(displacement_deg, duration_s, ...
        maximumVelocity_deg_s, maximumAcceleration_deg_s2)
%% Section 0: Header & Readme
% SYNTAX
%   profile = makeAxisProfile(displacement_deg, duration_s, ...
%       maximumVelocity_deg_s, maximumAcceleration_deg_s2)
%**************************************************************************
% PURPOSE
%   - Define one symmetric seven-phase S-curve over a prescribed duration.
%**************************************************************************
% INPUTS
%   - displacement_deg (finite scalar)
%   - duration_s, maximumVelocity_deg_s, maximumAcceleration_deg_s2
%       Positive scalars.
%**************************************************************************
% OUTPUTS
%   - profile (scalar struct)
%**************************************************************************
% UNITS
%   - Inputs use degrees and seconds.

if abs(displacement_deg) <= 64 * eps(max(1, abs(displacement_deg)))
    profile = struct( ...
        "duration_s", duration_s, ...
        "phaseDuration_s", duration_s, ...
        "jerk_deg_s3", 0, ...
        "boundaryTime_s", [0; duration_s], ...
        "exceedsAccelerationLimit", false);
    return;
end

distance_deg = abs(displacement_deg);
accelerationPhase_s = min(duration_s ./ 2, ...
    duration_s - distance_deg ./ maximumVelocity_deg_s);
minimumPhase_s = 256 .* eps(max(1, duration_s));
accelerationPhase_s = max(accelerationPhase_s, minimumPhase_s);
rampDuration_s = min(0.005 .* duration_s, ...
    0.49 .* accelerationPhase_s);
rampDuration_s = max(rampDuration_s, minimumPhase_s);
constantAcceleration_s = max(0, ...
    accelerationPhase_s - 2 .* rampDuration_s);
cruiseDuration_s = max(0, ...
    duration_s - 2 .* accelerationPhase_s);

denominator = (accelerationPhase_s - rampDuration_s) .* ...
    (duration_s - accelerationPhase_s);
peakAcceleration_deg_s2 = distance_deg ./ denominator;
direction = sign(displacement_deg);
signedJerk_deg_s3 = direction .* ...
    peakAcceleration_deg_s2 ./ rampDuration_s;
phaseDuration_s = [ ...
    rampDuration_s; constantAcceleration_s; rampDuration_s; ...
    cruiseDuration_s; rampDuration_s; constantAcceleration_s; ...
    rampDuration_s];
jerk_deg_s3 = [ ...
    signedJerk_deg_s3; 0; -signedJerk_deg_s3; 0; ...
    -signedJerk_deg_s3; 0; signedJerk_deg_s3];
boundaryTime_s = [0; cumsum(phaseDuration_s)];
boundaryTime_s(end) = duration_s;
profile = struct( ...
    "duration_s", duration_s, ...
    "phaseDuration_s", phaseDuration_s, ...
    "jerk_deg_s3", jerk_deg_s3, ...
    "boundaryTime_s", unique(boundaryTime_s), ...
    "exceedsAccelerationLimit", ...
        peakAcceleration_deg_s2 > maximumAcceleration_deg_s2);
end

function [position_deg, velocity_deg_s, acceleration_deg_s2] = ...
        evaluateAxisProfile(profile, queryTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [position_deg, velocity_deg_s, acceleration_deg_s2] = ...
%       evaluateAxisProfile(profile, queryTime_s)
%**************************************************************************
% PURPOSE
%   - Integrate one piecewise-constant-jerk axis profile exactly.
%**************************************************************************
% INPUTS
%   - profile (scalar axis-profile struct)
%   - queryTime_s (numeric column inside the profile duration)
%**************************************************************************
% OUTPUTS
%   - position_deg, velocity_deg_s, acceleration_deg_s2 (numeric columns)
%**************************************************************************
% UNITS
%   - Outputs use degrees and seconds.

queryTime_s = queryTime_s(:);
sampleCount = numel(queryTime_s);
position_deg = zeros(sampleCount, 1);
velocity_deg_s = zeros(sampleCount, 1);
acceleration_deg_s2 = zeros(sampleCount, 1);
for sampleIndex = 1:sampleCount
    remainingTime_s = queryTime_s(sampleIndex);
    samplePosition_deg = 0;
    sampleVelocity_deg_s = 0;
    sampleAcceleration_deg_s2 = 0;
    for phaseIndex = 1:numel(profile.phaseDuration_s)
        phaseStep_s = min(max(remainingTime_s, 0), ...
            profile.phaseDuration_s(phaseIndex));
        phaseJerk_deg_s3 = profile.jerk_deg_s3(phaseIndex);
        samplePosition_deg = samplePosition_deg + ...
            sampleVelocity_deg_s .* phaseStep_s + ...
            0.5 .* sampleAcceleration_deg_s2 .* phaseStep_s.^2 + ...
            phaseJerk_deg_s3 .* phaseStep_s.^3 ./ 6;
        sampleVelocity_deg_s = sampleVelocity_deg_s + ...
            sampleAcceleration_deg_s2 .* phaseStep_s + ...
            0.5 .* phaseJerk_deg_s3 .* phaseStep_s.^2;
        sampleAcceleration_deg_s2 = sampleAcceleration_deg_s2 + ...
            phaseJerk_deg_s3 .* phaseStep_s;
        remainingTime_s = remainingTime_s - phaseStep_s;
        if remainingTime_s <= 64 * eps(max(1, queryTime_s(sampleIndex)))
            break;
        end
    end
    position_deg(sampleIndex) = samplePosition_deg;
    velocity_deg_s(sampleIndex) = sampleVelocity_deg_s;
    acceleration_deg_s2(sampleIndex) = sampleAcceleration_deg_s2;
end
end

function profile = emptyAxisProfile()
%% Section 0: Header & Readme
% SYNTAX
%   profile = emptyAxisProfile()
%**************************************************************************
% PURPOSE
%   - Return the stable scalar schema used for axis-profile preallocation.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - profile (scalar struct)
%**************************************************************************
% UNITS
%   - Not applicable.

profile = struct( ...
    "duration_s", 0, ...
    "phaseDuration_s", zeros(0, 1), ...
    "jerk_deg_s3", zeros(0, 1), ...
    "boundaryTime_s", zeros(0, 1), ...
    "exceedsAccelerationLimit", false);
end

function knotTime_s = coalesceKnotTimes(knotTime_s, duration_s)
%% Section 0: Header & Readme
% SYNTAX
%   knotTime_s = coalesceKnotTimes(knotTime_s, duration_s)
%**************************************************************************
% PURPOSE
%   - Merge roundoff-level duplicates from independently phased axes so no
%     numerically singular Hermite interval is created.
%**************************************************************************
% INPUTS
%   - knotTime_s (numeric vector)
%   - duration_s (positive scalar)
%**************************************************************************
% OUTPUTS
%   - knotTime_s (strictly increasing numeric column)
%**************************************************************************
% UNITS
%   - Times are seconds.

knotTime_s = sort(knotTime_s(:));
mergeTolerance_s = 1e-10 .* max(1, duration_s);
writeIndex = 1;
for readIndex = 2:numel(knotTime_s)
    if all(knotTime_s(readIndex) - knotTime_s(writeIndex) <= ...
            mergeTolerance_s, "all")
        knotTime_s(writeIndex) = 0.5 .* ( ...
            knotTime_s(writeIndex) + knotTime_s(readIndex));
    else
        writeIndex = writeIndex + 1;
        knotTime_s(writeIndex) = knotTime_s(readIndex);
    end
end
knotTime_s = knotTime_s(1:writeIndex);
end

function routePosition_deg = removeAdjacentDuplicates(routePosition_deg)
%% Section 0: Header & Readme
% SYNTAX
%   routePosition_deg = removeAdjacentDuplicates(routePosition_deg)
%**************************************************************************
% PURPOSE
%   - Remove zero-length route edges before duration allocation.
%**************************************************************************
% INPUTS
%   - routePosition_deg (N-by-2 numeric)
%**************************************************************************
% OUTPUTS
%   - routePosition_deg (M-by-2 numeric)
%**************************************************************************
% UNITS
%   - Positions are degrees.

if size(routePosition_deg, 1) <= 2
    return;
end
edgeLength_deg = vecnorm(diff(routePosition_deg, 1, 1), 2, 2);
scale_deg = max(1, max(abs(routePosition_deg), [], "all"));
keep = [true; edgeLength_deg > 256 .* eps(scale_deg)];
routePosition_deg = routePosition_deg(keep, :);
if size(routePosition_deg, 1) < 2
    error("buildAzElCommand:DegenerateRoute", ...
        "Route collapsed to fewer than two distinct states.");
end
end

function velocity_deg_s = carriedTurnVelocity(previousDelta_deg, ...
        nextDelta_deg, previousDuration_s, nextDuration_s, ...
        maximumVelocity_deg_s)
%% Section 0: Header & Readme
% SYNTAX
%   velocity_deg_s = carriedTurnVelocity(previousDelta_deg, ...
%       nextDelta_deg, previousDuration_s, nextDuration_s, ...
%       maximumVelocity_deg_s)
%**************************************************************************
% PURPOSE
%   - Assign a deterministic velocity along the scaled angle bisector so
%     motion carries through a safe turn without exceeding either edge rate.
%**************************************************************************
% INPUTS
%   - previousDelta_deg (1-by-2 numeric)
%   - nextDelta_deg (1-by-2 numeric)
%   - previousDuration_s (positive scalar)
%   - nextDuration_s (positive scalar)
%   - maximumVelocity_deg_s (1-by-2 positive numeric)
%**************************************************************************
% OUTPUTS
%   - velocity_deg_s (1-by-2 numeric)
%**************************************************************************
% UNITS
%   - Angle is degrees and velocity is deg/s.

scaledPrevious = previousDelta_deg ./ maximumVelocity_deg_s;
scaledNext = nextDelta_deg ./ maximumVelocity_deg_s;
previousLength = norm(scaledPrevious);
nextLength = norm(scaledNext);
if previousLength == 0 || nextLength == 0
    velocity_deg_s = [0, 0];
    return;
end
previousDirection = scaledPrevious ./ previousLength;
nextDirection = scaledNext ./ nextLength;
bisector = previousDirection + nextDirection;
if norm(bisector) <= 0.15 || dot(previousDirection, nextDirection) < -0.95
    velocity_deg_s = [0, 0];
    return;
end
bisector = bisector ./ norm(bisector);
previousScaledSpeed = previousLength ./ previousDuration_s;
nextScaledSpeed = nextLength ./ nextDuration_s;
carriedScaledSpeed = 0.65 .* min([ ...
    previousScaledSpeed, nextScaledSpeed, 1]);
velocity_deg_s = bisector .* carriedScaledSpeed .* ...
    maximumVelocity_deg_s;
velocity_deg_s = min(max(velocity_deg_s, ...
    -0.8 .* maximumVelocity_deg_s), 0.8 .* maximumVelocity_deg_s);
end

function wrappedAzimuth_deg = wrapAzimuth( ...
        unwrappedAzimuth_deg, azimuthRange_deg)
%% Section 0: Header & Readme
% SYNTAX
%   wrappedAzimuth_deg = wrapAzimuth( ...
%       unwrappedAzimuth_deg, azimuthRange_deg)
%**************************************************************************
% PURPOSE
%   - Map continuous azimuth to the caller's display interval.
%**************************************************************************
% INPUTS
%   - unwrappedAzimuth_deg (numeric array)
%   - azimuthRange_deg (1-by-2 increasing numeric)
%**************************************************************************
% OUTPUTS
%   - wrappedAzimuth_deg (numeric array)
%**************************************************************************
% UNITS
%   - Azimuth is degrees.

span_deg = diff(azimuthRange_deg);
wrappedAzimuth_deg = azimuthRange_deg(1) + mod( ...
    unwrappedAzimuth_deg - azimuthRange_deg(1), span_deg);
end
