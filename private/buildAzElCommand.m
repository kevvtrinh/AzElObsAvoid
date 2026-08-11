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
