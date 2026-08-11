function validation = validateAzElCommand(command, request, arrivalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   validation = validateAzElCommand(command, request)
%   validation = validateAzElCommand(command, request, arrivalTime_s)
%**************************************************************************
% PURPOSE
%   - Independently validate a complete piecewise-quintic command against
%     canonical polygons, full dynamics, endpoints, waits, wrapping, and
%     first-arrival semantics.
%**************************************************************************
% INPUTS
%   - command (scalar struct)
%       Knot time, wrapped and continuous positions, velocity,
%       acceleration, interpolation, and wrap metadata.
%   - request (scalar struct)
%       The same mission request supplied to planAzElAvoidance.
%   - arrivalTime_s (optional finite scalar)
%       Claimed first complete-state arrival. Defaults to command end.
%**************************************************************************
% OUTPUTS
%   - validation (scalar struct)
%       Stable independent evidence with isValid and component results.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

%% Section 1: Normalize Inputs
if ~isfield(request, "isNormalizedInternal") || ...
        ~request.isNormalizedInternal
    request = normalizeAzElPlannerRequest(request);
end
validation = validationTemplate();
if ~isstruct(command) || ~isscalar(command) || ...
        ~isfield(command, "time_s") || isempty(command.time_s)
    validation.message = "Command is empty or structurally invalid.";
    return;
end
if nargin < 3 || isempty(arrivalTime_s)
    arrivalTime_s = command.time_s(end);
end
validateattributes(arrivalTime_s, "numeric", ...
    ["scalar", "real", "finite"], mfilename, "arrivalTime_s");
if string(command.interpolation) ~= "quinticHermite"
    validation.message = "Unsupported command interpolation contract.";
    return;
end
if arrivalTime_s < command.time_s(1) - ...
        request.options.arrivalTolerance_s || ...
        arrivalTime_s > command.time_s(end) + ...
        request.options.arrivalTolerance_s
    validation.message = "Claimed arrival lies outside the command interval.";
    return;
end

%% Section 2: Validate Continuous Motion
motion = validateAzElMotion(command, request.limits, request.options);
validation.motion = motion;

%% Section 3: Validate Complete Boundary States
boundaryState = sampleAzElCommand(command, ...
    [command.time_s(1); arrivalTime_s]);
initialPositionError_deg = boundaryState.unwrappedPosition_deg(1, :) - ...
    request.initialState.position_deg;
initialVelocityError_deg_s = boundaryState.velocity_deg_s(1, :) - ...
    request.initialState.velocity_deg_s;
initialAccelerationError_deg_s2 = ...
    boundaryState.acceleration_deg_s2(1, :) - ...
    request.initialState.acceleration_deg_s2;
goalState = evaluateAzElGoal(request.goal, arrivalTime_s);
terminalPositionError_deg = boundaryState.unwrappedPosition_deg(2, :) - ...
    goalState.position_deg;
terminalVelocityError_deg_s = boundaryState.velocity_deg_s(2, :) - ...
    goalState.velocity_deg_s;
terminalAccelerationError_deg_s2 = ...
    boundaryState.acceleration_deg_s2(2, :) - ...
    goalState.acceleration_deg_s2;

initialStateIsValid = ...
    all(abs(initialPositionError_deg) <= ...
        request.options.positionTolerance_deg) && ...
    all(abs(initialVelocityError_deg_s) <= ...
        request.options.velocityTolerance_deg_s) && ...
    all(abs(initialAccelerationError_deg_s2) <= ...
        request.options.accelerationTolerance_deg_s2);
terminalStateIsValid = ...
    all(abs(terminalPositionError_deg) <= ...
        request.options.positionTolerance_deg) && ...
    all(abs(terminalVelocityError_deg_s) <= ...
        request.options.velocityTolerance_deg_s) && ...
    all(abs(terminalAccelerationError_deg_s2) <= ...
        request.options.accelerationTolerance_deg_s2);

validation.initialStateIsValid = initialStateIsValid;
validation.terminalStateIsValid = terminalStateIsValid;
validation.initialPositionError_deg = initialPositionError_deg;
validation.initialVelocityError_deg_s = initialVelocityError_deg_s;
validation.initialAccelerationError_deg_s2 = ...
    initialAccelerationError_deg_s2;
validation.terminalPositionError_deg = terminalPositionError_deg;
validation.terminalVelocityError_deg_s = terminalVelocityError_deg_s;
validation.terminalAccelerationError_deg_s2 = ...
    terminalAccelerationError_deg_s2;

%% Section 4: Validate Collision Clearance & Wrapping
collision = validateAzElCollision(command, request, motion);
validation.collision = collision;
if request.options.azimuthWrap
    span_deg = diff(request.limits.azimuth_deg);
    wrappedFromContinuous_deg = request.limits.azimuth_deg(1) + mod( ...
        command.unwrappedPosition_deg(:, 1) - ...
            request.limits.azimuth_deg(1), span_deg);
    wrapError_deg = circularDifference(wrappedFromContinuous_deg, ...
        command.position_deg(:, 1), span_deg);
    seamIsValid = all(abs(wrapError_deg) <= ...
        request.options.positionTolerance_deg);
else
    seamIsValid = all(abs(command.unwrappedPosition_deg(:, 1) - ...
        command.position_deg(:, 1)) <= ...
        request.options.positionTolerance_deg);
end
validation.seamIsValid = seamIsValid;

%% Section 5: Validate Waits & First Arrival
[waitCount, totalWaitDuration_s, waitsAreFeasible] = ...
    validateWaitIntervals(command, request.options);
firstArrivalIsValid = noEarlierCompleteArrival(command, request, ...
    arrivalTime_s);
validation.waitCount = waitCount;
validation.totalWaitDuration_s = totalWaitDuration_s;
validation.waitsAreFeasible = waitsAreFeasible;
validation.firstArrivalIsValid = firstArrivalIsValid;
validation.arrivalTime_s = arrivalTime_s;
validation.executionDuration_s = arrivalTime_s - ...
    request.initialState.time_s;

%% Section 6: Assemble Overall Evidence
validation.isValid = motion.isValid && initialStateIsValid && ...
    terminalStateIsValid && collision.collisionFree && ...
    collision.resolved && seamIsValid && waitsAreFeasible && ...
    firstArrivalIsValid;
if validation.isValid
    validation.message = ["Command is independently validated over every " ...
        "continuous interval."];
elseif ~collision.resolved
    validation.message = "Collision clearance remains numerically unresolved.";
elseif ~collision.collisionFree
    validation.message = "Command violates obstacle safety clearance.";
elseif ~motion.isValid
    validation.message = "Command violates a continuous motion limit.";
elseif ~initialStateIsValid || ~terminalStateIsValid
    validation.message = "Command does not match a complete boundary state.";
elseif ~firstArrivalIsValid
    validation.message = "Claimed arrival is not the first complete-state arrival.";
else
    validation.message = "Command failed an independent validation gate.";
end
end

function validation = validationTemplate()
%% Section 0: Header & Readme
% SYNTAX
%   validation = validationTemplate()
%**************************************************************************
% PURPOSE
%   - Return the stable command-validation schema.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - validation (scalar struct)
%**************************************************************************
% UNITS
%   - Field names state their units.

validation = struct( ...
    "isValid", false, ...
    "message", "Validation did not complete.", ...
    "initialStateIsValid", false, ...
    "terminalStateIsValid", false, ...
    "initialPositionError_deg", [Inf, Inf], ...
    "initialVelocityError_deg_s", [Inf, Inf], ...
    "initialAccelerationError_deg_s2", [Inf, Inf], ...
    "terminalPositionError_deg", [Inf, Inf], ...
    "terminalVelocityError_deg_s", [Inf, Inf], ...
    "terminalAccelerationError_deg_s2", [Inf, Inf], ...
    "seamIsValid", false, ...
    "waitsAreFeasible", false, ...
    "waitCount", 0, ...
    "totalWaitDuration_s", 0, ...
    "firstArrivalIsValid", false, ...
    "arrivalTime_s", NaN, ...
    "executionDuration_s", NaN, ...
    "motion", struct(), ...
    "collision", struct());
end

function difference_deg = circularDifference(first_deg, second_deg, span_deg)
%% Section 0: Header & Readme
% SYNTAX
%   difference_deg = circularDifference(first_deg, second_deg, span_deg)
%**************************************************************************
% PURPOSE
%   - Compute the signed minimum modular azimuth difference.
%**************************************************************************
% INPUTS
%   - first_deg, second_deg (numeric arrays)
%   - span_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - difference_deg (numeric array)
%**************************************************************************
% UNITS
%   - Azimuth difference is degrees.

difference_deg = mod(first_deg - second_deg + 0.5 .* span_deg, ...
    span_deg) - 0.5 .* span_deg;
end

function [waitCount, totalWaitDuration_s, waitsAreFeasible] = ...
        validateWaitIntervals(command, options)
%% Section 0: Header & Readme
% SYNTAX
%   [waitCount, totalWaitDuration_s, waitsAreFeasible] = ...
%       validateWaitIntervals(command, options)
%**************************************************************************
% PURPOSE
%   - Identify explicit quintic holds and verify stationary endpoint states.
%**************************************************************************
% INPUTS
%   - command (scalar struct)
%   - options (scalar resolved mission-option struct)
%**************************************************************************
% OUTPUTS
%   - waitCount (nonnegative integer)
%   - totalWaitDuration_s (nonnegative scalar)
%   - waitsAreFeasible (logical scalar)
%**************************************************************************
% UNITS
%   - Time is seconds.

waitCount = 0;
totalWaitDuration_s = 0;
waitsAreFeasible = true;
for segmentIndex = 1:(numel(command.time_s) - 1)
    positionChange_deg = norm( ...
        command.unwrappedPosition_deg(segmentIndex + 1, :) - ...
        command.unwrappedPosition_deg(segmentIndex, :));
    isStationaryPosition = positionChange_deg <= ...
        options.positionTolerance_deg;
    if isStationaryPosition
        endpointVelocity_deg_s = command.velocity_deg_s( ...
            segmentIndex:(segmentIndex + 1), :);
        endpointAcceleration_deg_s2 = command.acceleration_deg_s2( ...
            segmentIndex:(segmentIndex + 1), :);
        intervalIsHold = all(abs(endpointVelocity_deg_s) <= ...
            options.velocityTolerance_deg_s, "all") && ...
            all(abs(endpointAcceleration_deg_s2) <= ...
            options.accelerationTolerance_deg_s2, "all");
        waitsAreFeasible = waitsAreFeasible && intervalIsHold;
        if intervalIsHold
            waitCount = waitCount + 1;
            totalWaitDuration_s = totalWaitDuration_s + ...
                command.time_s(segmentIndex + 1) - ...
                command.time_s(segmentIndex);
        end
    end
end
end

function isFirst = noEarlierCompleteArrival(command, request, arrivalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   isFirst = noEarlierCompleteArrival(command, request, arrivalTime_s)
%**************************************************************************
% PURPOSE
%   - Reject a claimed arrival when an earlier sampled complete-state match
%     exists; the final exact state check remains the arrival authority.
%**************************************************************************
% INPUTS
%   - command (scalar struct)
%   - request (normalized scalar request)
%   - arrivalTime_s (finite scalar)
%**************************************************************************
% OUTPUTS
%   - isFirst (logical scalar)
%**************************************************************************
% UNITS
%   - Time is seconds and state fields carry named angular units.

queryTime_s = zeros(0, 1);
for segmentIndex = 1:(numel(command.time_s) - 1)
    segmentStart_s = command.time_s(segmentIndex);
    segmentEnd_s = min(command.time_s(segmentIndex + 1), arrivalTime_s);
    if segmentEnd_s <= segmentStart_s
        continue;
    end
    segmentQuery_s = linspace(segmentStart_s, segmentEnd_s, 65).';
    queryTime_s = [queryTime_s; segmentQuery_s(1:end-1)]; %#ok<AGROW>
end
queryTime_s = unique(queryTime_s);
queryTime_s = queryTime_s(queryTime_s < arrivalTime_s - ...
    request.options.arrivalTolerance_s);
if isempty(queryTime_s)
    isFirst = true;
    return;
end
sampledState = sampleAzElCommand(command, queryTime_s);
isFirst = true;
for queryIndex = 1:numel(queryTime_s)
    if request.goal.type == "moving" && ...
            (queryTime_s(queryIndex) < request.goal.time_s(1) || ...
            queryTime_s(queryIndex) > request.goal.time_s(end))
        continue;
    end
    goalState = evaluateAzElGoal(request.goal, queryTime_s(queryIndex));
    positionMatches = all(abs( ...
        sampledState.unwrappedPosition_deg(queryIndex, :) - ...
        goalState.position_deg) <= request.options.positionTolerance_deg);
    velocityMatches = all(abs( ...
        sampledState.velocity_deg_s(queryIndex, :) - ...
        goalState.velocity_deg_s) <= request.options.velocityTolerance_deg_s);
    accelerationMatches = all(abs( ...
        sampledState.acceleration_deg_s2(queryIndex, :) - ...
        goalState.acceleration_deg_s2) <= ...
        request.options.accelerationTolerance_deg_s2);
    if positionMatches && velocityMatches && accelerationMatches
        isFirst = false;
        return;
    end
end
end
