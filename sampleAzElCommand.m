function state = sampleAzElCommand(command, queryTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   state = sampleAzElCommand(command, queryTime_s)
%**************************************************************************
% PURPOSE
%   - Evaluate the piecewise-quintic command without differencing wrapped
%     azimuth samples.
%**************************************************************************
% INPUTS
%   - command (scalar struct)
%       Planner command with knot time, unwrapped position, velocity, and
%       acceleration histories.
%   - queryTime_s (numeric vector)
%       Times inside the closed command interval.
%**************************************************************************
% OUTPUTS
%   - state (scalar struct)
%       time_s, position_deg, unwrappedPosition_deg, velocity_deg_s, and
%       acceleration_deg_s2 arrays with one row per query.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

%% Section 1: Validate Command & Query
requiredFields = ["time_s", "unwrappedPosition_deg", ...
    "velocity_deg_s", "acceleration_deg_s2", "azimuthWrap", ...
    "azimuthDisplayRange_deg"];
if ~isstruct(command) || ~isscalar(command)
    error("sampleAzElCommand:InvalidCommand", ...
        "command must be one scalar structure.");
end
for fieldIndex = 1:numel(requiredFields)
    if ~isfield(command, requiredFields(fieldIndex))
        error("sampleAzElCommand:MissingField", ...
            "command is missing required field '%s'.", ...
            requiredFields(fieldIndex));
    end
end

knotTime_s = double(command.time_s(:));
queryShape = size(queryTime_s);
queryTime_s = double(queryTime_s(:));
validateattributes(knotTime_s, "numeric", ...
    ["real", "finite", "nonempty", "increasing"], mfilename, ...
    "command.time_s");
validateattributes(queryTime_s, "numeric", ...
    ["real", "finite"], mfilename, "queryTime_s");
timeTolerance_s = 64 .* eps(max(1, max(abs(knotTime_s))));
if any(queryTime_s < knotTime_s(1) - timeTolerance_s) || ...
        any(queryTime_s > knotTime_s(end) + timeTolerance_s)
    error("sampleAzElCommand:QueryOutsideCommand", ...
        "Every query time must lie inside the command interval.");
end
queryTime_s = min(max(queryTime_s, knotTime_s(1)), knotTime_s(end));

unwrappedPosition_deg = double(command.unwrappedPosition_deg);
velocity_deg_s = double(command.velocity_deg_s);
acceleration_deg_s2 = double(command.acceleration_deg_s2);
knotCount = numel(knotTime_s);
expectedSize = [knotCount, 2];
if ~isequal(size(unwrappedPosition_deg), expectedSize) || ...
        ~isequal(size(velocity_deg_s), expectedSize) || ...
        ~isequal(size(acceleration_deg_s2), expectedSize)
    error("sampleAzElCommand:HistorySizeMismatch", ...
        "Position, velocity, and acceleration must be knotCount-by-2.");
end

%% Section 2: Evaluate Quintic Segments
queryCount = numel(queryTime_s);
sampledPosition_deg = zeros(queryCount, 2);
sampledVelocity_deg_s = zeros(queryCount, 2);
sampledAcceleration_deg_s2 = zeros(queryCount, 2);

if knotCount == 1
    sampledPosition_deg(:, :) = unwrappedPosition_deg(1, :);
    sampledVelocity_deg_s(:, :) = velocity_deg_s(1, :);
    sampledAcceleration_deg_s2(:, :) = acceleration_deg_s2(1, :);
else
    for queryIndex = 1:queryCount
        segmentIndex = find(knotTime_s <= queryTime_s(queryIndex), ...
            1, "last");
        segmentIndex = min(segmentIndex, knotCount - 1);
        localTime_s = queryTime_s(queryIndex) - ...
            knotTime_s(segmentIndex);
        startState = makeKnotState(unwrappedPosition_deg, ...
            velocity_deg_s, acceleration_deg_s2, segmentIndex);
        endState = makeKnotState(unwrappedPosition_deg, velocity_deg_s, ...
            acceleration_deg_s2, segmentIndex + 1);
        duration_s = knotTime_s(segmentIndex + 1) - ...
            knotTime_s(segmentIndex);
        coefficients = quinticHermiteCoefficients( ...
            startState, endState, duration_s);
        sampledPosition_deg(queryIndex, :) = evaluatePolynomial( ...
            coefficients, localTime_s, 0);
        sampledVelocity_deg_s(queryIndex, :) = evaluatePolynomial( ...
            coefficients, localTime_s, 1);
        sampledAcceleration_deg_s2(queryIndex, :) = ...
            evaluatePolynomial(coefficients, localTime_s, 2);
    end
end

%% Section 3: Apply Display Wrapping & Assemble
displayPosition_deg = sampledPosition_deg;
if logical(command.azimuthWrap)
    displayPosition_deg(:, 1) = wrapAzimuthForDisplay( ...
        sampledPosition_deg(:, 1), command.azimuthDisplayRange_deg);
end
state = struct( ...
    "time_s", reshape(queryTime_s, queryShape), ...
    "position_deg", displayPosition_deg, ...
    "unwrappedPosition_deg", sampledPosition_deg, ...
    "velocity_deg_s", sampledVelocity_deg_s, ...
    "acceleration_deg_s2", sampledAcceleration_deg_s2);
end

function state = makeKnotState(position_deg, velocity_deg_s, ...
        acceleration_deg_s2, knotIndex)
%% Section 0: Header & Readme
% SYNTAX
%   state = makeKnotState(position_deg, velocity_deg_s, ...
%       acceleration_deg_s2, knotIndex)
%**************************************************************************
% PURPOSE
%   - Assemble one boundary state used by quintic interpolation.
%**************************************************************************
% INPUTS
%   - position_deg (N-by-2 numeric)
%   - velocity_deg_s (N-by-2 numeric)
%   - acceleration_deg_s2 (N-by-2 numeric)
%   - knotIndex (positive integer)
%**************************************************************************
% OUTPUTS
%   - state (scalar struct)
%**************************************************************************
% UNITS
%   - Angles are degrees and time derivatives use seconds.

state = struct( ...
    "position_deg", position_deg(knotIndex, :), ...
    "velocity_deg_s", velocity_deg_s(knotIndex, :), ...
    "acceleration_deg_s2", acceleration_deg_s2(knotIndex, :));
end

function value = evaluatePolynomial(coefficients, localTime_s, order)
%% Section 0: Header & Readme
% SYNTAX
%   value = evaluatePolynomial(coefficients, localTime_s, order)
%**************************************************************************
% PURPOSE
%   - Evaluate an ascending-power polynomial or its first two derivatives.
%**************************************************************************
% INPUTS
%   - coefficients (6-by-2 numeric)
%   - localTime_s (nonnegative scalar)
%   - order (0, 1, or 2)
%**************************************************************************
% OUTPUTS
%   - value (1-by-2 numeric)
%**************************************************************************
% UNITS
%   - Units follow derivative order from degrees and seconds.

workingCoefficients = coefficients;
for derivativeIndex = 1:order
    powers = (1:(size(workingCoefficients, 1) - 1)).';
    workingCoefficients = workingCoefficients(2:end, :) .* powers;
end
powers = localTime_s .^ (0:(size(workingCoefficients, 1) - 1));
value = powers * workingCoefficients;
end

function wrappedAzimuth_deg = wrapAzimuthForDisplay( ...
        unwrappedAzimuth_deg, displayRange_deg)
%% Section 0: Header & Readme
% SYNTAX
%   wrappedAzimuth_deg = wrapAzimuthForDisplay( ...
%       unwrappedAzimuth_deg, displayRange_deg)
%**************************************************************************
% PURPOSE
%   - Map continuous azimuth onto the configured display interval.
%**************************************************************************
% INPUTS
%   - unwrappedAzimuth_deg (numeric array)
%   - displayRange_deg (1-by-2 increasing numeric)
%**************************************************************************
% OUTPUTS
%   - wrappedAzimuth_deg (numeric array)
%**************************************************************************
% UNITS
%   - Azimuth is degrees.

minimumAzimuth_deg = displayRange_deg(1);
azimuthSpan_deg = diff(displayRange_deg);
wrappedAzimuth_deg = minimumAzimuth_deg + mod( ...
    unwrappedAzimuth_deg - minimumAzimuth_deg, azimuthSpan_deg);
end
