function [profileTime_s, xBySlice_units, yBySlice_units] = createSandboxPolygonMotionHistory(polygon_units, time_s, motionVector_units, motionProfile)
%% Section 0: Header & Readme
% SYNTAX
%   [profileTime_s, xBySlice_units, yBySlice_units] = ...
%       createSandboxPolygonMotionHistory( ...
%       polygon_units, time_s, motionVector_units, motionProfile)
%**************************************************************************
% PURPOSE
%   - Create sampled translations for one polygon drawn in the sandbox.
%**************************************************************************
% INPUTS
%   - polygon_units (N-by-2 finite numeric array; N >= 3)
%       Polygon vertices in [x y] order.
%   - time_s (increasing numeric vector with at least two values)
%       The first and last values define the motion interval.
%   - motionVector_units (1-by-2 finite numeric row)
%       Translation from the initial polygon to the arrow endpoint.
%   - motionProfile (scalar text)
%       "nonzeroVelocity", "zeroStart", "trapezoidal", "oscillating",
%       or "stationary".
%**************************************************************************
% OUTPUTS
%   - profileTime_s (M-by-1 numeric array)
%       Absolute sample times for the obstacle history.
%   - xBySlice_units, yBySlice_units (M-by-1 cell arrays)
%       Translated polygon coordinates at each sample time.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Validate Inputs

validateattributes(polygon_units, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2, 'nonempty'});
if size(polygon_units, 1) < 3
    error("createSandboxPolygonMotionHistory:TooFewVertices", "polygon_units must contain at least three vertices.");
end
validateattributes(time_s, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2, 'increasing'});
validateattributes(motionVector_units, {'numeric'}, {'real', 'finite', 'row', 'numel', 2});
motionProfile = string(motionProfile);
validProfiles = [ ...
    "nonzeroVelocity", "zeroStart", "trapezoidal", ...
    "oscillating", "stationary"];
if ~isscalar(motionProfile) || ~any(motionProfile == validProfiles)
    error("createSandboxPolygonMotionHistory:UnknownMotionProfile", "motionProfile must be one of: %s.", strjoin(validProfiles, ", "));
end
polygon_units      = double(polygon_units);
time_s           = double(time_s(:));
motionVector_units = reshape(double(motionVector_units), 1, 2);

%% Section 2: Create The Normalized Displacement

startTime_s = time_s(1);
endTime_s   = time_s(end);
if norm(motionVector_units) <= 1e-12 || motionProfile == "stationary"
    profileTime_s      = time_s;
    displacementFactor = zeros(numel(profileTime_s), 1);
else
    profileSampleCount = 21;
    profileTime_s      = linspace(startTime_s, endTime_s, profileSampleCount).';
    phase              = (profileTime_s - startTime_s) / (endTime_s - startTime_s);
    switch motionProfile
        case "nonzeroVelocity"
            % Constant velocity starts immediately and reaches the arrow end.
            displacementFactor = phase;
        case "zeroStart"
            % Constant acceleration starts at zero velocity. It continues for
            % the full interval, so the final velocity is not zero.
            displacementFactor = phase .^ 2;
        case "trapezoidal"
            % Accelerate, move at constant speed, and decelerate to rest.
            accelerationFraction   = 0.25;
            peakNormalizedVelocity = 1 / (1 - accelerationFraction);
            displacementFactor     = zeros(size(phase));
            accelerating           = phase < accelerationFraction;
            cruising               = phase >= accelerationFraction & phase <= 1 - accelerationFraction;
            decelerating           = phase > 1 - accelerationFraction;
            displacementFactor(accelerating) = 0.5 * peakNormalizedVelocity / accelerationFraction .* phase(accelerating) .^ 2;
            displacementFactor(cruising) = peakNormalizedVelocity .* (phase(cruising) - 0.5 * accelerationFraction);
            displacementFactor(decelerating) = 1 - 0.5 * peakNormalizedVelocity / accelerationFraction .* (1 - phase(decelerating)) .^ 2;
        case "oscillating"
            % Move to the arrow end and return to the initial position once.
            displacementFactor = 0.5 * (1 - cos(2 * pi * phase));
    end
end

%% Section 3: Translate The Polygon

sliceCount           = numel(profileTime_s);
xBySlice_units   = cell(sliceCount, 1);
yBySlice_units = cell(sliceCount, 1);
% Process each sample needed by the sandbox workflow.
for sampleIndex = 1:sliceCount
    offset_units       = displacementFactor(sampleIndex) * motionVector_units;
    movedPolygon_units = polygon_units + offset_units;
    xBySlice_units{sampleIndex} = movedPolygon_units(:, 1);
    yBySlice_units{sampleIndex} = movedPolygon_units(:, 2);
end
end
