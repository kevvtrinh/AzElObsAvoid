function [profileTime_s, azimuthBySlice_deg, elevationBySlice_deg] = ...
        createSandboxPolygonMotionHistory( ...
        polygon_deg, time_s, motionVector_deg, motionProfile)
%% Section 0: Header & Readme
% SYNTAX
%   [profileTime_s, azimuthBySlice_deg, elevationBySlice_deg] = ...
%       createSandboxPolygonMotionHistory( ...
%       polygon_deg, time_s, motionVector_deg, motionProfile)
%**************************************************************************
% PURPOSE
%   - Create sampled translations for one polygon drawn in the sandbox.
%**************************************************************************
% INPUTS
%   - polygon_deg (N-by-2 finite numeric array; N >= 3)
%       Polygon vertices in [azimuth elevation] order.
%   - time_s (increasing numeric vector with at least two values)
%       The first and last values define the motion interval.
%   - motionVector_deg (1-by-2 finite numeric row)
%       Translation from the initial polygon to the arrow endpoint.
%   - motionProfile (scalar text)
%       "nonzeroVelocity", "zeroStart", "trapezoidal", "oscillating",
%       or "stationary".
%**************************************************************************
% OUTPUTS
%   - profileTime_s (M-by-1 numeric array)
%       Absolute sample times for the obstacle history.
%   - azimuthBySlice_deg, elevationBySlice_deg (M-by-1 cell arrays)
%       Translated polygon coordinates at each sample time.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************

%% Section 1: Validate Inputs

validateattributes(polygon_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2, 'nonempty'});
if size(polygon_deg, 1) < 3
    error("createSandboxPolygonMotionHistory:TooFewVertices", ...
        "polygon_deg must contain at least three vertices.");
end
validateattributes(time_s, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2, 'increasing'});
validateattributes(motionVector_deg, {'numeric'}, ...
    {'real', 'finite', 'row', 'numel', 2});
motionProfile = string(motionProfile);
validProfiles = [ ...
    "nonzeroVelocity", "zeroStart", "trapezoidal", ...
    "oscillating", "stationary"];
if ~isscalar(motionProfile) || ~any(motionProfile == validProfiles)
    error("createSandboxPolygonMotionHistory:UnknownMotionProfile", ...
        "motionProfile must be one of: %s.", ...
        strjoin(validProfiles, ", "));
end
polygon_deg = double(polygon_deg);
time_s = double(time_s(:));
motionVector_deg = reshape(double(motionVector_deg), 1, 2);

%% Section 2: Create The Normalized Displacement

startTime_s = time_s(1);
endTime_s = time_s(end);
if norm(motionVector_deg) <= 1e-12 || motionProfile == "stationary"
    profileTime_s = time_s;
    displacementFactor = zeros(numel(profileTime_s), 1);
else
    profileSampleCount = 21;
    profileTime_s = linspace( ...
        startTime_s, endTime_s, profileSampleCount).';
    phase = (profileTime_s - startTime_s) / ...
        (endTime_s - startTime_s);
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
            accelerationFraction = 0.25;
            peakNormalizedVelocity = 1 / (1 - accelerationFraction);
            displacementFactor = zeros(size(phase));
            accelerating = phase < accelerationFraction;
            cruising = phase >= accelerationFraction & ...
                phase <= 1 - accelerationFraction;
            decelerating = phase > 1 - accelerationFraction;
            displacementFactor(accelerating) = 0.5 * ...
                peakNormalizedVelocity / accelerationFraction .* ...
                phase(accelerating) .^ 2;
            displacementFactor(cruising) = peakNormalizedVelocity .* ...
                (phase(cruising) - 0.5 * accelerationFraction);
            displacementFactor(decelerating) = 1 - 0.5 * ...
                peakNormalizedVelocity / accelerationFraction .* ...
                (1 - phase(decelerating)) .^ 2;
        case "oscillating"
            % Move to the arrow end and return to the initial position once.
            displacementFactor = 0.5 * (1 - cos(2 * pi * phase));
    end
end

%% Section 3: Translate The Polygon

sliceCount = numel(profileTime_s);
azimuthBySlice_deg = cell(sliceCount, 1);
elevationBySlice_deg = cell(sliceCount, 1);
for sampleIndex = 1:sliceCount
    offset_deg = displacementFactor(sampleIndex) * motionVector_deg;
    movedPolygon_deg = polygon_deg + offset_deg;
    azimuthBySlice_deg{sampleIndex} = movedPolygon_deg(:, 1);
    elevationBySlice_deg{sampleIndex} = movedPolygon_deg(:, 2);
end
end
