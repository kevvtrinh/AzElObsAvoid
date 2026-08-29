function candidate = createProgressPolynomialMotion( ...
        baseMotion, initialState, goalState, progressAxisIndex, ...
        amplitude_deg, sampleStep_s, seedSource)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.createProgressPolynomialMotion( ...
%       baseMotion, initialState, goalState, progressAxisIndex, ...
%       amplitude_deg, sampleStep_s, seedSource)
%**************************************************************************
% PURPOSE
%   - Compose an endpoint-flat alternating lateral path with one exact
%     straight-progress motion without changing its physical clock.
%**************************************************************************
% INPUTS
%   - baseMotion (scalar trajectory-engine result struct)
%       Requires a successful straight-progress Polynomial.
%   - initialState, goalState (scalar state structs)
%       Require matching one-by-two positions and rest endpoint states.
%   - progressAxisIndex (integer scalar)
%       Coordinate whose monotone direct motion defines normalized progress.
%   - amplitude_deg (finite scalar)
%       Signed coefficient of the fixed endpoint-flat path basis.
%   - sampleStep_s (positive scalar)
%       Requested output-history spacing.
%   - seedSource (scalar text)
%       Input-derived construction label copied to the motion record.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar trajectory-engine result struct)
%       Exact degree-15 composition on the base switching intervals.
%**************************************************************************
% UNITS
%   - Position and amplitude are degrees; time is seconds; derivatives use
%     deg/s, deg/s^2, and deg/s^3. Histories are N-by-2.
%**************************************************************************

%% Section 1: Validate The Straight-Progress Clock

requiredFields = {'Success', 'UsedStraightProgress', 'Polynomial'};
stateFields = {'position_deg', 'velocity_deg_s', 'acceleration_deg_s2'};
if nargin ~= 7 || ~isstruct(baseMotion) || ~isscalar(baseMotion) || ...
        ~all(isfield(baseMotion, requiredFields)) || ...
        ~baseMotion.Success || ~baseMotion.UsedStraightProgress || ...
        ~isstruct(initialState) || ~isscalar(initialState) || ...
        ~isstruct(goalState) || ~isscalar(goalState) || ...
        ~all(isfield(initialState, stateFields)) || ...
        ~all(isfield(goalState, stateFields))
    error("createProgressPolynomialMotion:InvalidMotion", ...
        "A successful two-axis straight-progress rest motion is required.");
end
initialPosition_deg = double(initialState.position_deg(:).');
goalPosition_deg = double(goalState.position_deg(:).');
if numel(initialPosition_deg) ~= 2 || numel(goalPosition_deg) ~= 2 || ...
        max(abs([initialState.velocity_deg_s, goalState.velocity_deg_s, ...
        initialState.acceleration_deg_s2, goalState.acceleration_deg_s2])) > 1e-12
    error("createProgressPolynomialMotion:UnsupportedState", ...
        "The progress composition requires two-dimensional rest endpoints.");
end
validateattributes(progressAxisIndex, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 1, '<=', 2});
validateattributes(amplitude_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(sampleStep_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
seedSource = string(seedSource);
if ~isscalar(seedSource)
    error("createProgressPolynomialMotion:InvalidSource", ...
        "seedSource must be scalar text.");
end
lateralAxisIndex = 3 - progressAxisIndex;
progressDelta_deg = goalPosition_deg(progressAxisIndex) - ...
    initialPosition_deg(progressAxisIndex);
if progressDelta_deg == 0
    error("createProgressPolynomialMotion:ZeroProgress", ...
        "The selected progress coordinate must have nonzero displacement.");
end

%% Section 2: Compose The Degree-Fifteen Path

direct = baseMotion.Polynomial;
segmentCount = direct.SegmentCount;
coefficientCount = 16;
positionPower_deg = zeros(segmentCount, 2, coefficientCount);
for segmentIndex = 1:segmentCount
    directPower_deg = reshape( ...
        direct.positionPower_deg(segmentIndex, :, :), 2, []);
    progressPower = directPower_deg(progressAxisIndex, :) / progressDelta_deg;
    progressPower(1) = progressPower(1) - ...
        initialPosition_deg(progressAxisIndex) / progressDelta_deg;
    progressPower = trimPower(progressPower);
    progressSquared = conv(progressPower, progressPower);
    progressCubed = conv(progressSquared, progressPower);
    progressFourth = conv(progressCubed, progressPower);
    progressFifth = conv(progressFourth, progressPower);
    alternatingPower = zeros(1, coefficientCount);
    alternatingPower = addPower(alternatingPower, 4, progressPower);
    alternatingPower = addPower(alternatingPower, 4, progressSquared);
    alternatingPower = addPower(alternatingPower, -56, progressCubed);
    alternatingPower = addPower(alternatingPower, 80, progressFourth);
    alternatingPower = addPower(alternatingPower, -32, progressFifth);
    lateralPower = zeros(1, coefficientCount);
    lateralPower(1) = initialPosition_deg(lateralAxisIndex);
    lateralPower = addPower(lateralPower, ...
        goalPosition_deg(lateralAxisIndex) - ...
        initialPosition_deg(lateralAxisIndex), progressPower);
    lateralPower = lateralPower + double(amplitude_deg) * alternatingPower;
    progressOutput = zeros(1, coefficientCount);
    progressOutput(1:size(directPower_deg, 2)) = ...
        directPower_deg(progressAxisIndex, :);
    positionPower_deg(segmentIndex, progressAxisIndex, :) = progressOutput;
    positionPower_deg(segmentIndex, lateralAxisIndex, :) = lateralPower;
end
durationScale_s = reshape(double(direct.SegmentDuration_s(:)), [], 1, 1);
velocityPower_deg_s = positionPower_deg(:, :, 2:end) .* ...
    reshape(1:15, 1, 1, []) ./ durationScale_s;
accelerationPower_deg_s2 = velocityPower_deg_s(:, :, 2:end) .* ...
    reshape(1:14, 1, 1, []) ./ durationScale_s;
jerkPower_deg_s3 = accelerationPower_deg_s2(:, :, 2:end) .* ...
    reshape(1:13, 1, 1, []) ./ durationScale_s;
polynomial = struct( ...
    "Degree", 15, "SegmentCount", segmentCount, ...
    "SegmentStartTime_s", direct.SegmentStartTime_s, ...
    "SegmentDuration_s", direct.SegmentDuration_s, ...
    "SegmentBreakTau", direct.SegmentBreakTau, ...
    "FinalTime_s", direct.FinalTime_s, ...
    "positionPower_deg", positionPower_deg, ...
    "velocityPower_deg_s", velocityPower_deg_s, ...
    "accelerationPower_deg_s2", accelerationPower_deg_s2, ...
    "jerkPower_deg_s3", jerkPower_deg_s3, ...
    "TerminalState", struct( ...
        "position_deg", goalPosition_deg, ...
        "velocity_deg_s", double(goalState.velocity_deg_s(:).'), ...
        "acceleration_deg_s2", ...
        double(goalState.acceleration_deg_s2(:).')));
candidate = bmtpEngine.createMotionRecord( ...
    baseMotion, initialState, polynomial, [], sampleStep_s, seedSource);
end

%% Section 3: Local Functions

function power = addPower(power, scale, addend)
% Add one ascending-power polynomial after padding its coefficient vector.
power(1:numel(addend)) = power(1:numel(addend)) + scale * addend;
end

function power = trimPower(power)
% Remove only exact trailing zeros so convolution does not inflate degree.
lastIndex = find(power ~= 0, 1, "last");
if isempty(lastIndex)
    power = 0;
else
    power = power(1:lastIndex);
end
end
