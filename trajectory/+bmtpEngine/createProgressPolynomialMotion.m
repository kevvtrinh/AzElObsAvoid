function candidate = createProgressPolynomialMotion( ...
        baseMotion, initialState, goalState, progressAxisIndex, ...
        amplitude_deg, sampleStep_s, seedSource, basisPower)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.createProgressPolynomialMotion( ...
%       baseMotion, initialState, goalState, progressAxisIndex, ...
%       amplitude_deg, sampleStep_s, seedSource)
%   candidate = bmtpEngine.createProgressPolynomialMotion( ...
%       baseMotion, initialState, goalState, progressAxisIndex, ...
%       amplitude_deg, sampleStep_s, seedSource, basisPower)
%**************************************************************************
% PURPOSE
%   - Compose a bounded lateral polynomial in normalized progress with one
%     exact straight-progress motion without changing its physical clock.
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
%   - basisPower (finite numeric row vector, optional)
%       Ascending powers of the dimensionless lateral basis. The default is
%       the endpoint-flat alternating degree-five basis. Both endpoint values
%       must be zero so the requested endpoint positions remain unchanged.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar trajectory-engine result struct)
%       Exact polynomial composition on the base switching intervals.
%**************************************************************************
% UNITS
%   - Position and amplitude are degrees; time is seconds; derivatives use
%     deg/s, deg/s^2, and deg/s^3. Histories are N-by-2.
%**************************************************************************

%% Section 1: Validate The Straight-Progress Clock

if nargin < 8 || isempty(basisPower)
    basisPower = [0, 4, 4, -56, 80, -32];
end
requiredFields = {'Success', 'UsedStraightProgress', 'Polynomial'};
stateFields = {'position_deg', 'velocity_deg_s', 'acceleration_deg_s2'};
if (nargin ~= 7 && nargin ~= 8) || ...
        ~isstruct(baseMotion) || ~isscalar(baseMotion) || ...
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
basisPower = double(basisPower(:).');
lastBasisPower = find(basisPower ~= 0, 1, "last");
if isempty(lastBasisPower)
    basisPower = 0;
else
    basisPower = basisPower(1:lastBasisPower);
end
if any(~isfinite(basisPower)) || numel(basisPower) < 2
    error("createProgressPolynomialMotion:InvalidBasis", ...
        "basisPower must be a finite nonconstant numeric vector.");
end
endpointTolerance = 256 * eps(max(1, sum(abs(basisPower))));
if abs(basisPower(1)) > endpointTolerance || ...
        abs(sum(basisPower)) > endpointTolerance
    error("createProgressPolynomialMotion:NonzeroBasisEndpoint", ...
        "The lateral basis must evaluate to zero at progress zero and one.");
end
lateralAxisIndex = 3 - progressAxisIndex;
progressDelta_deg = goalPosition_deg(progressAxisIndex) - ...
    initialPosition_deg(progressAxisIndex);
if progressDelta_deg == 0
    error("createProgressPolynomialMotion:ZeroProgress", ...
        "The selected progress coordinate must have nonzero displacement.");
end

%% Section 2: Compose The Progress-Polynomial Path

direct = baseMotion.Polynomial;
segmentCount = direct.SegmentCount;
directDegree = size(direct.positionPower_deg, 3) - 1;
basisDegree = numel(basisPower) - 1;
coefficientCount = max(directDegree + 1, directDegree * basisDegree + 1);
positionPower_deg = zeros(segmentCount, 2, coefficientCount);
for segmentIndex = 1:segmentCount
    directPower_deg = reshape( ...
        direct.positionPower_deg(segmentIndex, :, :), 2, []);
    progressPower = directPower_deg(progressAxisIndex, :) / progressDelta_deg;
    progressPower(1) = progressPower(1) - ...
        initialPosition_deg(progressAxisIndex) / progressDelta_deg;
    progressPower = trimPower(progressPower);
    lateralBasisPower = composePower( ...
        basisPower, progressPower, coefficientCount);
    lateralPower = zeros(1, coefficientCount);
    lateralPower(1) = initialPosition_deg(lateralAxisIndex);
    lateralPower = addPower(lateralPower, ...
        goalPosition_deg(lateralAxisIndex) - ...
        initialPosition_deg(lateralAxisIndex), progressPower);
    lateralPower = lateralPower + double(amplitude_deg) * lateralBasisPower;
    progressOutput = zeros(1, coefficientCount);
    progressOutput(1:size(directPower_deg, 2)) = ...
        directPower_deg(progressAxisIndex, :);
    positionPower_deg(segmentIndex, progressAxisIndex, :) = progressOutput;
    positionPower_deg(segmentIndex, lateralAxisIndex, :) = lateralPower;
end
durationScale_s = reshape(double(direct.SegmentDuration_s(:)), [], 1, 1);
velocityPower_deg_s = positionPower_deg(:, :, 2:end) .* ...
    reshape(1:coefficientCount - 1, 1, 1, []) ./ durationScale_s;
accelerationPower_deg_s2 = velocityPower_deg_s(:, :, 2:end) .* ...
    reshape(1:coefficientCount - 2, 1, 1, []) ./ durationScale_s;
jerkPower_deg_s3 = accelerationPower_deg_s2(:, :, 2:end) .* ...
    reshape(1:coefficientCount - 3, 1, 1, []) ./ durationScale_s;
polynomial = struct( ...
    "Degree", coefficientCount - 1, "SegmentCount", segmentCount, ...
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

function composition = composePower(outerPower, innerPower, outputCount)
% Compose ascending-power polynomials without optional symbolic functions.
composition = zeros(1, outputCount);
innerProduct = 1;
for outerIndex = 1:numel(outerPower)
    if outerPower(outerIndex) ~= 0
        composition(1:numel(innerProduct)) = ...
            composition(1:numel(innerProduct)) + ...
            outerPower(outerIndex) * innerProduct;
    end
    if outerIndex < numel(outerPower)
        innerProduct = conv(innerProduct, innerPower);
    end
end
end
