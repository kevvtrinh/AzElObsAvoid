function environment = createAzElRpRetimerEnvironment(seed)
%% Section 0: Header & Readme
% SYNTAX
%   environment = createAzElRpRetimerEnvironment()
%   environment = createAzElRpRetimerEnvironment(seed)
%**************************************************************************
% PURPOSE
%   - Create a reproducible contextual RP environment that proposes one
%     unconstrained G3 corner-radius fraction.
%   - Keep obstacles and all acceptance decisions outside the learned policy.
%**************************************************************************
% INPUTS
%   - seed (nonnegative integer scalar, optional; default 41)
%       Seed for the deterministic sequence of synthetic corner problems.
%**************************************************************************
% OUTPUTS
%   - environment (rl.env.MATLABEnvironment)
%       One-step continuous-action environment with ten observations.
%**************************************************************************
% UNITS
%   - Observations, actions, and reward are dimensionless. Synthetic
%     geometry uses degrees and seconds before feature normalization.
%**************************************************************************

%% Section 1: Define The Contextual Policy Interface

if nargin < 1 || isempty(seed)
    seed = 41;
end
validateattributes(seed, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
observationInfo = rlNumericSpec([10 1], ...
    "LowerLimit", -10 * ones(10, 1), ...
    "UpperLimit", 10 * ones(10, 1));
observationInfo.Name = "unconstrained corner problem";
actionInfo = rlNumericSpec([1 1], "LowerLimit", -1, "UpperLimit", 1);
actionInfo.Name = "normalized corner-radius proposal";

%% Section 2: Build The Reproducible One-Step Environment

stream = RandStream("Threefry", "Seed", seed);
resetHandle = @() resetEpisode(stream);
environment = rlFunctionEnv( ...
    observationInfo, actionInfo, @stepEpisode, resetHandle);
end

%% Section 3: Local Functions

function [observation, loggedSignals] = resetEpisode(stream)
% Draw one general corner and compute its deterministic surrogate optimum.
incomingLength_deg = 10^(log10(0.5) + rand(stream) * log10(60));
outgoingLength_deg = 10^(log10(0.5) + rand(stream) * log10(60));
deflectionAngle_rad = 0.15 + rand(stream) * (pi - 0.3);
incomingHeading_rad = -pi + 2 * pi * rand(stream);
turnSign = 2 * (rand(stream) >= 0.5) - 1;
outgoingHeading_rad = incomingHeading_rad + ...
    turnSign * deflectionAngle_rad;
incoming = [cos(incomingHeading_rad), sin(incomingHeading_rad)];
outgoing = [cos(outgoingHeading_rad), sin(outgoingHeading_rad)];
velocityLimit_deg_s = 10 .^ ( ...
    log10(0.4) + rand(stream, 1, 2) * log10(20));
accelerationLimit_deg_s2 = 10 .^ ( ...
    log10(0.2) + rand(stream, 1, 2) * log10(50));
jerkLimit_deg_s3 = 10 .^ ( ...
    log10(0.5) + rand(stream, 1, 2) * log10(200));
turnRadius_deg = 10^(log10(0.05) + rand(stream) * log10(100));
problem = struct( ...
    "DeflectionAngle_rad", deflectionAngle_rad, ...
    "IncomingLength_deg", incomingLength_deg, ...
    "OutgoingLength_deg", outgoingLength_deg, ...
    "IncomingDirection", incoming, ...
    "OutgoingDirection", outgoing, ...
    "TurnRadius_deg", turnRadius_deg, ...
    "MaxVelocity_deg_s", velocityLimit_deg_s, ...
    "MaxAcceleration_deg_s2", accelerationLimit_deg_s2, ...
    "MaxJerk_deg_s3", jerkLimit_deg_s3);
observation = azElInternal.rpRetimerObservation(problem);
radiusFraction = linspace(0, 1, 101).';
estimatedTime_s = zeros(size(radiusFraction));
for candidateIndex = 1:numel(radiusFraction)
    estimatedTime_s(candidateIndex) = estimateTraversalTime( ...
        problem, radiusFraction(candidateIndex));
end
[minimumTime_s, minimumIndex] = min(estimatedTime_s);
loggedSignals = struct( ...
    "Problem", problem, ...
    "MinimumTime_s", minimumTime_s, ...
    "BestRadiusFraction", radiusFraction(minimumIndex));
end

function [nextObservation, reward, isDone, loggedSignals] = ...
        stepEpisode(action, loggedSignals)
% Reward a fast radius proposal and leave every safety decision to the planner.
action = min(1, max(-1, double(action)));
radiusFraction = 0.5 * (action + 1);
estimatedTime_s = estimateTraversalTime( ...
    loggedSignals.Problem, radiusFraction);
timeRatio = estimatedTime_s / loggedSignals.MinimumTime_s;
fractionError = radiusFraction - loggedSignals.BestRadiusFraction;
reward = -timeRatio - 4 * fractionError^2;
nextObservation = zeros(10, 1);
isDone = true;
end

function time_s = estimateTraversalTime(problem, radiusFraction)
% Estimate time with curvature, directional velocity, and acceleration limits.
incomingLength_deg = problem.IncomingLength_deg;
outgoingLength_deg = problem.OutgoingLength_deg;
incoming = problem.IncomingDirection;
outgoing = problem.OutgoingDirection;
incomingSpeed_deg_s = directionalLimit( ...
    incoming, problem.MaxVelocity_deg_s);
outgoingSpeed_deg_s = directionalLimit( ...
    outgoing, problem.MaxVelocity_deg_s);
incomingAcceleration_deg_s2 = directionalLimit( ...
    incoming, problem.MaxAcceleration_deg_s2);
outgoingAcceleration_deg_s2 = directionalLimit( ...
    outgoing, problem.MaxAcceleration_deg_s2);
if radiusFraction <= 1e-12
    time_s = minimumRestToRestTime( ...
        incomingLength_deg, incomingSpeed_deg_s, ...
        incomingAcceleration_deg_s2) + ...
        minimumRestToRestTime( ...
        outgoingLength_deg, outgoingSpeed_deg_s, ...
        outgoingAcceleration_deg_s2);
    return;
end
angle_rad = problem.DeflectionAngle_rad;
tangentScale = (384 / 125) * sin(angle_rad / 2) / ...
    cos(angle_rad / 2)^2;
geometricMaximumRadius_deg = 0.45 * ...
    min(incomingLength_deg, outgoingLength_deg) / tangentScale;
radius_deg = radiusFraction * min( ...
    problem.TurnRadius_deg, geometricMaximumRadius_deg);
trim_deg = radius_deg * tangentScale;
controlPoints_deg = [ ...
    -trim_deg * incoming; ...
    -0.5 * trim_deg * incoming; ...
    0 0; ...
    0 0; ...
    0.5 * trim_deg * outgoing; ...
    trim_deg * outgoing];
parameter = linspace(0, 1, 101).';
[position_deg, firstDerivative, secondDerivative] = ...
    evaluateQuintic(controlPoints_deg, parameter);
parameterSpeed_deg = vecnorm(firstDerivative, 2, 2);
curveLength_deg = sum(vecnorm(diff(position_deg), 2, 2));
tangent = firstDerivative ./ parameterSpeed_deg;
crossProduct_deg2 = firstDerivative(:, 1) .* ...
    secondDerivative(:, 2) - firstDerivative(:, 2) .* ...
    secondDerivative(:, 1);
curvature_deg_inv = abs(crossProduct_deg2) ./ parameterSpeed_deg.^3;
curveSpeedLimit_deg_s = Inf(size(parameter));
for sampleIndex = 1:numel(parameter)
    velocityCap_deg_s = directionalLimit( ...
        tangent(sampleIndex, :), problem.MaxVelocity_deg_s);
    normal = [-tangent(sampleIndex, 2), tangent(sampleIndex, 1)];
    normalAcceleration_deg_s2 = directionalLimit( ...
        normal, problem.MaxAcceleration_deg_s2);
    curvatureCap_deg_s = Inf;
    if curvature_deg_inv(sampleIndex) > 1e-12
        curvatureCap_deg_s = sqrt(normalAcceleration_deg_s2 / ...
            curvature_deg_inv(sampleIndex));
    end
    curveSpeedLimit_deg_s(sampleIndex) = min( ...
        velocityCap_deg_s, curvatureCap_deg_s);
end
curveTime_s = curveLength_deg / max( ...
    min(curveSpeedLimit_deg_s), eps);
incomingTime_s = minimumRestToRestTime( ...
    max(0, incomingLength_deg - trim_deg), incomingSpeed_deg_s, ...
    incomingAcceleration_deg_s2);
outgoingTime_s = minimumRestToRestTime( ...
    max(0, outgoingLength_deg - trim_deg), outgoingSpeed_deg_s, ...
    outgoingAcceleration_deg_s2);
% Carrying speed through the curve replaces one full stop. Half of each
% straight rest-to-rest time is retained as a conservative bandit target.
time_s = 0.5 * incomingTime_s + curveTime_s + 0.5 * outgoingTime_s;
end

function limit = directionalLimit(direction, axisLimit)
% Convert per-axis bounds to one scalar bound along a unit direction.
activeAxis = abs(direction) > 1e-12;
limit = min(axisLimit(activeAxis) ./ abs(direction(activeAxis)));
end

function duration_s = minimumRestToRestTime( ...
        length_deg, speedLimit_deg_s, accelerationLimit_deg_s2)
% Return the scalar trapezoidal or triangular rest-to-rest duration.
if length_deg <= 0
    duration_s = 0;
    return;
end
accelerationDistance_deg = speedLimit_deg_s^2 / ...
    accelerationLimit_deg_s2;
if length_deg <= accelerationDistance_deg
    duration_s = 2 * sqrt(length_deg / accelerationLimit_deg_s2);
else
    duration_s = 2 * speedLimit_deg_s / accelerationLimit_deg_s2 + ...
        (length_deg - accelerationDistance_deg) / speedLimit_deg_s;
end
end

function [position_deg, firstDerivative, secondDerivative] = ...
        evaluateQuintic(controlPoints_deg, parameter)
% Evaluate the fixed quintic family used by the production proposal search.
oneMinus = 1 - parameter;
position_deg = [oneMinus.^5, 5 * oneMinus.^4 .* parameter, ...
    10 * oneMinus.^3 .* parameter.^2, ...
    10 * oneMinus.^2 .* parameter.^3, ...
    5 * oneMinus .* parameter.^4, parameter.^5] * controlPoints_deg;
firstDerivative = [oneMinus.^4, 4 * oneMinus.^3 .* parameter, ...
    6 * oneMinus.^2 .* parameter.^2, ...
    4 * oneMinus .* parameter.^3, parameter.^4] * ...
    (5 * diff(controlPoints_deg, 1, 1));
secondDerivative = [oneMinus.^3, 3 * oneMinus.^2 .* parameter, ...
    3 * oneMinus .* parameter.^2, parameter.^3] * ...
    (20 * diff(controlPoints_deg, 2, 1));
end
