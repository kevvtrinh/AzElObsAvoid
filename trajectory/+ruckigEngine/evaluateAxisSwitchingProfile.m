function candidate = evaluateAxisSwitchingProfile( ...
        initialState, terminalState, limits, phaseDuration, phaseJerk, family)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = ruckigEngine.evaluateAxisSwitchingProfile( ...
%       initialState, terminalState, limits, phaseDuration, phaseJerk, family)
%**************************************************************************
% PURPOSE
%   - Integrate and certify one scalar piecewise-constant-jerk profile.
%**************************************************************************
% INPUTS
%   - initialState (scalar struct)
%       Scalar position, velocity, and acceleration fields.
%   - terminalState (scalar struct)
%       Scalar position, velocity, and acceleration fields.
%   - limits (scalar struct)
%       Positive maximumVelocity and maximumAcceleration fields.
%   - phaseDuration (1-by-N finite numeric vector)
%       Nonnegative phase durations.
%   - phaseJerk (1-by-N finite numeric vector)
%       Constant jerk applied during each corresponding phase.
%   - family (scalar text)
%       Diagnostic name for the switching family.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Success and exact phase-boundary position, velocity, acceleration,
%       duration, jerk, and family values.
%**************************************************************************
% UNITS
%   - Time and coordinate units are caller-defined and must be consistent.
%**************************************************************************

%% Section 1: Integrate The Phase Law

phaseDuration = phaseDuration(:).';
phaseJerk = phaseJerk(:).';
phaseCount = numel(phaseDuration);
candidate = createEmptyCandidate(phaseCount, family);
if numel(phaseJerk) ~= phaseCount || any(~isfinite(phaseDuration)) || ...
        any(~isfinite(phaseJerk)) || any(phaseDuration < -1e-10)
    return;
end
phaseDuration(abs(phaseDuration) < 1e-12) = 0;
position = zeros(1, phaseCount + 1);
velocity = zeros(1, phaseCount + 1);
acceleration = zeros(1, phaseCount + 1);
position(1) = initialState.position;
velocity(1) = initialState.velocity;
acceleration(1) = initialState.acceleration;

for phaseIndex = 1:phaseCount
    duration = phaseDuration(phaseIndex);
    jerk = phaseJerk(phaseIndex);
    acceleration(phaseIndex + 1) = ...
        acceleration(phaseIndex) + duration * jerk;
    velocity(phaseIndex + 1) = velocity(phaseIndex) + duration * ...
        (acceleration(phaseIndex) + duration * jerk / 2);
    position(phaseIndex + 1) = position(phaseIndex) + duration * ...
        (velocity(phaseIndex) + duration * ...
        (acceleration(phaseIndex) / 2 + duration * jerk / 6));
end
%% Section 2: Certify Endpoints And Continuous Limits

positionScale = max([1, abs(initialState.position), abs(terminalState.position)]);
velocityScale = max([1, limits.maximumVelocity, ...
    abs(initialState.velocity), abs(terminalState.velocity)]);
accelerationScale = max([1, limits.maximumAcceleration, ...
    abs(initialState.acceleration), abs(terminalState.acceleration)]);
endpointError = max([ ...
    abs(position(end) - terminalState.position) / positionScale, ...
    abs(velocity(end) - terminalState.velocity) / velocityScale, ...
    abs(acceleration(end) - terminalState.acceleration) / accelerationScale]);
velocityPeak = maximumAbsoluteVelocity( ...
    phaseDuration, phaseJerk, velocity, acceleration);
isFeasible = endpointError <= 5e-8 && ...
    max(abs(acceleration)) <= ...
    limits.maximumAcceleration + 5e-9 * accelerationScale && ...
    velocityPeak <= limits.maximumVelocity + 5e-9 * velocityScale;

candidate.Success = isFeasible;
candidate.PhaseDuration = phaseDuration;
candidate.PhaseJerk = phaseJerk;
candidate.Duration = sum(phaseDuration);
candidate.Position = position;
candidate.Velocity = velocity;
candidate.Acceleration = acceleration;
candidate.EndpointError = endpointError;
candidate.MaximumVelocity = velocityPeak;
candidate.MaximumAcceleration = max(abs(acceleration));
candidate.PathLength = continuousPathLength( ...
    phaseDuration, phaseJerk, position, velocity, acceleration);
end

%% Section 3: Local Functions

function peak = maximumAbsoluteVelocity( ...
        phaseDuration, phaseJerk, velocity, acceleration)
% Include interior velocity extrema where acceleration crosses zero.
peak = max(abs(velocity));
for phaseIndex = 1:numel(phaseDuration)
    jerk = phaseJerk(phaseIndex);
    if jerk == 0
        continue;
    end
    extremumTime = -acceleration(phaseIndex) / jerk;
    if extremumTime <= 0 || extremumTime >= phaseDuration(phaseIndex)
        continue;
    end
    extremumVelocity = velocity(phaseIndex) + extremumTime * ...
        (acceleration(phaseIndex) + extremumTime * jerk / 2);
    peak = max(peak, abs(extremumVelocity));
end
end

function lengthValue = continuousPathLength( ...
        phaseDuration, phaseJerk, position, velocity, acceleration)
% Split each phase at velocity zeros so scalar path length is exact.
lengthValue = 0;
for phaseIndex = 1:numel(phaseDuration)
    duration = phaseDuration(phaseIndex);
    jerk = phaseJerk(phaseIndex);
    zeroTimes = velocityZeroTimes( ...
        velocity(phaseIndex), acceleration(phaseIndex), jerk, duration);
    times = [0, zeroTimes, duration];
    phasePosition = position(phaseIndex) + times .* ...
        (velocity(phaseIndex) + times .* ...
        (acceleration(phaseIndex) / 2 + times * jerk / 6));
    lengthValue = lengthValue + sum(abs(diff(phasePosition)));
end
end

function zeroTimes = velocityZeroTimes(velocity, acceleration, jerk, duration)
% Return only strict interior roots of the quadratic velocity law.
if jerk == 0
    if acceleration == 0
        zeroTimes = zeros(1, 0);
    else
        zeroTimes = -velocity / acceleration;
    end
else
    discriminant = acceleration^2 - 2 * jerk * velocity;
    if discriminant < 0
        zeroTimes = zeros(1, 0);
    else
        root = sqrt(discriminant);
        zeroTimes = [(-acceleration - root) / jerk, ...
            (-acceleration + root) / jerk];
    end
end
zeroTimes = sort(unique(zeroTimes( ...
    zeroTimes > 1e-12 & zeroTimes < duration - 1e-12)));
end

function candidate = createEmptyCandidate(phaseCount, family)
% Define stable fields for either a valid profile or a rejected candidate.
candidate = struct( ...
    "Success", false, ...
    "PhaseDuration", zeros(1, phaseCount), ...
    "PhaseJerk", zeros(1, phaseCount), ...
    "Duration", NaN, ...
    "Position", zeros(1, phaseCount + 1), ...
    "Velocity", zeros(1, phaseCount + 1), ...
    "Acceleration", zeros(1, phaseCount + 1), ...
    "Family", string(family), ...
    "EndpointError", Inf, ...
    "MaximumVelocity", Inf, ...
    "MaximumAcceleration", Inf, ...
    "PathLength", Inf);
end
