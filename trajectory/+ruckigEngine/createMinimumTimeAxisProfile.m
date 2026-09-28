function [profile, candidates] = createMinimumTimeAxisProfile(initialState, terminalState, limits)
%% Section 0: Header & Readme
% SYNTAX
%   profile = ruckigEngine.createMinimumTimeAxisProfile( ...
%       initialState, terminalState, limits)
%   [profile, candidates] = ruckigEngine.createMinimumTimeAxisProfile( ...
%       initialState, terminalState, limits)
% PURPOSE
%   - Create the shortest one-dimensional position trajectory admitted by
%     symmetric velocity, acceleration, and jerk limits.
% INPUTS
%   - initialState (scalar struct)
%       Scalar time, position, velocity, and acceleration fields.
%   - terminalState (scalar struct)
%       Scalar position, velocity, and acceleration fields.
%   - limits (scalar struct)
%       Positive scalar maximumVelocity, maximumAcceleration, and
%       maximumJerk fields.
% OUTPUTS
%   - profile (scalar struct)
%       Success, seven phase durations and jerks, minimum duration, and the
%       exact phase-boundary state histories. Unsupported numerical edge
%       cases return Success = false so the caller receives an identified unsupported result.
%   - candidates (structure array)
%       Every certified extremal profile used to derive synchronization
%       block intervals, including duration and signed initial direction.
% UNITS
%   - Time and coordinate units are caller-defined and must be consistent.

% The switching equations are adapted from Ruckig v0.19.4 under its MIT
% license; see trajectory/THIRD_PARTY_NOTICES.txt.

%% Section 1: Create Candidate Switching Profiles

context             = createContext(initialState, terminalState, limits);
candidates          = repmat(createEmptyCandidate(), 0, 1);
stateScale          = max(1, max(abs([context.p0, context.pf, context.v0, context.vf, context.a0, context.af])));
stationaryTolerance = 256 * eps(stateScale);
isStationary        = abs(context.displacement) <= stationaryTolerance && max(abs([context.v0, context.vf, context.a0, context.af])) <= stationaryTolerance;
if isStationary
    % An unchanged axis needs zero motion time and can wait for the other axes.
    candidate = createEmptyCandidate();
    candidate.Position(:) = context.p0;
    candidate.Velocity(:) = 0;
    candidate.Acceleration(:) = 0;
    candidate.Family     = "stationary";
    candidate.PathLength = 0;
    candidate.Duration   = 0;
    candidates = candidate;
    profile    = createEmptyProfile();
    profile.Success       = true;
    profile.Message       = "The axis is stationary.";
    profile.PhaseDuration = candidate.PhaseDuration;
    profile.PhaseJerk     = candidate.PhaseJerk;
    profile.Duration      = 0;
    profile.FinalTime     = initialState.time;
    profile.Position      = candidate.Position;
    profile.Velocity      = candidate.Velocity;
    profile.Acceleration  = candidate.Acceleration;
    profile.Family        = candidate.Family;
    profile.PathLength    = 0;
    return;
end
% Port PositionThirdOrderStep1::get_profile. The public input checks reject
% zero derivative limits, so Ruckig's time_all_single_step zero-limit branch
% is unreachable in this engine and is deliberately not duplicated here.
targetIsSettled = abs(context.vf) < eps && abs(context.af) < eps;
if targetIsSettled
    if context.displacement >= 0
        firstDirection = 1;
    else
        firstDirection = -1;
    end
    directions = [firstDirection, -firstDirection];
    for direction = directions
        directedLimits = createDirectedLimits(context, direction);
        candidates     = appendVelocityProfiles(candidates, context, directedLimits);
        if ~isempty(candidates)
            break;
        end
        candidates = appendUnconstrainedProfiles(candidates, context, directedLimits);
        if ~isempty(candidates)
            break;
        end
        candidates = appendAccelerationProfiles(candidates, context, directedLimits);
        if ~isempty(candidates)
            break;
        end
    end
else
    directedPositive = createDirectedLimits(context, 1);
    directedNegative = createDirectedLimits(context, -1);
    candidates = appendUnconstrainedProfiles(candidates, context, directedPositive);
    candidates = appendUnconstrainedProfiles(candidates, context, directedNegative);
    candidates = appendAccelerationProfiles(candidates, context, directedPositive);
    candidates = appendAccelerationProfiles(candidates, context, directedNegative);
    candidates = appendVelocityProfiles(candidates, context, directedPositive);
    candidates = appendVelocityProfiles(candidates, context, directedNegative);
end

% The official two-step cases are true fallbacks. They are not retries for a
% rejected full profile; they represent lower-phase-count boundary solutions.
if isempty(candidates)
    for direction = [1, -1]
        candidates = appendNoneTwoStep(candidates, context, ...
            createDirectedLimits(context, direction));
        if ~isempty(candidates)
            break;
        end
    end
end
if isempty(candidates)
    for direction = [1, -1]
        candidates = appendInitialAccelerationTwoStep(candidates, context, ...
            createDirectedLimits(context, direction));
        if ~isempty(candidates)
            break;
        end
    end
end
if isempty(candidates)
    for direction = [1, -1]
        candidates = appendVelocityTwoStep(candidates, context, ...
            createDirectedLimits(context, direction));
        if ~isempty(candidates)
            break;
        end
    end
end
if isempty(candidates)
    for direction = [1, -1]
        candidates = appendTerminalAccelerationVelocityTwoStep(candidates, ...
            context, createDirectedLimits(context, direction));
        if ~isempty(candidates)
            break;
        end
    end
end
%% Section 2: Select The Shortest Valid Profile

profile = createEmptyProfile();
% Return the stable failure when no exact switching family works; otherwise rank feasible families by duration and path length.
if isempty(candidates)
    profile.Message = "No exact jerk-switching family satisfied the boundary states.";
    return;
end

durations = zeros(numel(candidates), 1);
% Evaluate each candidate before retaining the best admissible candidate.
for candidateIndex = 1:numel(candidates)
    durations(candidateIndex) = sum(candidates(candidateIndex).PhaseDuration);
end
[minimumDuration, selectedIndex] = min(durations);
selected = candidates(selectedIndex);
profile.Success       = true;
profile.Message       = "An exact minimum-time axis profile was created.";
profile.PhaseDuration = selected.PhaseDuration;
profile.PhaseJerk     = selected.PhaseJerk;
profile.Duration      = minimumDuration;
profile.FinalTime     = initialState.time + minimumDuration;
profile.Position      = selected.Position;
profile.Velocity      = selected.Velocity;
profile.Acceleration  = selected.Acceleration;
profile.Family        = selected.Family;
profile.PathLength    = selected.PathLength;
end

%% Section 3: Local Functions

function context = createContext(initialState, terminalState, limits)
    % Cache the scalar boundary powers used by several switching families.
    context = struct("p0", initialState.position, ...
        "pf", terminalState.position, ...
        "v0", initialState.velocity, ...
        "vf", terminalState.velocity, ...
        "a0", initialState.acceleration, ...
        "af", terminalState.acceleration, ...
        "vMaximum", limits.maximumVelocity, ...
        "aMaximum", limits.maximumAcceleration, ...
        "jMaximum", limits.maximumJerk);
    context.displacement = context.pf - context.p0;
    context.v0Squared    = context.v0^2;
    context.vfSquared    = context.vf^2;
    context.a0Squared    = context.a0^2;
    context.afSquared    = context.af^2;
    context.a0Cubed      = context.a0^3;
    context.afCubed      = context.af^3;
    context.a0Fourth     = context.a0^4;
    context.afFourth     = context.af^4;
end

function directed = createDirectedLimits(context, direction)
    % Reverse all signed bounds to reuse the equations in the opposite direction.
    directed = struct("vMaximum", direction * context.vMaximum, ...
        "vMinimum", -direction * context.vMaximum, ...
        "aMaximum", direction * context.aMaximum, ...
        "aMinimum", -direction * context.aMaximum, ...
        "jMaximum", direction * context.jMaximum);
end

function candidates = appendVelocityProfiles(candidates, context, limits)
    % Add the four families whose middle phase reaches the velocity bound.
    vMaximum = limits.vMaximum;
    aMaximum = limits.aMaximum;
    aMinimum = limits.aMinimum;
    jMaximum = limits.jMaximum;
    jSquared = jMaximum^2;

    phase = zeros(1, 7);
    phase(1) = (-context.a0 + aMaximum) / jMaximum;
    phase(2) = (context.a0Squared / 2 - aMaximum^2 - jMaximum * (context.v0 - vMaximum)) / (aMaximum * jMaximum);
    phase(3) = aMaximum / jMaximum;
    phase(4) = (3 * (context.a0Fourth * aMinimum - context.afFourth * aMaximum) + 8 * aMaximum * aMinimum * (context.afCubed - context.a0Cubed + 3 * jMaximum * (context.a0 * context.v0 - context.af * context.vf)) + 6 * context.a0Squared * aMinimum * (aMaximum^2 - 2 * jMaximum * context.v0) - 6 * context.afSquared * aMaximum * (aMinimum^2 - 2 * jMaximum * context.vf) - 12 * jMaximum * (aMaximum * aMinimum * (aMaximum * (context.v0 + vMaximum) - aMinimum * (context.vf + vMaximum) - 2 * jMaximum * context.displacement) + (aMinimum - aMaximum) * jMaximum * vMaximum^2 + jMaximum * (aMaximum * context.vfSquared - aMinimum * context.v0Squared))) / (24 * aMaximum * aMinimum * jSquared * vMaximum);
    phase(5) = -aMinimum / jMaximum;
    phase(6) = -(context.afSquared / 2 - aMinimum^2 - jMaximum * (context.vf - vMaximum)) / (aMinimum * jMaximum);
    phase(7) = phase(5) + context.af / jMaximum;
    candidates = appendCandidate(candidates, context, limits, phase, "accelerationVelocity");

    timeAcceleration0 = ruckigEngine.internal.safeSqrt(context.a0Squared / (2 * jSquared) + (vMaximum - context.v0) / jMaximum);
    timeAcceleration1 = ruckigEngine.internal.safeSqrt(context.afSquared / (2 * jSquared) + (vMaximum - context.vf) / jMaximum);
    if isfinite(timeAcceleration0)
        phase(1:3) = [timeAcceleration0 - context.a0 / jMaximum, ...
            0, timeAcceleration0];
        phase(4) = -(3 * context.afFourth - 8 * aMinimum * (context.afCubed - context.a0Cubed) - 24 * aMinimum * jMaximum * (context.a0 * context.v0 - context.af * context.vf) + 6 * context.afSquared * (aMinimum^2 - 2 * jMaximum * context.vf) - 12 * jMaximum * (2 * aMinimum * jMaximum * context.displacement + aMinimum^2 * (context.vf + vMaximum) + jMaximum * (vMaximum^2 - context.vfSquared) + aMinimum * timeAcceleration0 * (context.a0Squared - 2 * jMaximum * (context.v0 + vMaximum)))) / (24 * aMinimum * jSquared * vMaximum);
        candidates = appendCandidate(candidates, context, limits, phase, "terminalAccelerationVelocity");
    end

    if isfinite(timeAcceleration1)
        phase(1) = (-context.a0 + aMaximum) / jMaximum;
        phase(2) = (context.a0Squared / 2 - aMaximum^2 - jMaximum * (context.v0 - vMaximum)) / (aMaximum * jMaximum);
        phase(3) = aMaximum / jMaximum;
        phase(4) = (3 * context.a0Fourth + 8 * aMaximum * (context.afCubed - context.a0Cubed) + 24 * aMaximum * jMaximum * (context.a0 * context.v0 - context.af * context.vf) + 6 * context.a0Squared * (aMaximum^2 - 2 * jMaximum * context.v0) - 12 * jMaximum * (-2 * aMaximum * jMaximum * context.displacement + aMaximum^2 * (context.v0 + vMaximum) + jMaximum * (vMaximum^2 - context.v0Squared) + aMaximum * timeAcceleration1 * (-context.afSquared + 2 * (context.vf + vMaximum) * jMaximum))) / (24 * aMaximum * jSquared * vMaximum);
        phase(5:7) = [timeAcceleration1, 0, ...
            timeAcceleration1 + context.af / jMaximum];
        candidates = appendCandidate(candidates, context, limits, phase, "initialAccelerationVelocity");
    end

    if isfinite(timeAcceleration0) && isfinite(timeAcceleration1)
        phase(1:3) = [timeAcceleration0 - context.a0 / jMaximum, ...
            0, timeAcceleration0];
        phase(4) = (context.afCubed - context.a0Cubed) / (3 * jSquared * vMaximum) + (context.a0 * context.v0 - context.af * context.vf + (context.afSquared * timeAcceleration1 + context.a0Squared * timeAcceleration0) / 2) / (jMaximum * vMaximum) - (context.v0 / vMaximum + 1) * timeAcceleration0 - (context.vf / vMaximum + 1) * timeAcceleration1 + context.displacement / vMaximum;
        phase(5:7) = [timeAcceleration1, 0, ...
            timeAcceleration1 + context.af / jMaximum];
        candidates = appendCandidate(candidates, context, limits, phase, "velocity");
    end
end

function candidates = appendAccelerationProfiles(candidates, context, limits)
    % Add the two families that reach both signed acceleration bounds.
    aMaximum    = limits.aMaximum;
    aMinimum    = limits.aMinimum;
    jMaximum    = limits.jMaximum;
    jSquared    = jMaximum^2;
    denominator = aMaximum - aMinimum;

    radicand = (3 * (context.afFourth * aMaximum - context.a0Fourth * aMinimum) + aMaximum * aMinimum * (8 * (context.a0Cubed - context.afCubed) + 3 * aMaximum * aMinimum * (aMaximum - aMinimum) + 6 * aMinimum * context.afSquared - 6 * aMaximum * context.a0Squared) + 12 * jMaximum * (aMaximum * aMinimum * ((aMaximum - 2 * context.a0) * context.v0 - (aMinimum - 2 * context.af) * context.vf) + aMinimum * context.a0Squared * context.v0 - aMaximum * context.afSquared * context.vf)) / (3 * denominator * jSquared) + 4 * (aMaximum * context.vfSquared - aMinimum * context.v0Squared - 2 * aMinimum * aMaximum * context.displacement) / denominator;
    if radicand < 0
        return;
    end

    root         = sqrt(radicand) / 2;
    baseInitial  = context.a0Squared / (2 * aMaximum * jMaximum) + (aMinimum - 2 * aMaximum) / (2 * jMaximum) - context.v0 / aMaximum;
    baseTerminal = -context.afSquared / (2 * aMinimum * jMaximum) - (aMaximum - 2 * aMinimum) / (2 * jMaximum) + context.vf / aMinimum;
    phase        = zeros(1, 7);
    phase([1, 3, 5, 7]) = [ ...
        (-context.a0 + aMaximum) / jMaximum, ...
        aMaximum / jMaximum, -aMinimum / jMaximum, ...
        -aMinimum / jMaximum + context.af / jMaximum];

    phase(2) = baseInitial - root / aMaximum;
    phase(6) = baseTerminal + root / aMinimum;
    candidates = appendCandidate(candidates, context, limits, phase, "accelerationBounds");

    phase(2) = baseInitial + root / aMaximum;
    phase(6) = baseTerminal - root / aMinimum;
    candidates = appendCandidate(candidates, context, limits, phase, "accelerationBounds");
end

function candidates = appendUnconstrainedProfiles(candidates, context, limits)
    % Port PositionThirdOrderStep1::time_all_none_acc0_acc1.
    % The supplied v0.19.4 source checks UDDU here; despite the source comment
    % mentioning UDUD strategy, it contains no separate UDUD check to port.
    aMaximum = limits.aMaximum;
    aMinimum = limits.aMinimum;
    jMaximum = limits.jMaximum;
    jSquared = jMaximum^2;

    h2None        = (context.a0Squared - context.afSquared) / (2 * jMaximum) + context.vf - context.v0;
    h2NoneSquared = h2None^2;
    rootsNone     = realQuarticRoots([1, 0, -2 * (context.a0Squared + context.afSquared - 2 * jMaximum * (context.v0 + context.vf)) / jSquared, 4 * (context.a0Cubed - context.afCubed + 3 * jMaximum * (context.af * context.vf - context.a0 * context.v0)) / (3 * jMaximum * jSquared) - 4 * context.displacement / jMaximum, -h2NoneSquared / jSquared]);
    for rootIndex = 1:numel(rootsNone)
        time = rootsNone(rootIndex);
        if time < (context.a0 - context.af) / jMaximum || time > (aMaximum - aMinimum) / jMaximum || time <= eps
            continue;
        end
        h1         = jMaximum * time^2;
        residual   = -h2NoneSquared / (4 * jMaximum * time) + h2None * (context.af / jMaximum + time) + (4 * context.a0Cubed + 2 * context.afCubed - 6 * context.a0Squared * (context.af + 2 * jMaximum * time) + 12 * (context.af - context.a0) * jMaximum * context.v0 + 3 * jSquared * (-4 * context.displacement + (h1 + 8 * context.v0) * time)) / (12 * jSquared);
        derivative = h2None + 2 * context.v0 - context.a0Squared / jMaximum + h2NoneSquared / (4 * h1) + 3 * h1 / 4;
        if isfinite(derivative) && derivative ~= 0
            time = time - residual / derivative;
        end
        h0    = h2None / (2 * jMaximum * time);
        phase = [h0 + time / 2 - context.a0 / jMaximum, ...
            0, time, 0, 0, 0, -h0 + time / 2 + ...
            context.af / jMaximum];
        candidates = appendCandidate(candidates, context, limits, phase, "unconstrained");
    end

    h3Initial    = (context.a0Squared - context.afSquared) / (2 * aMaximum * jMaximum) + (context.vf - context.v0) / aMaximum;
    h0Initial    = 3 * (context.afFourth - context.a0Fourth) + 8 * (context.a0Cubed - context.afCubed) * aMaximum + 24 * aMaximum * jMaximum * (context.af * context.vf - context.a0 * context.v0) - 6 * context.a0Squared * (aMaximum^2 - 2 * jMaximum * context.v0) + 6 * context.afSquared * (aMaximum^2 - 2 * jMaximum * context.vf) + 12 * jMaximum * (jMaximum * (context.vfSquared - context.v0Squared - 2 * aMaximum * context.displacement) - aMaximum^2 * (context.vf - context.v0));
    h2Initial    = -context.afSquared + aMaximum^2 + 2 * jMaximum * context.vf;
    rootsInitial = realQuarticRoots([1, -2 * aMaximum / jMaximum, h2Initial / jSquared, 0, h0Initial / (12 * jSquared^2)]);
    for rootIndex = 1:numel(rootsInitial)
        time = rootsInitial(rootIndex);
        if time < (aMaximum - context.af) / jMaximum || time > (aMaximum - aMinimum) / jMaximum || time <= eps
            continue;
        end
        h1         = jMaximum * time;
        residual   = h0Initial / (12 * jSquared * time) + time * (h2Initial + h1 * (h1 - 2 * aMaximum));
        derivative = 2 * (h2Initial + h1 * (2 * h1 - 3 * aMaximum));
        if isfinite(derivative) && derivative ~= 0
            time = time - residual / derivative;
        end
        phase = [(-context.a0 + aMaximum) / jMaximum, ...
            h3Initial - 2 * time + jMaximum / aMaximum * time^2, ...
            time, 0, 0, 0, (context.af - aMaximum) / jMaximum + time];
        candidates = appendCandidate(candidates, context, limits, phase, "initialAcceleration");
    end

    h3Terminal    = -(context.a0Squared + context.afSquared) / (2 * jMaximum * aMinimum) + aMinimum / jMaximum + (context.vf - context.v0) / aMinimum;
    h0Terminal    = (context.a0Fourth - context.afFourth) / 4 + 2 * (context.afCubed - context.a0Cubed) * aMinimum / 3 + (context.a0Squared - context.afSquared) * aMinimum^2 / 2 + jMaximum * (context.afSquared * context.vf + context.a0Squared * context.v0 + 2 * aMinimum * (jMaximum * context.displacement - context.a0 * context.v0 - context.af * context.vf) + aMinimum^2 * (context.v0 + context.vf) + jMaximum * (context.v0Squared - context.vfSquared));
    h2Terminal    = context.a0Squared - context.a0 * aMinimum + 2 * jMaximum * context.v0;
    rootsTerminal = realQuarticRoots([1, 2 * (2 * context.a0 - aMinimum) / jMaximum, (5 * context.a0Squared + aMinimum * (aMinimum - 6 * context.a0) + 2 * jMaximum * context.v0) / jSquared, 2 * (context.a0 - aMinimum) * h2Terminal / (jSquared * jMaximum), h0Terminal / jSquared^2]);
    for rootIndex = 1:numel(rootsTerminal)
        time = rootsTerminal(rootIndex);
        if time < (aMinimum - context.a0) / jMaximum || time > (aMaximum - context.a0) / jMaximum || time <= eps
            continue;
        end
        % Repeat the refinement alternatives needed to refine the current solution.
        for refinementIndex = 1:3
            h1         = jMaximum * time;
            residual   = -(h0Terminal / 2 + h1 * (context.a0Cubed + 2 * jMaximum * context.a0 * context.v0 + context.a0 * (aMinimum - 2 * h1) * (aMinimum - h1) + context.a0Squared * (5 * h1 / 2 - 2 * aMinimum) + aMinimum^2 * h1 / 2 + jMaximum * (h1 / 2 - aMinimum) * (h1 * time + 2 * context.v0))) / jMaximum;
            derivative = (aMinimum - context.a0 - h1) * (h2Terminal + h1 * (4 * context.a0 - aMinimum + 2 * h1));
            if abs(residual) <= 1e-9 || ~isfinite(derivative) || derivative == 0
                break;
            end
            time = time - min(residual / derivative, time);
        end
        phase = [time, 0, (context.a0 - aMinimum) / jMaximum + time, ...
            0, 0, h3Terminal - ...
            (2 * context.a0 + jMaximum * time) * time / aMinimum, ...
            (context.af - aMinimum) / jMaximum];
        candidates = appendCandidate(candidates, context, limits, phase, "terminalAcceleration");
    end
end

function candidates = appendTerminalAccelerationVelocityTwoStep(candidates, context, limits)
    % Port PositionThirdOrderStep1::time_acc1_vel_two_step.
    vMaximum = limits.vMaximum;
    aMinimum = limits.aMinimum;
    jMaximum = limits.jMaximum;
    jSquared = jMaximum^2;
    phase = [0, 0, context.a0 / jMaximum, 0, ...
        -aMinimum / jMaximum, 0, -aMinimum / jMaximum + context.af / jMaximum];
    phase(4) = -(3 * context.afFourth - 8 * aMinimum * ...
        (context.afCubed - context.a0Cubed) - 24 * aMinimum * jMaximum * ...
        (context.a0 * context.v0 - context.af * context.vf) + ...
        6 * context.afSquared * (aMinimum^2 - 2 * jMaximum * context.vf) - ...
        12 * jMaximum * (2 * aMinimum * jMaximum * context.displacement + ...
        aMinimum^2 * (context.vf + vMaximum) + jMaximum * ...
        (vMaximum^2 - context.vfSquared) + aMinimum * context.a0 * ...
        (context.a0Squared - 2 * jMaximum * (context.v0 + vMaximum)) / jMaximum)) / ...
        (24 * aMinimum * jSquared * vMaximum);
    phase(6) = -(context.afSquared / 2 - aMinimum^2 + ...
        jMaximum * (vMaximum - context.vf)) / (aMinimum * jMaximum);
    candidates = appendCandidate(candidates, context, limits, phase, ...
        "terminalAccelerationVelocityTwoStep");
end

function candidates = appendInitialAccelerationTwoStep(candidates, context, limits)
    % Port PositionThirdOrderStep1::time_acc0_two_step.
    aMaximum = limits.aMaximum;
    aMinimum = limits.aMinimum;
    jMaximum = limits.jMaximum;
    initialCount = numel(candidates);

    phase = [0, (context.afSquared - context.a0Squared + ...
        2 * jMaximum * (context.vf - context.v0)) / ...
        (2 * context.a0 * jMaximum), ...
        (context.a0 - context.af) / jMaximum, 0, 0, 0, 0];
    candidates = appendCandidate(candidates, context, limits, phase, ...
        "initialAccelerationTwoStep");
    if numel(candidates) > initialCount
        return;
    end

    phase = [(-context.a0 + aMaximum) / jMaximum, ...
        (context.a0Squared + context.afSquared - 2 * aMaximum^2 + ...
        2 * jMaximum * (context.vf - context.v0)) / ...
        (2 * aMaximum * jMaximum), ...
        (-context.af + aMaximum) / jMaximum, 0, 0, 0, 0];
    candidates = appendCandidate(candidates, context, limits, phase, ...
        "initialAccelerationTwoStep");
    if numel(candidates) > initialCount
        return;
    end

    h0 = 3 * (context.afSquared - context.a0Squared + ...
        2 * jMaximum * (context.v0 + context.vf));
    h2 = context.a0Cubed + 2 * context.afCubed + ...
        6 * jMaximum^2 * context.displacement + ...
        6 * (context.af - context.a0) * jMaximum * context.vf - ...
        3 * context.a0 * context.afSquared;
    h1 = ruckigEngine.internal.safeSqrt(2 * (2 * h2^2 + h0 * ...
        (context.a0Fourth - 6 * context.a0Squared * ...
        (context.afSquared + 2 * jMaximum * context.vf) + ...
        8 * context.a0 * (context.afCubed + 3 * jMaximum^2 * ...
        context.displacement + 3 * context.af * jMaximum * context.vf) - ...
        3 * (context.afFourth + 4 * context.afSquared * jMaximum * context.vf + ...
        4 * jMaximum^2 * (context.vfSquared - context.v0Squared))))) * ...
        sign(jMaximum);
    phase = [(4 * context.afCubed + 2 * context.a0Cubed - ...
        6 * context.a0 * context.afSquared + 12 * jMaximum^2 * ...
        context.displacement + 12 * (context.af - context.a0) * ...
        jMaximum * context.vf + h1) / (2 * jMaximum * h0), ...
        -h1 / (jMaximum * h0), ...
        (-4 * context.a0Cubed - 2 * context.afCubed + ...
        6 * context.a0Squared * context.af + 12 * jMaximum^2 * ...
        context.displacement - 12 * (context.af - context.a0) * ...
        jMaximum * context.v0 + h1) / (2 * jMaximum * h0), 0, 0, 0, 0];
    candidates = appendCandidate(candidates, context, limits, phase, ...
        "initialAccelerationTwoStep");
    if numel(candidates) > initialCount
        return;
    end

    time  = (aMaximum - aMinimum) / jMaximum;
    phase = [(-context.a0 + aMaximum) / jMaximum, ...
        (context.a0Squared - context.afSquared) / ...
        (2 * aMaximum * jMaximum) + ...
        (context.vf - context.v0 + jMaximum * time^2) / aMaximum - 2 * time, ...
        time, 0, 0, 0, (context.af - aMinimum) / jMaximum];
    candidates = appendCandidate(candidates, context, limits, phase, ...
        "initialAccelerationTwoStep");
end

function candidates = appendVelocityTwoStep(candidates, context, limits)
    % Port PositionThirdOrderStep1::time_vel_two_step.
    vMaximum = limits.vMaximum;
    jMaximum = limits.jMaximum;
    h1 = ruckigEngine.internal.safeSqrt(context.afSquared / ...
        (2 * jMaximum^2) + (vMaximum - context.vf) / jMaximum);
    initialCount = numel(candidates);
    phase = [-context.a0 / jMaximum, 0, 0, 0, h1, 0, ...
        h1 + context.af / jMaximum];
    phase(4) = (context.afCubed - context.a0Cubed) / ...
        (3 * jMaximum^2 * vMaximum) + ...
        (context.a0 * context.v0 - context.af * context.vf + ...
        context.afSquared * h1 / 2) / (jMaximum * vMaximum) - ...
        (context.vf / vMaximum + 1) * h1 + context.displacement / vMaximum;
    candidates = appendCandidate(candidates, context, limits, phase, ...
        "velocityTwoStep");
    if numel(candidates) > initialCount
        return;
    end

    phase = [0, 0, context.a0 / jMaximum, 0, h1, 0, ...
        h1 + context.af / jMaximum];
    phase(4) = (context.afCubed - context.a0Cubed) / ...
        (3 * jMaximum^2 * vMaximum) + ...
        (context.a0 * context.v0 - context.af * context.vf + ...
        (context.afSquared * h1 + context.a0Cubed / jMaximum) / 2) / ...
        (jMaximum * vMaximum) - (context.v0 / vMaximum + 1) * ...
        context.a0 / jMaximum - (context.vf / vMaximum + 1) * h1 + ...
        context.displacement / vMaximum;
    candidates = appendCandidate(candidates, context, limits, phase, ...
        "velocityTwoStep");
end

function candidates = appendNoneTwoStep(candidates, context, limits)
    % Port PositionThirdOrderStep1::time_none_two_step.
    jMaximum = limits.jMaximum;
    h0 = ruckigEngine.internal.safeSqrt((context.a0Squared + ...
        context.afSquared) / 2 + jMaximum * (context.vf - context.v0)) * ...
        sign(jMaximum);
    initialCount = numel(candidates);
    phase = [(h0 - context.a0) / jMaximum, 0, ...
        (h0 - context.af) / jMaximum, 0, 0, 0, 0];
    candidates = appendCandidate(candidates, context, limits, phase, ...
        "noneTwoStep");
    if numel(candidates) > initialCount
        return;
    end

    phase = [(context.af - context.a0) / jMaximum, 0, 0, 0, 0, 0, 0];
    candidates = appendCandidate(candidates, context, limits, phase, ...
        "noneSingleStep");
end

function candidates = appendCandidate(candidates, context, limits, phaseDuration, family)
    % Integrate one seven-phase UDDU law and retain it only when fully valid.
    phaseJerk    = limits.jMaximum * [1, 0, -1, 0, -1, 0, 1];
    initialState = struct("position", context.p0, ...
        "velocity", context.v0, ...
        "acceleration", context.a0);
    terminalState = struct("position", context.pf, ...
        "velocity", context.vf, ...
        "acceleration", context.af);
    physicalLimits = struct("maximumVelocity", context.vMaximum, ...
        "maximumAcceleration", context.aMaximum);
    evaluated = ruckigEngine.evaluateAxisSwitchingProfile(initialState, terminalState, physicalLimits, phaseDuration, phaseJerk, family);
    if ~evaluated.Success
        return;
    end

    candidate = createEmptyCandidate();
    candidate.PhaseDuration = evaluated.PhaseDuration;
    candidate.PhaseJerk     = evaluated.PhaseJerk;
    candidate.Position      = evaluated.Position;
    candidate.Velocity      = evaluated.Velocity;
    candidate.Acceleration  = evaluated.Acceleration;
    candidate.Family        = family;
    candidate.PathLength    = evaluated.PathLength;
    candidate.Duration      = sum(evaluated.PhaseDuration);
    candidate.Direction     = sign(limits.jMaximum);
    candidates(end + 1, 1) = candidate;
end

function values = realQuarticRoots(coefficients)
    % Port roots::solve_quart_monic from roots.hpp.
    coefficients = coefficients / coefficients(1);
    values = solveQuarticMonic(coefficients(2), coefficients(3), ...
        coefficients(4), coefficients(5));
end

function values = solveQuarticMonic(a, b, c, d)
    % Factor the quartic using Ruckig's resolvent-cubic method.
    values = zeros(1, 0);
    if abs(d) < eps
        if abs(c) < eps
            values = insertPositive(values, 0);
            discriminant = a^2 - 4 * b;
            if abs(discriminant) < eps
                values = insertPositive(values, -a / 2);
            elseif discriminant > 0
                root   = sqrt(discriminant);
                values = insertPositive(values, (-a - root) / 2);
                values = insertPositive(values, (-a + root) / 2);
            end
            values = sort(values);
            return;
        end
        if abs(a) < eps && abs(b) < eps
            values = insertPositive(values, 0);
            values = insertPositive(values, -realCubeRoot(c));
            values = sort(values);
            return;
        end
    end

    [resolvent, count] = solveResolvent(-b, a * c - 4 * d, ...
        -a^2 * d - c^2 + 4 * b * d);
    y = resolvent(1);
    if count ~= 1
        if abs(resolvent(2)) > abs(y)
            y = resolvent(2);
        end
        if abs(resolvent(3)) > abs(y)
            y = resolvent(3);
        end
    end
    discriminant = y^2 - 4 * d;
    if abs(discriminant) < eps
        q1 = y / 2;
        q2 = q1;
        discriminant = a^2 - 4 * (b - y);
        if abs(discriminant) < eps
            p1 = a / 2;
            p2 = p1;
        else
            root = sqrt(discriminant);
            p1   = (a + root) / 2;
            p2   = (a - root) / 2;
        end
    else
        root = sqrt(discriminant);
        q1   = (y + root) / 2;
        q2   = (y - root) / 2;
        p1   = (a * q1 - c) / (q1 - q2);
        p2   = (c - a * q2) / (q1 - q2);
    end
    rootTolerance = 16 * eps;
    discriminant  = p1^2 - 4 * q1;
    if abs(discriminant) < rootTolerance
        values = insertPositive(values, -p1 / 2);
    elseif discriminant > 0
        root   = sqrt(discriminant);
        values = insertPositive(values, (-p1 - root) / 2);
        values = insertPositive(values, (-p1 + root) / 2);
    end
    discriminant = p2^2 - 4 * q2;
    if abs(discriminant) < rootTolerance
        values = insertPositive(values, -p2 / 2);
    elseif discriminant > 0
        root   = sqrt(discriminant);
        values = insertPositive(values, (-p2 - root) / 2);
        values = insertPositive(values, (-p2 + root) / 2);
    end
    values = sort(values);
end

function [solutions, count] = solveResolvent(a, b, c)
    % Port roots::solve_resolvent, including its repeated-root convention.
    cos120 = -0.5;
    sin120 = 0.866025403784438646764;
    a      = a / 3;
    a2     = a^2;
    q      = a2 - b / 3;
    r      = (a * (2 * a2 - b) + c) / 2;
    r2     = r^2;
    q3     = q^3;
    solutions = zeros(1, 3);
    if r2 < q3
        qRoot = sqrt(q);
        angle = acos(min(max(r / (q * qRoot), -1), 1)) / 3;
        q     = -2 * qRoot;
        ux    = cos(angle) * q;
        uy    = sin(angle) * q;
        solutions = [ux - a, ux * cos120 - uy * sin120 - a, ...
            ux * cos120 + uy * sin120 - a];
        count = 3;
        return;
    end
    valueA = -realCubeRoot(abs(r) + sqrt(r2 - q3));
    if r < 0
        valueA = -valueA;
    end
    if valueA == 0
        valueB = 0;
    else
        valueB = q / valueA;
    end
    solutions(1) = valueA + valueB - a;
    solutions(2) = -(valueA + valueB) / 2 - a;
    solutions(3) = sqrt(3) * (valueA - valueB) / 2;
    if abs(solutions(3)) < eps
        solutions(3) = solutions(2);
        count = 2;
    else
        count = 1;
    end
end

function values = insertPositive(values, value)
    % Match roots::PositiveSet: negative roots are not profile candidates.
    if isreal(value) && value >= 0
        values(end + 1) = value;
    end
end

function value = realCubeRoot(value)
    % Match std::cbrt for negative real inputs.
    value = sign(value) * abs(value)^(1 / 3);
end

function candidate = createEmptyCandidate()
    % Initialize a switching-profile candidate.
    candidate = struct();
    candidate.PhaseDuration = zeros(1, 7);
    candidate.PhaseJerk     = zeros(1, 7);
    candidate.Position      = zeros(1, 8);
    candidate.Velocity      = zeros(1, 8);
    candidate.Acceleration  = zeros(1, 8);
    candidate.Family        = "";
    candidate.PathLength    = Inf;
    candidate.Duration      = NaN;
    candidate.Direction     = 0;
end

function profile = createEmptyProfile()
    % Report the unsupported switching family.
    profile = struct();
    profile.Success       = false;
    profile.Message       = "No exact axis profile was created.";
    profile.PhaseDuration = zeros(1, 7);
    profile.PhaseJerk     = zeros(1, 7);
    profile.Duration      = NaN;
    profile.FinalTime     = NaN;
    profile.Position      = zeros(1, 8);
    profile.Velocity      = zeros(1, 8);
    profile.Acceleration  = zeros(1, 8);
    profile.Family        = "";
    profile.PathLength    = Inf;
end
