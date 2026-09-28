function report = checkRuckigEngineRandomCases(caseCount, seed, options)
%% Section 0: Header & Readme
% SYNTAX
%   report = checkRuckigEngineRandomCases()
%   report = checkRuckigEngineRandomCases(caseCount)
%   report = checkRuckigEngineRandomCases(caseCount, seed)
%   report = checkRuckigEngineRandomCases(caseCount, seed, options)
%
% PURPOSE
%   - Throw many random state-to-state requests at ruckigEngine.solve and
%     check every returned motion with code that does not share anything
%     with the engine. The engine already validates its own answer; this
%     script checks the answer a second time from the polynomial record,
%     and then checks properties the engine's validator cannot see:
%       * the earliest-arrival duration is really minimal (a fixed-time
%         request a little shorter must be refused),
%       * the same request with a fixed time equal to, or longer than, the
%         earliest duration is still solvable,
%       * the answer does not depend on axis order or on mirroring all
%         coordinates,
%       * repeating a solve gives the identical answer.
%   - A random request that the engine refuses is recorded by its
%     TerminationReason and is not a defect by itself. A refused request
%     with an unexpected reason, or a returned motion that fails a check
%     here, is a defect.
%   - Two expected behaviors are reported as observations, not defects:
%       * Blocked intervals. With nonzero boundary velocities a fixed time
%         longer than the earliest one can be impossible even though a
%         still longer time works again (the reachable displacement at a
%         given duration is an interval that moves with the duration). A
%         refused long fixed time is only a defect when no duration on a
%         ladder up to 20 times the earliest one solves either.
%       * A too-short fixed time refused with a reason other than
%         fixedTimeBelowMinimum. Multi-axis requests can have a synchronized
%         minimum above every single-axis minimum, and the engine then
%         reports the synchronization failure instead.
%   - Position boxes only reject a motion; they do not steer it. Boxed
%     requests therefore often end as exactProfileValidationFailed, which
%     is counted, not flagged.
%
% INPUTS
%   - caseCount (positive integer scalar, optional; default 200)
%       Number of random base requests to generate.
%   - seed (nonnegative integer scalar, optional; default 1)
%       Random generator seed. The same seed always builds the same cases,
%       so any defect can be reproduced from its case index alone.
%   - options (scalar struct, optional; default struct())
%       Verbose (logical; default true) prints one line per defect.
%       ShortFactor (scalar; default 0.98) scales the earliest duration to
%       build the fixed-time request that must be refused.
%       LongFactor (scalar; default 1.5) scales the earliest duration to
%       build the fixed-time request that is expected to succeed.
%
% OUTPUTS
%   - report (scalar struct)
%       Cases: one table row per base request (dimension, control order,
%         reason, duration, wall time, follow-up outcomes, defect count).
%       Requests: cell array of the generated requests, one per case, each
%         with initialState, terminalState, limits, and options fields that
%         can be passed straight back to ruckigEngine.solve to replay a case.
%       Defects: struct array with CaseIndex, Check, and Detail for every
%         failed check, in the order they were found.
%       Observations: same layout, for the expected behaviors listed above.
%       ReasonCounts: table of TerminationReason counts over base requests.
%       Seed, CaseCount: the inputs, so the run can be repeated.
%
% UNITS
%   - Positions are abstract coordinate units, time is seconds, and the
%     derivatives use units/s, units/s^2, and units/s^3.

%% Section 1: Validate Inputs And Seed The Generator

if nargin < 1 || isempty(caseCount)
    caseCount = 200;
end
if nargin < 2 || isempty(seed)
    seed = 1;
end
if nargin < 3 || isempty(options)
    options = struct();
end
validateattributes(caseCount, {'numeric'}, {'scalar', 'integer', 'positive'});
validateattributes(seed, {'numeric'}, {'scalar', 'integer', 'nonnegative'});
verbose     = resolveOption(options, "Verbose", true);
shortFactor = resolveOption(options, "ShortFactor", 0.98);
longFactor  = resolveOption(options, "LongFactor", 1.5);
validateattributes(shortFactor, {'numeric'}, {'scalar', '>', 0, '<', 1});
validateattributes(longFactor, {'numeric'}, {'scalar', '>', 1});

repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(fullfile(repositoryRoot, "trajectory"));
generator = RandStream("mt19937ar", "Seed", seed);

% Tolerances for the independent checks. The engine certifies to 1e-6
% (ten times its 1e-7 constraint tolerance); allow a little more here so a
% pass or fail is about the motion, not about rounding in this script.
endpointTolerance = 1e-6;
limitTolerance    = 1e-6;
durationTolerance = 1e-9;

%% Section 2: Run Every Random Case

caseTable = table('Size', [caseCount, 14], ...
    'VariableTypes', ["double", "double", "string", "logical", "string", "string", "logical", ...
    "double", "double", "string", "string", "string", "string", "double"], ...
    'VariableNames', ["Dimension", "ControlOrder", "Kind", "HasBox", "Reason", "Message", ...
    "Success", "Duration_s", "WallTime_s", "ShortFixed", "ExactFixed", ...
    "LongFixed", "Symmetry", "DefectCount"]);
defects      = struct("CaseIndex", {}, "Check", {}, "Detail", {});
observations = struct("CaseIndex", {}, "Check", {}, "Detail", {});
requests     = cell(caseCount, 1);

for caseIndex = 1:caseCount
    request             = createRandomRequest(generator);
    requests{caseIndex} = request;
    caseTable.Dimension(caseIndex)    = numel(request.initialState.position);
    caseTable.ControlOrder(caseIndex) = request.controlOrder;
    caseTable.Kind(caseIndex)         = request.kind;
    caseTable.HasBox(caseIndex)       = request.hasBox;
    caseDefects      = struct("CaseIndex", {}, "Check", {}, "Detail", {});
    caseObservations = struct("CaseIndex", {}, "Check", {}, "Detail", {});

    % Base request: earliest arrival with a generous horizon.
    caseTimer = tic;
    result    = ruckigEngine.solve(request.initialState, request.terminalState, request.limits, request.options);
    caseTable.WallTime_s(caseIndex) = toc(caseTimer);
    caseTable.Reason(caseIndex)     = string(result.TerminationReason);
    caseTable.Message(caseIndex)    = string(result.Message);
    caseTable.Success(caseIndex)    = result.Success;
    caseTable.Duration_s(caseIndex) = result.Duration;

    expectedReasons = ["goalReached", "kinematicallyInfeasibleBoundaryState", ...
        "fixedTimeBelowMinimum", "exactProfileValidationFailed", "unsupportedSwitchingFamily"];
    if ~any(string(result.TerminationReason) == expectedReasons)
        caseDefects(end + 1) = makeDefect(caseIndex, "unexpectedReason", ...
            sprintf("%s: %s", result.TerminationReason, result.Message)); %#ok<AGROW>
    end
    if result.Success ~= result.Validation.Passed
        caseDefects(end + 1) = makeDefect(caseIndex, "successDisagreesWithValidation", result.Message); %#ok<AGROW>
    end

    if result.Success
        % Independent re-check of the returned motion.
        motionDefects = checkReturnedMotion(caseIndex, result, request, ...
            endpointTolerance, limitTolerance, durationTolerance);
        caseDefects   = [caseDefects, motionDefects]; %#ok<AGROW>

        % Repeating the same solve must give the identical answer.
        repeatResult = ruckigEngine.solve(request.initialState, request.terminalState, request.limits, request.options);
        if ~isequal(repeatResult.Polynomial, result.Polynomial) || repeatResult.Duration ~= result.Duration
            caseDefects(end + 1) = makeDefect(caseIndex, "notDeterministic", ...
                sprintf("durations %.12g and %.12g", result.Duration, repeatResult.Duration)); %#ok<AGROW>
        end

        % A fixed time a little shorter than the earliest arrival must be
        % refused. If it is accepted and the motion checks out, the
        % earliest-arrival answer was not minimal.
        shortResult = solveFixed(request, shortFactor * result.Duration);
        caseTable.ShortFixed(caseIndex) = string(shortResult.TerminationReason);
        if shortResult.Success
            shortDefects = checkReturnedMotion(caseIndex, shortResult, request, ...
                endpointTolerance, limitTolerance, durationTolerance);
            if isempty(shortDefects)
                caseDefects(end + 1) = makeDefect(caseIndex, "earliestArrivalNotMinimal", ...
                    sprintf("earliest %.9g s but fixed %.9g s also solves", ...
                    result.Duration, shortResult.Duration)); %#ok<AGROW>
            end
        elseif string(shortResult.TerminationReason) ~= "fixedTimeBelowMinimum"
            caseObservations(end + 1) = makeDefect(caseIndex, "shortFixedReasonNotMinimum", ...
                sprintf("%s: %s", shortResult.TerminationReason, shortResult.Message)); %#ok<AGROW>
        end

        % The exact earliest duration requested as a fixed time must solve.
        exactResult = solveFixed(request, result.Duration);
        caseTable.ExactFixed(caseIndex) = string(exactResult.TerminationReason);
        if exactResult.Success
            caseDefects = [caseDefects, checkReturnedMotion(caseIndex, exactResult, request, ...
                endpointTolerance, limitTolerance, durationTolerance)]; %#ok<AGROW>
        else
            caseDefects(end + 1) = makeDefect(caseIndex, "exactFixedRefused", ...
                sprintf("%s: %s", exactResult.TerminationReason, exactResult.Message)); %#ok<AGROW>
        end

        % A longer fixed time is expected to solve. If it does not, climb a
        % ladder of longer durations: success further up means the refused
        % duration sat in a blocked interval, which is expected.
        longResult = solveFixed(request, longFactor * result.Duration);
        caseTable.LongFixed(caseIndex) = string(longResult.TerminationReason);
        if longResult.Success
            caseDefects = [caseDefects, checkReturnedMotion(caseIndex, longResult, request, ...
                endpointTolerance, limitTolerance, durationTolerance)]; %#ok<AGROW>
        else
            ladderSolved = false;
            for ladderFactor = [2, 3, 5, 10, 20]
                ladderResult = solveFixed(request, ladderFactor * result.Duration);
                if ladderResult.Success
                    ladderSolved = true;
                    caseDefects  = [caseDefects, checkReturnedMotion(caseIndex, ladderResult, request, ...
                        endpointTolerance, limitTolerance, durationTolerance)]; %#ok<AGROW>
                    break;
                end
            end
            if ladderSolved
                caseObservations(end + 1) = makeDefect(caseIndex, "blockedInterval", ...
                    sprintf("x%g refused (%s), x%g solves", longFactor, ...
                    longResult.TerminationReason, ladderFactor)); %#ok<AGROW>
            else
                caseDefects(end + 1) = makeDefect(caseIndex, "longFixedRefused", ...
                    sprintf("x%g through x20 all refused: %s: %s", longFactor, ...
                    longResult.TerminationReason, longResult.Message)); %#ok<AGROW>
            end
        end

        % Reordering the axes or mirroring every coordinate must not change
        % the earliest duration.
        symmetryDefects = checkSymmetry(caseIndex, request, result.Duration, generator, durationTolerance);
        if isempty(symmetryDefects)
            caseTable.Symmetry(caseIndex) = "same";
        else
            caseTable.Symmetry(caseIndex) = "different";
        end
        caseDefects = [caseDefects, symmetryDefects]; %#ok<AGROW>
    end

    caseTable.DefectCount(caseIndex) = numel(caseDefects);
    defects      = [defects, caseDefects]; %#ok<AGROW>
    observations = [observations, caseObservations]; %#ok<AGROW>
    if verbose
        for defectIndex = 1:numel(caseDefects)
            fprintf("case %4d  %-32s %s\n", caseIndex, ...
                caseDefects(defectIndex).Check, caseDefects(defectIndex).Detail);
        end
    end
end

%% Section 3: Summarize

[reasonNames, ~, reasonIndex] = unique(caseTable.Reason);
reasonCounts = table(reasonNames, accumarray(reasonIndex, 1), ...
    'VariableNames', ["Reason", "Count"]);
report = struct("Seed", seed, "CaseCount", caseCount, "Cases", caseTable, ...
    "Requests", {requests}, "Defects", defects, "Observations", observations, ...
    "ReasonCounts", reasonCounts);

fprintf("\nRandom cases: %d (seed %d), solved %d, wall %.2f s\n", caseCount, seed, ...
    nnz(caseTable.Success), sum(caseTable.WallTime_s));
disp(reasonCounts);
solved = caseTable(caseTable.Success, :);
if ~isempty(solved)
    fprintf("Follow-ups on solved cases: short fixed refused %d/%d, exact fixed solved %d/%d, long fixed solved %d/%d, symmetric %d/%d\n", ...
        nnz(solved.ShortFixed == "fixedTimeBelowMinimum"), height(solved), ...
        nnz(solved.ExactFixed == "goalReached"), height(solved), ...
        nnz(solved.LongFixed == "goalReached"), height(solved), ...
        nnz(solved.Symmetry == "same"), height(solved));
end
if ~isempty(observations)
    [observationNames, ~, observationIndex] = unique(string({observations.Check}));
    fprintf("Observations (expected behavior, see header):\n");
    disp(table(observationNames(:), accumarray(observationIndex(:), 1), 'VariableNames', ["Check", "Count"]));
end
fprintf("Defects: %d in %d cases\n", numel(defects), nnz(caseTable.DefectCount > 0));
if ~isempty(defects)
    [checkNames, ~, checkIndex] = unique(string({defects.Check}));
    disp(table(checkNames(:), accumarray(checkIndex(:), 1), 'VariableNames', ["Check", "Count"]));
end
end

%% Section 4: Local Functions

function value = resolveOption(options, name, default)
    % Read one option field, falling back to the default when absent or empty.
    value = default;
    if isfield(options, name) && ~isempty(options.(name))
        value = options.(name);
    end
end

function defect = makeDefect(caseIndex, check, detail)
    defect = struct("CaseIndex", caseIndex, "Check", string(check), "Detail", string(detail));
end

function request = createRandomRequest(generator)
    % Build one random request. Mix of dimensions, control orders, and
    % boundary-state kinds so both the easy and the awkward families show up.
    dimension = generator.randi(3);
    if generator.rand() < 0.7
        controlOrder = 3;
    else
        controlOrder = 2;
    end

    % Limits spread over a few decades, sometimes different per axis.
    if generator.rand() < 0.5
        axisCount = dimension;
    else
        axisCount = 1;
    end
    maximumVelocity     = 10 .^ (generator.rand(1, axisCount) * 2 - 0.5);   % 0.3 .. 30
    maximumAcceleration = 10 .^ (generator.rand(1, axisCount) * 2 - 1);     % 0.1 .. 10
    maximumJerk         = 10 .^ (generator.rand(1, axisCount) * 2.5 - 0.5); % 0.3 .. 300
    velocityBound       = repmat(maximumVelocity, 1, dimension / axisCount);
    accelerationBound   = repmat(maximumAcceleration, 1, dimension / axisCount);

    % Boundary states: rest-to-rest, general, or sitting on a limit.
    kindDraw = generator.rand();
    if kindDraw < 0.25
        kind = "restToRest";
        initialVelocity = zeros(1, dimension);      terminalVelocity = zeros(1, dimension);
        initialAcceleration = zeros(1, dimension);  terminalAcceleration = zeros(1, dimension);
    elseif kindDraw < 0.85
        kind = "general";
        initialVelocity      = velocityBound .* (2 * generator.rand(1, dimension) - 1);
        terminalVelocity     = velocityBound .* (2 * generator.rand(1, dimension) - 1);
        initialAcceleration  = accelerationBound .* (2 * generator.rand(1, dimension) - 1);
        terminalAcceleration = accelerationBound .* (2 * generator.rand(1, dimension) - 1);
    else
        kind = "onLimit";
        initialVelocity      = velocityBound .* sign(generator.randn(1, dimension));
        terminalVelocity     = velocityBound .* (2 * generator.rand(1, dimension) - 1);
        initialAcceleration  = accelerationBound .* (2 * generator.rand(1, dimension) - 1);
        terminalAcceleration = accelerationBound .* sign(generator.randn(1, dimension));
    end
    positionScale    = 10 ^ (generator.rand() * 3 - 1);                    % 0.1 .. 100
    initialPosition  = positionScale * (2 * generator.rand(1, dimension) - 1);
    terminalPosition = positionScale * (2 * generator.rand(1, dimension) - 1);

    initialState  = struct("time", 0, "position", initialPosition, "velocity", initialVelocity);
    terminalState = struct("position", terminalPosition, "velocity", terminalVelocity, "maximumTime", 1e6);
    limits        = struct("maximumVelocity", maximumVelocity, "maximumAcceleration", maximumAcceleration);
    if controlOrder == 3
        initialState.acceleration  = initialAcceleration;
        terminalState.acceleration = terminalAcceleration;
        limits.maximumJerk         = maximumJerk;
    end

    % Sometimes box the position so the position-bound check has work to do.
    % The box always contains both endpoints; the motion may still leave it.
    hasBox = generator.rand() < 0.15;
    if hasBox
        margin = positionScale * generator.rand() * 0.5;
        limits.positionLower = min(initialPosition, terminalPosition) - margin - 1e-3;
        limits.positionUpper = max(initialPosition, terminalPosition) + margin + 1e-3;
    end

    request = struct("kind", kind, "controlOrder", controlOrder, ...
        "initialState", initialState, "terminalState", terminalState, ...
        "limits", limits, "options", struct("SampleTime", 0.01), "hasBox", hasBox);
end

function result = solveFixed(request, finalTime)
    % Same request with a fixed final time.
    options           = request.options;
    options.TimeMode  = "fixed";
    options.FinalTime = finalTime;
    terminalState     = request.terminalState;
    terminalState.maximumTime = finalTime;
    result = ruckigEngine.solve(request.initialState, terminalState, request.limits, options);
end

function defects = checkReturnedMotion(caseIndex, result, request, endpointTolerance, limitTolerance, durationTolerance)
    % Re-derive everything from the polynomial record with plain code.
    defects    = struct("CaseIndex", {}, "Check", {}, "Detail", {});
    polynomial = result.Polynomial;
    startTimes = polynomial.SegmentStartTime_s(:);
    durations  = polynomial.SegmentDuration_s(:);
    if isscalar(durations)
        durations = repmat(durations, numel(startTimes), 1);
    end
    segmentCount = numel(startTimes);
    dimension    = numel(request.initialState.position);

    % Segment partition: positive durations that chain start to finish.
    if any(durations <= 0) || any(abs(startTimes(2:end) - (startTimes(1:end - 1) + durations(1:end - 1))) > durationTolerance)
        defects(end + 1) = makeDefect(caseIndex, "segmentPartition", "segment starts do not chain by their durations"); %#ok<AGROW>
        return;
    end
    finalTime = startTimes(end) + durations(end);
    if abs(finalTime - request.initialState.time - result.Duration) > durationTolerance
        defects(end + 1) = makeDefect(caseIndex, "durationMismatch", ...
            sprintf("Duration %.12g vs polynomial span %.12g", result.Duration, finalTime - request.initialState.time)); %#ok<AGROW>
    end
    if isfield(result.Options, "TimeMode") && string(result.Options.TimeMode) == "fixed" ...
            && abs(result.Duration - (result.Options.FinalTime - request.initialState.time)) > durationTolerance
        defects(end + 1) = makeDefect(caseIndex, "fixedTimeNotHonored", ...
            sprintf("requested %.12g got %.12g", result.Options.FinalTime, result.Duration)); %#ok<AGROW>
    end

    % Each record must be the time derivative of the one above it. The
    % coefficients are in local tau = (t - start) / duration, so
    % d/dt = (1 / duration) d/dtau.
    positionPower     = polynomial.positionPower_units;
    velocityPower     = polynomial.velocityPower_units_s;
    accelerationPower = polynomial.accelerationPower_units_s2;
    jerkPower         = polynomial.jerkPower_units_s3;
    derivativeError = max([ ...
        derivativeMismatch(positionPower, velocityPower, durations), ...
        derivativeMismatch(velocityPower, accelerationPower, durations), ...
        derivativeMismatch(accelerationPower, jerkPower, durations)]);
    if derivativeError > limitTolerance
        defects(end + 1) = makeDefect(caseIndex, "recordsNotDerivatives", ...
            sprintf("largest coefficient mismatch %.3g", derivativeError)); %#ok<AGROW>
    end

    % Endpoints from the record itself.
    startState = evaluateRecord(positionPower, velocityPower, accelerationPower, 1, 0);
    endState   = evaluateRecord(positionPower, velocityPower, accelerationPower, segmentCount, 1);
    endpointError = max([abs(startState.position - request.initialState.position), ...
        abs(startState.velocity - request.initialState.velocity), ...
        abs(endState.position - request.terminalState.position), ...
        abs(endState.velocity - request.terminalState.velocity)]);
    if request.controlOrder == 3
        endpointError = max([endpointError, ...
            abs(startState.acceleration - request.initialState.acceleration), ...
            abs(endState.acceleration - request.terminalState.acceleration)]);
    end
    if endpointError > endpointTolerance
        defects(end + 1) = makeDefect(caseIndex, "endpointMismatch", ...
            sprintf("largest endpoint error %.3g", endpointError)); %#ok<AGROW>
    end

    % Continuity at every segment join. Third order: position, velocity,
    % and acceleration. Second order: acceleration is the control and may jump.
    jumpError = 0;
    for segmentIndex = 1:segmentCount - 1
        before = evaluateRecord(positionPower, velocityPower, accelerationPower, segmentIndex, 1);
        after  = evaluateRecord(positionPower, velocityPower, accelerationPower, segmentIndex + 1, 0);
        jumpError = max([jumpError, abs(before.position - after.position), abs(before.velocity - after.velocity)]);
        if request.controlOrder == 3
            jumpError = max([jumpError, abs(before.acceleration - after.acceleration)]);
        end
    end
    if jumpError > endpointTolerance
        defects(end + 1) = makeDefect(caseIndex, "discontinuousAtSwitch", ...
            sprintf("largest jump %.3g", jumpError)); %#ok<AGROW>
    end

    % Limits. Velocity is at most quadratic in tau per segment, so its
    % extremes are at the ends or where acceleration crosses zero, and
    % acceleration is at most linear, so its extremes are at the ends.
    % Position is cubic; sample it densely and also check where velocity
    % crosses zero. Jerk is constant per segment.
    limits = result.Inputs.limits;
    velocityExcess     = 0;
    accelerationExcess = 0;
    jerkExcess         = 0;
    positionExcess     = 0;
    denseTau = linspace(0, 1, 201);
    for segmentIndex = 1:segmentCount
        for axisIndex = 1:dimension
            pc = squeeze(positionPower(segmentIndex, axisIndex, :)).';
            vc = squeeze(velocityPower(segmentIndex, axisIndex, :)).';
            ac = squeeze(accelerationPower(segmentIndex, axisIndex, :)).';
            jc = squeeze(jerkPower(segmentIndex, axisIndex, :)).';
            candidateTau = [0, 1, realRootsInUnit(ac)];
            velocityValues = polyvalAscending(vc, candidateTau);
            velocityExcess = max([velocityExcess, ...
                velocityValues - limits.velocityUpper(axisIndex), ...
                limits.velocityLower(axisIndex) - velocityValues]);
            accelerationValues = polyvalAscending(ac, [0, 1]);
            accelerationExcess = max([accelerationExcess, ...
                accelerationValues - limits.accelerationUpper(axisIndex), ...
                limits.accelerationLower(axisIndex) - accelerationValues]);
            if request.controlOrder == 3
                jerkValues = polyvalAscending(jc, [0, 1]);
                jerkExcess = max([jerkExcess, ...
                    jerkValues - limits.jerkUpper(axisIndex), ...
                    limits.jerkLower(axisIndex) - jerkValues]);
            end
            positionValues = polyvalAscending(pc, [denseTau, realRootsInUnit(vc)]);
            positionExcess = max([positionExcess, ...
                positionValues - limits.positionUpper(axisIndex), ...
                limits.positionLower(axisIndex) - positionValues]);
        end
    end
    if velocityExcess > limitTolerance
        defects(end + 1) = makeDefect(caseIndex, "velocityLimitExceeded", sprintf("by %.3g", velocityExcess)); %#ok<AGROW>
    end
    if accelerationExcess > limitTolerance
        defects(end + 1) = makeDefect(caseIndex, "accelerationLimitExceeded", sprintf("by %.3g", accelerationExcess)); %#ok<AGROW>
    end
    if jerkExcess > limitTolerance
        defects(end + 1) = makeDefect(caseIndex, "jerkLimitExceeded", sprintf("by %.3g", jerkExcess)); %#ok<AGROW>
    end
    if positionExcess > limitTolerance
        defects(end + 1) = makeDefect(caseIndex, "positionBoundExceeded", sprintf("by %.3g", positionExcess)); %#ok<AGROW>
    end

    % The sampled histories the engine returns must agree with the record.
    sampleError = 0;
    for sampleIndex = round(linspace(1, numel(result.time), min(25, numel(result.time))))
        sampleTime   = result.time(sampleIndex);
        segmentIndex = find(sampleTime >= startTimes - durationTolerance, 1, "last");
        tau = min(1, max(0, (sampleTime - startTimes(segmentIndex)) / durations(segmentIndex)));
        state = evaluateRecord(positionPower, velocityPower, accelerationPower, segmentIndex, tau);
        sampleError = max([sampleError, abs(state.position - result.position(sampleIndex, :)), ...
            abs(state.velocity - result.velocity(sampleIndex, :))]);
    end
    if sampleError > endpointTolerance
        defects(end + 1) = makeDefect(caseIndex, "historyDisagreesWithRecord", sprintf("by %.3g", sampleError)); %#ok<AGROW>
    end
end

function mismatch = derivativeMismatch(upperPower, lowerPower, durations)
    % Largest difference between d/dt of the upper record and the lower record.
    upperCount = size(upperPower, 3);
    lowerCount = size(lowerPower, 3);
    mismatch   = 0;
    for power = 1:max(upperCount - 1, lowerCount)
        if power <= upperCount - 1
            derived = upperPower(:, :, power + 1) * power ./ durations;
        else
            derived = zeros(size(upperPower, 1), size(upperPower, 2));
        end
        if power <= lowerCount
            stated = lowerPower(:, :, power);
        else
            stated = zeros(size(lowerPower, 1), size(lowerPower, 2));
        end
        mismatch = max(mismatch, max(abs(derived - stated), [], "all"));
    end
end

function state = evaluateRecord(positionPower, velocityPower, accelerationPower, segmentIndex, tau)
    % Evaluate one segment at local tau for every axis.
    dimension = size(positionPower, 2);
    state = struct("position", zeros(1, dimension), "velocity", zeros(1, dimension), "acceleration", zeros(1, dimension));
    for axisIndex = 1:dimension
        state.position(axisIndex)     = polyvalAscending(squeeze(positionPower(segmentIndex, axisIndex, :)).', tau);
        state.velocity(axisIndex)     = polyvalAscending(squeeze(velocityPower(segmentIndex, axisIndex, :)).', tau);
        state.acceleration(axisIndex) = polyvalAscending(squeeze(accelerationPower(segmentIndex, axisIndex, :)).', tau);
    end
end

function values = polyvalAscending(coefficients, tau)
    % Evaluate c(1) + c(2) tau + c(3) tau^2 + ... at every tau.
    values = zeros(size(tau));
    for power = numel(coefficients):-1:1
        values = values .* tau + coefficients(power);
    end
end

function tau = realRootsInUnit(coefficients)
    % Real roots of an ascending-power polynomial that lie inside (0, 1).
    coefficients = coefficients(:).';
    while ~isempty(coefficients) && coefficients(end) == 0
        coefficients(end) = [];
    end
    if numel(coefficients) < 2
        tau = zeros(1, 0);
        return;
    end
    candidates = roots(flip(coefficients));
    candidates = real(candidates(abs(imag(candidates)) < 1e-9));
    tau = candidates(candidates > 0 & candidates < 1).';
end

function defects = checkSymmetry(caseIndex, request, duration, generator, durationTolerance)
    % Axis order and coordinate mirroring must not change the earliest duration.
    defects   = struct("CaseIndex", {}, "Check", {}, "Detail", {});
    dimension = numel(request.initialState.position);

    order = generator.randperm(dimension);
    permutedResult = ruckigEngine.solve(permuteAxes(request.initialState, order), ...
        permuteAxes(request.terminalState, order), permuteAxes(request.limits, order), request.options);
    if ~permutedResult.Success || abs(permutedResult.Duration - duration) > max(durationTolerance, 1e-9 * duration)
        defects(end + 1) = makeDefect(caseIndex, "axisOrderChangesAnswer", ...
            sprintf("order [%s]: %s, duration %.12g vs %.12g", num2str(order), ...
            permutedResult.TerminationReason, permutedResult.Duration, duration)); %#ok<AGROW>
    end

    mirroredResult = ruckigEngine.solve(mirrorState(request.initialState), ...
        mirrorState(request.terminalState), mirrorLimits(request.limits), request.options);
    if ~mirroredResult.Success || abs(mirroredResult.Duration - duration) > max(durationTolerance, 1e-9 * duration)
        defects(end + 1) = makeDefect(caseIndex, "mirroringChangesAnswer", ...
            sprintf("%s, duration %.12g vs %.12g", mirroredResult.TerminationReason, ...
            mirroredResult.Duration, duration)); %#ok<AGROW>
    end
end

function record = permuteAxes(record, order)
    % Reorder every per-axis vector field of a state or limits struct.
    for name = string(fieldnames(record)).'
        value = record.(name);
        if isnumeric(value) && numel(value) == numel(order) && numel(order) > 1
            record.(name) = value(order);
        end
    end
end

function state = mirrorState(state)
    for name = ["position", "velocity", "acceleration"]
        if isfield(state, name)
            state.(name) = -state.(name);
        end
    end
end

function limits = mirrorLimits(limits)
    % Symmetric limits stay; a position box flips and swaps sides.
    if isfield(limits, "positionLower")
        lower = limits.positionLower;
        limits.positionLower = -limits.positionUpper;
        limits.positionUpper = -lower;
    end
end
