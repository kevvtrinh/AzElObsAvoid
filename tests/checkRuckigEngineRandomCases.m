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
%         given duration is an interval that moves with the duration). Some
%         requests have no longer duration at all: when an end state's
%         acceleration can only be reached by keeping its sign the whole
%         way, velocity is monotone and the displacement pins the duration
%         to a narrow window. A refused long fixed time is therefore only a
%         defect when the same LP search used for refusals finds a motion
%         at one of the refused durations; otherwise it is an observation.
%       * A too-short fixed time refused with a reason other than
%         fixedTimeBelowMinimum. Multi-axis requests can have a synchronized
%         minimum above every single-axis minimum, and the engine then
%         reports the synchronization failure instead.
%   - Position boxes only reject a motion; they do not steer it. Boxed
%     requests therefore often end as exactProfileValidationFailed, which
%     is counted, not flagged.
%   - Refusals are checked too (CheckRefusals). For every refused base
%     request an independent search looks for any motion between the two
%     states that stays inside every limit. Fix the duration and split it
%     into equal segments with one constant control (jerk, or acceleration
%     for second-order requests) per segment. Every node state is then a
%     linear function of those controls, so "is there a control that meets
%     every limit and the end state" is a linear program (LP): a set of
%     linear inequalities that a standard solver (linprog) answers exactly.
%     To search over durations, a second LP asks for the smallest single
%     amount ("slack") by which every constraint would have to be loosened
%     for a solution to exist. Zero slack means feasible at the nodes; the
%     slack grows continuously as the duration moves away from a feasible
%     one. That slack is sampled on a log grid around the request's own
%     time scale, its lowest dips are refined with a one-dimensional
%     minimizer, and every candidate duration is then re-verified exactly.
%     Near-bound requests often have a feasible duration window only a few
%     percent wide, which a grid alone would miss. When a motion exists the
%     refusal is bucketed:
%       * falseRefusal (defect): a motion exists, so the engine's claim
%         that the request is impossible is wrong.
%       * boxRefusedButSolvable (observation): a boxed request the engine
%         rejected after validation, although a motion inside the box
%         exists. Expected, because boxes do not steer the profile.
%     The LP search is a search, not a proof, so a refusal it cannot
%     overturn is reported as unchallenged, not as confirmed.
%
% INPUTS
%   - caseCount (positive integer scalar, optional; default 200)
%       Number of random base requests to generate.
%   - seed (nonnegative integer scalar, optional; default 1)
%       Random generator seed. The same seed always builds the same cases,
%       so any defect can be reproduced from its case index alone. Extra
%       requests follow the random ones, so their indices start at
%       caseCount + 1.
%   - options (scalar struct, optional; default struct())
%       Verbose (logical; default true) prints one line per defect.
%       ShortFactor (scalar; default 0.98) scales the earliest duration to
%       build the fixed-time request that must be refused.
%       LongFactor (scalar; default 1.5) scales the earliest duration to
%       build the fixed-time request that is expected to succeed.
%       CheckRefusals (logical; default true) runs the LP search on every
%       refused base request. It needs linprog and adds up to a second per
%       refused request.
%       ExtraRequests (cell array; default {}) hand-written requests to run
%       after the random ones, each a struct with initialState,
%       terminalState, and limits exactly as ruckigEngine.solve takes them.
%       They go through every check, so a known case can be replayed here.
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
%       Refusals: one table row per refused base request with the engine
%         reason, whether the LP search found a motion, at what duration,
%         and the bucket it landed in.
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
checkRefusals = resolveOption(options, "CheckRefusals", true);
extraRequests = resolveOption(options, "ExtraRequests", {});
validateattributes(extraRequests, {'cell'}, {});
randomCount = caseCount;
caseCount   = randomCount + numel(extraRequests);
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

% Build every request first so the random stream never depends on how the
% engine answered an earlier case. The symmetry check draws its axis
% permutations from a second stream for the same reason.
for caseIndex = 1:caseCount
    if caseIndex <= randomCount
        requests{caseIndex} = createRandomRequest(generator);
    else
        requests{caseIndex} = normalizeExtraRequest(extraRequests{caseIndex - randomCount});
    end
end
permutationGenerator = RandStream("mt19937ar", "Seed", seed + 1);

for caseIndex = 1:caseCount
    request = requests{caseIndex};
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
                % No longer duration solved. That is a defect only if a motion
                % exists at one of the refused durations.
                solvableFactor = NaN;
                if checkRefusals && exist("linprog", "file") == 2
                    for candidateFactor = [longFactor, 2, 3, 5, 10, 20]
                        if confirmDuration(request, candidateFactor * result.Duration, 100, limitTolerance)
                            solvableFactor = candidateFactor;
                            break;
                        end
                    end
                end
                if isfinite(solvableFactor)
                    caseDefects(end + 1) = makeDefect(caseIndex, "longFixedRefusedButSolvable", ...
                        sprintf("x%g refused (%s) but the LP search finds a motion there", ...
                        solvableFactor, longResult.TerminationReason)); %#ok<AGROW>
                else
                    caseObservations(end + 1) = makeDefect(caseIndex, "noLongerDurationFound", ...
                        sprintf("x%g through x20 all refused (%s) and the LP search finds none either", ...
                        longFactor, longResult.TerminationReason)); %#ok<AGROW>
                end
            end
        end

        % Reordering the axes or mirroring every coordinate must not change
        % the earliest duration.
        symmetryDefects = checkSymmetry(caseIndex, request, result.Duration, permutationGenerator, durationTolerance);
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

%% Section 3: Challenge Every Refusal

refusedIndex  = find(~caseTable.Success);
refusalTable  = table('Size', [numel(refusedIndex), 5], ...
    'VariableTypes', ["double", "string", "logical", "double", "string"], ...
    'VariableNames', ["CaseIndex", "Reason", "MotionFound", "Duration_s", "Bucket"]);
refusalTable.CaseIndex = refusedIndex;
refusalTable.Reason    = caseTable.Reason(refusedIndex);
refusalTable.Bucket(:) = "unchallenged";
if checkRefusals && ~isempty(refusedIndex)
    if exist("linprog", "file") ~= 2
        warning("checkRuckigEngineRandomCases:NoLinprog", "linprog is not available; refusals were not challenged.");
    else
        % Calibrate the search first: on solved cases it must find a motion,
        % otherwise its silence on refused cases means nothing.
        solvedIndex = find(caseTable.Success, 10);
        for caseIndex = solvedIndex(:).'
            [found, ~] = findMotionByLp(requests{caseIndex}, limitTolerance);
            if ~found
                caseDefect = makeDefect(caseIndex, "lpSearchMissedSolvedCase", ...
                    sprintf("engine solved it in %.6g s", caseTable.Duration_s(caseIndex)));
                defects(end + 1) = caseDefect; %#ok<AGROW>
                caseTable.DefectCount(caseIndex) = caseTable.DefectCount(caseIndex) + 1;
                if verbose
                    fprintf("case %4d  %-32s %s\n", caseIndex, caseDefect.Check, caseDefect.Detail);
                end
            end
        end
        for rowIndex = 1:numel(refusedIndex)
            caseIndex = refusedIndex(rowIndex);
            request   = requests{caseIndex};
            [found, foundDuration] = findMotionByLp(request, limitTolerance);
            refusalTable.MotionFound(rowIndex) = found;
            refusalTable.Duration_s(rowIndex)  = foundDuration;
            if ~found
                continue;
            end
            reason = caseTable.Reason(caseIndex);
            if request.hasBox && reason == "exactProfileValidationFailed"
                bucket = "boxRefusedButSolvable";
                observations(end + 1) = makeDefect(caseIndex, bucket, ...
                    sprintf("motion inside the box exists at %.6g s", foundDuration)); %#ok<AGROW>
            else
                bucket = "falseRefusal";
                caseDefect = makeDefect(caseIndex, bucket, ...
                    sprintf("%s: %s, but a motion exists at %.6g s", reason, ...
                    caseTable.Message(caseIndex), foundDuration));
                defects(end + 1) = caseDefect; %#ok<AGROW>
                caseTable.DefectCount(caseIndex) = caseTable.DefectCount(caseIndex) + 1;
                if verbose
                    fprintf("case %4d  %-32s %s\n", caseIndex, caseDefect.Check, caseDefect.Detail);
                end
            end
            refusalTable.Bucket(rowIndex) = bucket;
        end
    end
end

%% Section 4: Summarize

[reasonNames, ~, reasonIndex] = unique(caseTable.Reason);
reasonCounts = table(reasonNames, accumarray(reasonIndex, 1), ...
    'VariableNames', ["Reason", "Count"]);
report = struct("Seed", seed, "CaseCount", caseCount, "Cases", caseTable, ...
    "Requests", {requests}, "Defects", defects, "Observations", observations, ...
    "ReasonCounts", reasonCounts, "Refusals", refusalTable);

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
if ~isempty(refusalTable) && checkRefusals
    [bucketNames, ~, bucketIndex] = unique(refusalTable.Bucket);
    bucketCounts = accumarray(bucketIndex, 1);
    fprintf("Refusals challenged by LP search: %d, motion found for %d (%s)\n", ...
        height(refusalTable), nnz(refusalTable.MotionFound), ...
        strjoin(bucketNames(:).' + " " + string(bucketCounts(:).'), ", "));
elseif ~isempty(refusalTable)
    fprintf("Refusals: %d (not challenged; CheckRefusals is false)\n", height(refusalTable));
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

%% Section 5: Local Functions

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

function request = normalizeExtraRequest(supplied)
    % Give a hand-written request the same bookkeeping fields the random
    % generator produces, reading the control order from the fields given.
    validateattributes(supplied, {'struct'}, {'scalar'});
    if ~all(isfield(supplied, ["initialState", "terminalState", "limits"]))
        error("checkRuckigEngineRandomCases:InvalidExtraRequest", ...
            "Each extra request needs initialState, terminalState, and limits.");
    end
    request = struct("kind", "extra", "controlOrder", 2, ...
        "initialState", supplied.initialState, "terminalState", supplied.terminalState, ...
        "limits", supplied.limits, "options", struct("SampleTime", 0.01), "hasBox", false);
    if isfield(supplied.limits, "maximumJerk") && all(isfinite(supplied.limits.maximumJerk))
        request.controlOrder = 3;
    end
    if isfield(supplied.limits, "positionLower") || isfield(supplied.limits, "positionUpper")
        request.hasBox = true;
    end
    if ~isfield(request.terminalState, "maximumTime")
        request.terminalState.maximumTime = 1e6;
    end
    if ~isfield(request.initialState, "time")
        request.initialState.time = 0;
    end
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
        defects(end + 1) = makeDefect(caseIndex, "segmentPartition", "segment starts do not chain by their durations");
        return;
    end
    finalTime = startTimes(end) + durations(end);
    if abs(finalTime - request.initialState.time - result.Duration) > durationTolerance
        defects(end + 1) = makeDefect(caseIndex, "durationMismatch", ...
            sprintf("Duration %.12g vs polynomial span %.12g", result.Duration, finalTime - request.initialState.time));
    end
    if isfield(result.Options, "TimeMode") && string(result.Options.TimeMode) == "fixed" ...
            && abs(result.Duration - (result.Options.FinalTime - request.initialState.time)) > durationTolerance
        defects(end + 1) = makeDefect(caseIndex, "fixedTimeNotHonored", ...
            sprintf("requested %.12g got %.12g", result.Options.FinalTime, result.Duration));
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
            sprintf("largest coefficient mismatch %.3g", derivativeError));
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
            sprintf("largest endpoint error %.3g", endpointError));
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
            sprintf("largest jump %.3g", jumpError));
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
        defects(end + 1) = makeDefect(caseIndex, "velocityLimitExceeded", sprintf("by %.3g", velocityExcess));
    end
    if accelerationExcess > limitTolerance
        defects(end + 1) = makeDefect(caseIndex, "accelerationLimitExceeded", sprintf("by %.3g", accelerationExcess));
    end
    if jerkExcess > limitTolerance
        defects(end + 1) = makeDefect(caseIndex, "jerkLimitExceeded", sprintf("by %.3g", jerkExcess));
    end
    if positionExcess > limitTolerance
        defects(end + 1) = makeDefect(caseIndex, "positionBoundExceeded", sprintf("by %.3g", positionExcess));
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
        defects(end + 1) = makeDefect(caseIndex, "historyDisagreesWithRecord", sprintf("by %.3g", sampleError));
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
            permutedResult.TerminationReason, permutedResult.Duration, duration));
    end

    mirroredResult = ruckigEngine.solve(mirrorState(request.initialState), ...
        mirrorState(request.terminalState), mirrorLimits(request.limits), request.options);
    if ~mirroredResult.Success || abs(mirroredResult.Duration - duration) > max(durationTolerance, 1e-9 * duration)
        defects(end + 1) = makeDefect(caseIndex, "mirroringChangesAnswer", ...
            sprintf("%s, duration %.12g vs %.12g", mirroredResult.TerminationReason, ...
            mirroredResult.Duration, duration));
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


function value = expandToAxes(value, dimension)
    if isscalar(value)
        value = repmat(value, 1, dimension);
    end
end

function [found, foundDuration] = findMotionByLp(request, tolerance)
    % Search durations for any motion between the two states inside every
    % limit. The smallest violation slack over all axes is sampled on a
    % log grid spanning the request's own time scale, its dips are refined
    % with fminbnd, and each candidate is confirmed by an exact feasibility
    % solve. A run of equal grid slacks counts as one dip, so a plateau of
    % zeros at long durations (node-only feasibility that the exact check
    % may reject) cannot crowd out a narrow dip elsewhere.
    found         = false;
    foundDuration = NaN;
    segmentCount  = 100;
    if ~boundaryStatesInsideLimits(request)
        return;
    end
    referenceTime = requestTimeScale(request);
    logGrid       = log10(referenceTime) + linspace(-2, 1.5, 29);
    slackOnGrid   = arrayfun(@(logDuration) totalSlack(request, 10 ^ logDuration, segmentCount), logGrid);

    isDip    = [true, slackOnGrid(2:end - 1) <= slackOnGrid(1:end - 2) & slackOnGrid(2:end - 1) <= slackOnGrid(3:end), true];
    dipIndex = find(isDip);
    dipIndex = dipIndex([true, diff(dipIndex) > 1 | abs(diff(slackOnGrid(dipIndex))) > 1e-9]);
    [~, order] = sort(slackOnGrid(dipIndex));
    dipIndex   = dipIndex(order(1:min(6, numel(order))));
    fminOptions = optimset("TolX", 1e-4, "MaxFunEvals", 60, "Display", "off");
    for gridIndex = dipIndex
        candidateLog = logGrid(gridIndex);
        if slackOnGrid(gridIndex) > 1e-7
            lowerLog     = logGrid(max(1, gridIndex - 1));
            upperLog     = logGrid(min(numel(logGrid), gridIndex + 1));
            candidateLog = fminbnd(@(logDuration) totalSlack(request, 10 ^ logDuration, segmentCount), lowerLog, upperLog, fminOptions);
        end
        if confirmDuration(request, 10 ^ candidateLog, segmentCount, tolerance)
            found         = true;
            foundDuration = 10 ^ candidateLog;
            return;
        end
    end
end

function inside = boundaryStatesInsideLimits(request)
    % A boundary state outside the supplied bounds rules out every motion,
    % so the search has nothing to look for.
    dimension       = numel(request.initialState.position);
    velocityMax     = expandToAxes(request.limits.maximumVelocity, dimension);
    accelerationMax = expandToAxes(request.limits.maximumAcceleration, dimension);
    inside = true;
    for stateCell = {request.initialState, request.terminalState}
        state  = stateCell{1};
        inside = inside && all(abs(state.velocity) <= velocityMax + 1e-9);
        if request.controlOrder == 3
            inside = inside && all(abs(state.acceleration) <= accelerationMax + 1e-9);
        end
        if isfield(request.limits, "positionLower")
            inside = inside && all(state.position >= request.limits.positionLower - 1e-9) ...
                && all(state.position <= request.limits.positionUpper + 1e-9);
        end
    end
end

function referenceTime = requestTimeScale(request)
    % Rough time a motion of this size needs: travel at the velocity bound
    % plus the time to build and cancel velocity and acceleration.
    dimension       = numel(request.initialState.position);
    velocityMax     = expandToAxes(request.limits.maximumVelocity, dimension);
    accelerationMax = expandToAxes(request.limits.maximumAcceleration, dimension);
    displacement    = abs(request.terminalState.position - request.initialState.position);
    velocityChange  = abs(request.terminalState.velocity - request.initialState.velocity) + abs(request.initialState.velocity);
    referenceTime   = max(displacement ./ velocityMax + (velocityChange + velocityMax) ./ accelerationMax);
    if request.controlOrder == 3
        jerkMax       = expandToAxes(request.limits.maximumJerk, dimension);
        referenceTime = referenceTime + max(3 * accelerationMax ./ jerkMax);
    end
    referenceTime = max(referenceTime, 1e-3);
end

function slack = totalSlack(request, duration, segmentCount)
    % Sum over axes of the smallest violation slack at this duration.
    dimension = numel(request.initialState.position);
    slack     = 0;
    for axisIndex = 1:dimension
        axisProblem = extractAxisProblem(request, axisIndex, dimension);
        slack       = slack + axisSlack(axisProblem, request.controlOrder, duration, segmentCount);
    end
end

function ok = confirmDuration(request, duration, baseSegmentCount, tolerance)
    % Exact feasibility at one duration: a feasibility LP per axis whose
    % node bounds are tightened by any excursion the exact check finds.
    % Between nodes a segment can overshoot its node values by about
    % control * h^2 / 8, so the segment count grows with the duration to
    % keep that overshoot near one percent of the velocity bound. The count
    % is capped; a duration that would need more stays unconfirmed. A
    % single slack LP at the finer resolution filters out durations that
    % only looked feasible at the coarse one before the costlier solves.
    dimension = numel(request.initialState.position);
    ok = true;
    for axisIndex = 1:dimension
        axisProblem = extractAxisProblem(request, axisIndex, dimension);
        if request.controlOrder == 3
            targetStep = sqrt(0.08 * axisProblem.V / axisProblem.J);
        else
            targetStep = sqrt(0.08 * axisProblem.V / axisProblem.A);
        end
        segmentCount = min(400, max(baseSegmentCount, ceil(duration / targetStep)));
        if segmentCount > baseSegmentCount && axisSlack(axisProblem, request.controlOrder, duration, segmentCount) > 1e-7
            ok = false;
            return;
        end
        margins      = struct("velocity", 0, "position", 0);
        axisFeasible = false;
        for attempt = 1:3
            control = solveAxisLp(axisProblem, request.controlOrder, duration, segmentCount, margins);
            if isempty(control)
                break;
            end
            [axisFeasible, excess] = verifyAxisMotion(axisProblem, request.controlOrder, control, duration, tolerance);
            if axisFeasible || ~isfinite(excess.velocity) || ~isfinite(excess.position)
                break;
            end
            margins.velocity = margins.velocity + 1.5 * excess.velocity + 1e-9;
            margins.position = margins.position + 1.5 * excess.position + 1e-9;
        end
        if ~axisFeasible
            ok = false;
            return;
        end
    end
end

function lp = buildAxisLp(axisProblem, controlOrder, duration, segmentCount)
    % Node values of every state are affine in the piecewise-constant
    % control vector, so the limits become linear inequalities at the nodes
    % and the terminal state becomes equalities. Rows are grouped so the
    % caller can tighten velocity and position bounds by a margin.
    h = duration / segmentCount;
    lowerTriangle = tril(ones(segmentCount));
    if controlOrder == 3
        % a_k = a0 + h * sum(j_1..j_k); v_k = v_{k-1} + a_{k-1} h + j_k h^2/2;
        % p_k = p_{k-1} + v_{k-1} h + a_{k-1} h^2/2 + j_k h^3/6.
        accelerationMatrix = h * lowerTriangle;
        accelerationOffset = axisProblem.a0 * ones(segmentCount, 1);
        previousAcceleration = [zeros(1, segmentCount); accelerationMatrix(1:end - 1, :)];
        previousAccelerationOffset = [axisProblem.a0; accelerationOffset(1:end - 1)];
        velocityMatrix = lowerTriangle * (h * previousAcceleration + h ^ 2 / 2 * eye(segmentCount));
        velocityOffset = axisProblem.v0 + lowerTriangle * (h * previousAccelerationOffset);
        previousVelocityMatrix = [zeros(1, segmentCount); velocityMatrix(1:end - 1, :)];
        previousVelocityOffset = [axisProblem.v0; velocityOffset(1:end - 1)];
        positionMatrix = lowerTriangle * (h * previousVelocityMatrix + h ^ 2 / 2 * previousAcceleration + h ^ 3 / 6 * eye(segmentCount));
        positionOffset = axisProblem.p0 + lowerTriangle * (h * previousVelocityOffset + h ^ 2 / 2 * previousAccelerationOffset);
        lp.controlBound   = axisProblem.J;
        lp.equalityMatrix = [accelerationMatrix(end, :); velocityMatrix(end, :); positionMatrix(end, :)];
        lp.equalityValue  = [axisProblem.af - accelerationOffset(end); axisProblem.vf - velocityOffset(end); axisProblem.pf - positionOffset(end)];
        lp.fixedMatrix    = [accelerationMatrix; -accelerationMatrix];
        lp.fixedBound     = [axisProblem.A - accelerationOffset; axisProblem.A + accelerationOffset];
    else
        % v_k = v0 + h * sum(a_1..a_k); p_k = p_{k-1} + v_{k-1} h + a_k h^2/2.
        velocityMatrix = h * lowerTriangle;
        velocityOffset = axisProblem.v0 * ones(segmentCount, 1);
        previousVelocityMatrix = [zeros(1, segmentCount); velocityMatrix(1:end - 1, :)];
        previousVelocityOffset = [axisProblem.v0; velocityOffset(1:end - 1)];
        positionMatrix = lowerTriangle * (h * previousVelocityMatrix + h ^ 2 / 2 * eye(segmentCount));
        positionOffset = axisProblem.p0 + lowerTriangle * (h * previousVelocityOffset);
        lp.controlBound   = axisProblem.A;
        lp.equalityMatrix = [velocityMatrix(end, :); positionMatrix(end, :)];
        lp.equalityValue  = [axisProblem.vf - velocityOffset(end); axisProblem.pf - positionOffset(end)];
        lp.fixedMatrix    = zeros(0, segmentCount);
        lp.fixedBound     = zeros(0, 1);
    end
    lp.velocityMatrix = [velocityMatrix; -velocityMatrix];
    lp.velocityBound  = [axisProblem.V - velocityOffset; axisProblem.V + velocityOffset];
    lp.positionMatrix = [positionMatrix; -positionMatrix];
    lp.positionBound  = [axisProblem.pHi - positionOffset; -axisProblem.pLo + positionOffset];
    lp.segmentCount   = segmentCount;
    lp.scale          = max([1, abs(axisProblem.pf), abs(axisProblem.vf), axisProblem.V, axisProblem.A]);
end

function control = solveAxisLp(axisProblem, controlOrder, duration, segmentCount, margins)
    % Feasibility LP at one duration with the node bounds tightened by
    % margins. It minimizes the largest control magnitude, which returns
    % the gentlest control that fits and so keeps the excursions between
    % nodes small enough for the exact check to pass.
    lp = buildAxisLp(axisProblem, controlOrder, duration, segmentCount);
    inequalityMatrix = [lp.fixedMatrix; lp.velocityMatrix; lp.positionMatrix];
    inequalityBound  = [lp.fixedBound; lp.velocityBound - margins.velocity; lp.positionBound - margins.position];
    keep = isfinite(inequalityBound);
    % Variables: [control; peak]. |control_k| <= peak <= controlBound.
    peakMatrix = [inequalityMatrix(keep, :), zeros(nnz(keep), 1); ...
        eye(segmentCount), -ones(segmentCount, 1); ...
        -eye(segmentCount), -ones(segmentCount, 1)];
    peakBound  = [inequalityBound(keep); zeros(2 * segmentCount, 1)];
    objective  = [zeros(segmentCount, 1); 1];
    lowerBound = [-lp.controlBound * ones(segmentCount, 1); 0];
    upperBound = [lp.controlBound * ones(segmentCount, 1); lp.controlBound];
    lpOptions = optimoptions("linprog", "Display", "off", "Algorithm", "dual-simplex", "ConstraintTolerance", 1e-9);
    [solution, ~, exitFlag] = linprog(objective, peakMatrix, peakBound, ...
        [lp.equalityMatrix, zeros(numel(lp.equalityValue), 1)], lp.equalityValue, lowerBound, upperBound, lpOptions);
    control = [];
    if exitFlag == 1
        control = solution(1:segmentCount).';
    end
end

function slack = axisSlack(axisProblem, controlOrder, duration, segmentCount)
    % Smallest single slack that makes every node bound and the terminal
    % equalities hold, divided by the axis scale. Zero means feasible at
    % the nodes; it grows continuously with the duration otherwise.
    lp = buildAxisLp(axisProblem, controlOrder, duration, segmentCount);
    inequalityMatrix = [lp.fixedMatrix; lp.velocityMatrix; lp.positionMatrix];
    inequalityBound  = [lp.fixedBound; lp.velocityBound; lp.positionBound];
    keep = isfinite(inequalityBound);
    rowCount = nnz(keep);
    equalityCount = numel(lp.equalityValue);
    % Variables: [control; slack]. Every inequality and both sides of every
    % equality get the same slack, scaled so the slack is unitless.
    slackMatrix = [inequalityMatrix(keep, :), -lp.scale * ones(rowCount, 1); ...
        lp.equalityMatrix, -lp.scale * ones(equalityCount, 1); ...
        -lp.equalityMatrix, -lp.scale * ones(equalityCount, 1)];
    slackBound  = [inequalityBound(keep); lp.equalityValue; -lp.equalityValue];
    objective   = [zeros(segmentCount, 1); 1];
    lowerBound  = [-lp.controlBound * ones(segmentCount, 1); 0];
    upperBound  = [lp.controlBound * ones(segmentCount, 1); Inf];
    lpOptions = optimoptions("linprog", "Display", "off", "Algorithm", "dual-simplex", "ConstraintTolerance", 1e-9);
    [solution, slack, exitFlag] = linprog(objective, slackMatrix, slackBound, [], [], lowerBound, upperBound, lpOptions);
    if exitFlag ~= 1 || isempty(solution)
        slack = Inf;
    end
end

function axisProblem = extractAxisProblem(request, axisIndex, dimension)
    % One axis of the request with every bound spelled out per side.
    limits = request.limits;
    velocityMax     = expandToAxes(limits.maximumVelocity, dimension);
    accelerationMax = expandToAxes(limits.maximumAcceleration, dimension);
    axisProblem = struct();
    axisProblem.p0 = request.initialState.position(axisIndex);
    axisProblem.v0 = request.initialState.velocity(axisIndex);
    axisProblem.pf = request.terminalState.position(axisIndex);
    axisProblem.vf = request.terminalState.velocity(axisIndex);
    axisProblem.a0 = 0;
    axisProblem.af = 0;
    axisProblem.J  = Inf;
    if request.controlOrder == 3
        axisProblem.a0 = request.initialState.acceleration(axisIndex);
        axisProblem.af = request.terminalState.acceleration(axisIndex);
        axisProblem.J  = expandToAxes(limits.maximumJerk, dimension);
        axisProblem.J  = axisProblem.J(axisIndex);
    end
    axisProblem.V   = velocityMax(axisIndex);
    axisProblem.A   = accelerationMax(axisIndex);
    axisProblem.pLo = -Inf;
    axisProblem.pHi = Inf;
    if isfield(limits, "positionLower")
        axisProblem.pLo = limits.positionLower(axisIndex);
        axisProblem.pHi = limits.positionUpper(axisIndex);
    end
end


function [ok, excess] = verifyAxisMotion(axisProblem, controlOrder, control, duration, tolerance)
    % Exact check of the piecewise polynomial the control defines: limits on
    % every segment (acceleration at ends, velocity at ends and its vertex,
    % position densely plus where velocity is zero) and the terminal state.
    % Each quantity is judged against its own scale: velocity against the
    % velocity bound, acceleration against the acceleration bound, position
    % against the size of the positions involved. A large absolute position
    % must not loosen the velocity or acceleration checks.
    % excess reports how far velocity and position went past their bounds,
    % so the caller can tighten the LP node bounds and try again. A missed
    % terminal state cannot be fixed that way and is reported as Inf.
    segmentCount = numel(control);
    h = duration / segmentCount;
    p = axisProblem.p0; v = axisProblem.v0; a = axisProblem.a0;
    velocityScale     = max(1, axisProblem.V);
    accelerationScale = max(1, axisProblem.A);
    positionScale     = max([1, abs(axisProblem.p0), abs(axisProblem.pf), ...
        abs(axisProblem.pLo(isfinite(axisProblem.pLo))), abs(axisProblem.pHi(isfinite(axisProblem.pHi)))]);
    excess = struct("velocity", 0, "position", 0, "acceleration", 0);
    for k = 1:segmentCount
        if controlOrder == 3
            j = control(k);
        else
            j = 0;
            a = control(k);
        end
        % Local polynomials in t on [0, h], ascending powers.
        positionCoefficients = [p, v, a / 2, j / 6];
        velocityCoefficients = [v, a, j / 2];
        endAcceleration = a + j * h;
        excess.acceleration = max(excess.acceleration, max(abs([a, endAcceleration])) - axisProblem.A);
        velocityTimes = [0, h];
        if j ~= 0
            vertex = -a / j;
            if vertex > 0 && vertex < h
                velocityTimes(end + 1) = vertex; %#ok<AGROW>
            end
        end
        velocityValues  = polyvalAscending(velocityCoefficients, velocityTimes);
        excess.velocity = max(excess.velocity, max(abs(velocityValues)) - axisProblem.V);
        positionTimes   = [linspace(0, h, 33), realRootsInInterval(velocityCoefficients, h)];
        positionValues  = polyvalAscending(positionCoefficients, positionTimes);
        excess.position = max([excess.position, max(positionValues) - axisProblem.pHi, axisProblem.pLo - min(positionValues)]);
        p = polyvalAscending(positionCoefficients, h);
        v = polyvalAscending(velocityCoefficients, h);
        a = endAcceleration;
    end
    terminalOk = abs(p - axisProblem.pf) <= 1e-6 * positionScale && abs(v - axisProblem.vf) <= 1e-6 * velocityScale;
    if controlOrder == 3
        terminalOk = terminalOk && abs(a - axisProblem.af) <= 1e-6 * accelerationScale;
    end
    if ~terminalOk
        excess.velocity = Inf;
        excess.position = Inf;
    end
    ok = terminalOk && excess.acceleration <= tolerance * accelerationScale ...
        && excess.velocity <= tolerance * velocityScale && excess.position <= tolerance * positionScale;
end

function t = realRootsInInterval(coefficients, h)
    % Real roots of an ascending-power polynomial inside (0, h).
    coefficients = coefficients(:).';
    while ~isempty(coefficients) && coefficients(end) == 0
        coefficients(end) = [];
    end
    if numel(coefficients) < 2
        t = zeros(1, 0);
        return;
    end
    candidates = roots(flip(coefficients));
    candidates = real(candidates(abs(imag(candidates)) < 1e-9));
    t = candidates(candidates > 0 & candidates < h).';
end
