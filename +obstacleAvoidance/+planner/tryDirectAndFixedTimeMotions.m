function exactMotionSet = tryDirectAndFixedTimeMotions( ...
        initialState, goalState, limits, options, scene, stageTiming)
%% Section 0: Header & Readme
% SYNTAX
%   defaults = obstacleAvoidance.planner.tryDirectAndFixedTimeMotions()
%   exactMotionSet = obstacleAvoidance.planner.tryDirectAndFixedTimeMotions( ...
%       initialState, goalState, limits, options, scene, stageTiming)
%
% PURPOSE
%   - Try exact direct and fixed-clock lateral motions before route search,
%     and expose whether either passed the full trajectory check.
%
% INPUTS
%   - initialState, goalState, limits, options: motion constraints.
%   - scene (scalar prepared-scene struct)
%       Prepared obstacles shared with later graph and validation stages.
%   - stageTiming (scalar timing struct)
%       Accumulated planner stage timings before exact motion work.
%
% OUTPUTS
%   - exactMotionSet (scalar struct)
%       Direct and excursion candidates, checks, diagnostics, timing, and an
%       explicit fully validated fast-path record. A zero-input call returns
%       stable not-attempted diagnostics.
%
% UNITS
%   - Position and path length are degrees; time is seconds.
%

%% Section 1: Create Stable Attempt Records

[~, excursionDiagnostics] = ...
    obstacleAvoidance.planner.tryFixedTimeDetour();
exactMotionSet = struct( ...
    "DirectAttempt", directAttemptTemplate(), ...
    "ExcursionCandidate", struct(), ...
    "ExcursionDiagnostics", excursionDiagnostics, ...
    "ExcursionElapsedTime_s", 0, ...
    "ExcursionIsValidated", false, ...
    "ExcursionSeed", obstacleAvoidance.search.createEmptyPathGuess(), ...
    "FastPath", emptyFastPath(), ...
    "StageTiming", struct());
if nargin == 0
    return;
end
preparedObstacles = scene.preparedObstacles;

%% Section 2: Create And Check The Exact Direct Motion
motionTimer = tic;
directCandidate = bmtpEngine.createDirectMotion( ...
    initialState, goalState, limits, options);
directElapsedTime_s = toc(motionTimer);
stageTiming.MotionSolvingElapsedTime_s = ...
    stageTiming.MotionSolvingElapsedTime_s + directElapsedTime_s;
[directCandidate, directValidation, directValidationTime_s, stageTiming] = ...
    obstacleAvoidance.planner.checkCandidateMotion( ...
    directCandidate, preparedObstacles, initialState, ...
    goalState, limits, options, stageTiming, "");
directAttempt = recordDirectAttempt(directCandidate, directValidation, ...
    directElapsedTime_s, directValidationTime_s);
exactMotionSet.DirectAttempt = directAttempt;
if directValidation.Passed
    exactMotionSet.FastPath = createFastPath( ...
        directCandidate, directValidation, directAttempt, ...
        directElapsedTime_s, createDirectSeed( ...
        initialState, goalState, directCandidate.TrajectoryDuration_s), ...
        "An exact direct rest-to-rest motion passed independent validation.");
    exactMotionSet.StageTiming = stageTiming;
    return;
end
exactMotionSet.DirectAttempt.FallbackContinued = true;

%% Section 3: Create And Check The Fixed-Clock Excursion

% The excursion constructor already validates its motion.
% Account for that validation separately from construction time.
motionTimer = tic;
[excursionCandidate, excursionDiagnostics] = ...
    obstacleAvoidance.planner.tryFixedTimeDetour( ...
    directCandidate, preparedObstacles, initialState, goalState, ...
    limits, options, directValidation);
excursionElapsedTime_s = toc(motionTimer);
[stageTiming, excursionSolvingTime_s] = accountConstructorValidation( ...
    stageTiming, excursionElapsedTime_s, excursionDiagnostics);
exactMotionSet.ExcursionCandidate = excursionCandidate;
exactMotionSet.ExcursionDiagnostics = excursionDiagnostics;
exactMotionSet.ExcursionElapsedTime_s = excursionSolvingTime_s;
if excursionDiagnostics.Success && excursionCandidate.Validation.Passed
    excursionSeed = createMotionSeed( ...
        excursionCandidate, "fixedClockLateralExcursion");
    exactMotionSet.ExcursionIsValidated = true;
    exactMotionSet.ExcursionSeed = excursionSeed;
    if options.GoalTimeMode == "earliestArrival"
        exactMotionSet.FastPath = createFastPath( ...
            excursionCandidate, excursionCandidate.Validation, ...
            excursionDiagnostics, excursionSolvingTime_s, excursionSeed, ...
            "A fixed-clock lateral excursion attained the physical time floor.");
    end
end
exactMotionSet.StageTiming = stageTiming;
end

%% Section 4: Local Functions

function record = directAttemptTemplate()
% Initialize direct-motion diagnostics.
record = struct("Identifier", "analyticRestToRest", ...
    "Attempted", false, "ProfileCreated", false, ...
    "ValidationAttempted", false, "ValidationPassed", false, ...
    "CollisionFree", false, "CollisionResolved", false, ...
    "FallbackContinued", false, "KernelTerminationReason", "notRun", ...
    "TerminationReason", "notRun", ...
    "Message", "The exact direct motion was not attempted.", ...
    "ElapsedTime_s", 0, "ValidationElapsedTime_s", 0, ...
    "TrajectoryDuration_s", NaN, "MotionLength_deg", NaN, ...
    "MinimumAxisDuration_s", [NaN NaN], ...
    "StraightProgressMinimumDuration_s", NaN, ...
    "UsedStraightProgress", false);
end

function record = recordDirectAttempt(candidate, validation, elapsedTime_s, ...
        validationElapsedTime_s)
% Record construction and validation results.
record = directAttemptTemplate();
if isfield(candidate, "SolverDiagnostics") && ...
        isfield(candidate.SolverDiagnostics, "Identifier")
    record.Identifier = candidate.SolverDiagnostics.Identifier;
end
record.Attempted = true;
record.ProfileCreated = ~isempty(candidate.time_s);
record.ValidationAttempted = record.ProfileCreated;
record.ValidationPassed = validation.Passed;
record.CollisionFree = validation.CollisionFree;
record.CollisionResolved = validation.CollisionResolved;
record.KernelTerminationReason = candidate.TerminationReason;
record.TerminationReason = candidate.TerminationReason;
record.Message = candidate.Message;
record.ElapsedTime_s = elapsedTime_s;
record.ValidationElapsedTime_s = validationElapsedTime_s;
for name = ["TrajectoryDuration_s", "MotionLength_deg", ...
        "MinimumAxisDuration_s", "StraightProgressMinimumDuration_s", ...
        "UsedStraightProgress"]
    record.(name) = candidate.(name);
end
if record.ValidationAttempted && ~validation.Passed
    record.TerminationReason = "directValidationFailed";
    record.Message = strtrim(candidate.Message + " " + validation.Message);
elseif validation.Passed
    record.TerminationReason = "goalReached";
    record.Message = validation.Message;
end
end

function seed = createDirectSeed(initialState, goalState, duration_s)
% Create the two-endpoint direct seed.
position_deg = [initialState.position_deg; goalState.position_deg];
seed = obstacleAvoidance.search.createEmptyPathGuess();
seed.Index = 1;
seed.Source = "directRestToRest";
[seed.position_deg, seed.tau] = deal(position_deg, [0; 1]);
seed.EstimatedDuration_s = duration_s;
seed.Length_deg = norm(diff(position_deg, 1, 1));
end

function seed = createMotionSeed(candidate, source)
% Store the accepted curve as the seed, not the blocked straight chord.
time_s = double(candidate.time_s(:));
duration_s = candidate.TrajectoryDuration_s;
tau = (time_s - time_s(1)) / duration_s;
seed = obstacleAvoidance.search.createEmptyPathGuess();
seed.Index = 1;
seed.Source = string(source);
seed.ParameterBasis = "normalizedTime";
[seed.position_deg, seed.tau] = deal(candidate.position_deg, tau);
seed.EstimatedDuration_s = duration_s;
seed.Length_deg = candidate.MotionLength_deg;
end

function [stageTiming, motionSolvingTime_s] = accountConstructorValidation( ...
        stageTiming, constructorElapsedTime_s, diagnostics)
% Separate validation time from constructor time.
validationElapsedTime_s = 0;
collisionElapsedTime_s = 0;
if isfield(diagnostics, "ValidationElapsedTime_s")
    validationElapsedTime_s = double(diagnostics.ValidationElapsedTime_s);
end
if isfield(diagnostics, "CollisionCheckingElapsedTime_s")
    collisionElapsedTime_s = double(diagnostics.CollisionCheckingElapsedTime_s);
end
tolerance_s = 256 * eps(max(1, constructorElapsedTime_s));
if validationElapsedTime_s > constructorElapsedTime_s + tolerance_s || ...
        collisionElapsedTime_s > validationElapsedTime_s + tolerance_s
    error("tryDirectAndFixedTimeMotions:InvalidConstructorTiming", ...
        "Nested validation timing exceeds its constructor or validation total.");
end
validationElapsedTime_s = min(validationElapsedTime_s, ...
    constructorElapsedTime_s);
collisionElapsedTime_s = min(collisionElapsedTime_s, ...
    validationElapsedTime_s);
motionSolvingTime_s = constructorElapsedTime_s - validationElapsedTime_s;
stageTiming.MotionSolvingElapsedTime_s = ...
    stageTiming.MotionSolvingElapsedTime_s + motionSolvingTime_s;
stageTiming.CollisionCheckingElapsedTime_s = ...
    stageTiming.CollisionCheckingElapsedTime_s + collisionElapsedTime_s;
stageTiming.FinalValidationElapsedTime_s = ...
    stageTiming.FinalValidationElapsedTime_s + ...
    validationElapsedTime_s - collisionElapsedTime_s;
end

function fastPath = emptyFastPath()
% Initialize an unavailable fast-path result.
fastPath = struct("Available", false, "Candidate", struct(), ...
    "Validation", struct(), "AttemptDetails", struct(), ...
    "ElapsedTime_s", 0, "Seed", obstacleAvoidance.search.createEmptyPathGuess(), ...
    "Message", "");
end

function fastPath = createFastPath( ...
        candidate, validation, details, elapsedTime_s, seed, message)
% Return a fast path only after full validation.
fastPath = struct("Available", true, "Candidate", candidate, ...
    "Validation", validation, "AttemptDetails", details, ...
    "ElapsedTime_s", elapsedTime_s, "Seed", seed, "Message", message);
end
