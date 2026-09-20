function [candidate, diagnostics, directMotion] = solve(seed, scene, motionRequest, directMotion)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics, directMotion] = bmtpEngine.solve( ...
%       seed, scene, motionRequest, directMotion)
%**************************************************************************
% PURPOSE
%   - Turn one proposed path into a smooth motion that respects motion limits.
%   - Adjust the curve and obstacle-separating boundaries in alternating
%     steps. The planner independently validates the resulting Bezier motion.
%**************************************************************************
% INPUTS
%   - seed (scalar struct)
%       position_units is N-by-2; tau strictly increases from zero to one.
%   - scene (scalar struct)
%       The exclusion geometry: regions_units (R-by-1 cell array, each one
%       finite convex N-by-2 polygon) and coverage (geometry provenance whose
%       ExactRegionCount and timed end-region metadata must be internally
%       consistent; optional ActiveTimeInterval_s limits each region to an
%       absolute physical motion-time interval). The public validator, not
%       this metadata check, establishes obstacle-coverage completeness.
%   - motionRequest (scalar struct)
%       What to plan: initialState and goalState (normalized position,
%       velocity, and acceleration with their times), limits (normalized
%       workspace, velocity, acceleration, and jerk bounds), and options
%       (resolved goal-time policy, sampling, work limits, and tolerances).
%   - directMotion (scalar struct)
%       Request-owned direct-motion product, or struct() before construction.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Stable motion record. Expected infeasibility returns Success = false
%       and may set OptimizerIterateUnavailable = true. Invalid input throws.
%   - diagnostics (scalar struct)
%       Solver, timing, coverage, motion, and plane-proof evidence.
%   - directMotion (scalar struct)
%       Unchanged input product or the direct motion and proof for reuse.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds. Derivatives use
%     units/s, units/s^2, and units/s^3. Polynomial powers use normalized time.
%**************************************************************************

%% Section 1: Validate And Create The Exclusion Representation
totalTimer = tic;
% Validate the request and resolve shared solver settings.
request = bmtpEngine.pipeline.createSolveRequest(seed, scene, motionRequest);
initialState  = request.InitialState;
goalState     = request.GoalState;
regions_units = request.Regions_units;
coverage      = request.Coverage;
limits        = request.Limits;
options       = request.Options;

% Create a kinematically feasible starting curve from the seed.
warmStart             = bmtpEngine.pipeline.createWarmStart(request);
degree                = request.Degree;
route_units           = warmStart.Route_units;
segmentCount          = warmStart.SegmentCount;
regionActiveBySegment = warmStart.RegionActiveBySegment;
candidate             = createEmptyCandidate(initialState);
diagnostics           = createEmptyDiagnostics(degree, segmentCount, numel(regions_units));
diagnostics.OriginalSeedSegmentCount = warmStart.OriginalSeedSegmentCount;
diagnostics.WarmRouteResampled       = warmStart.WarmRouteResampled;
diagnostics.ApplicablePairCount      = nnz(regionActiveBySegment);
diagnostics.MaximumAlternatingIterations = request.MaximumAlternatingIterations;
endRegions_units = cell(0, 1);
if isfield(coverage, 'EndRegions_units')
    endRegions_units = coverage.EndRegions_units;
end
[~, roundoffReserve_units] = bmtpEngine.validation.createCoordinateTolerances( ...
    route_units, limits.xInterval_units, limits.yInterval_units, ...
    regions_units, endRegions_units);
normalNormLimit     = 1 + 2 ^ 20 * eps;
obstacleTarget_units = normalNormLimit * options.CollisionClearanceTolerance_units + roundoffReserve_units;

%% Section 2: Solve The Direct Curve Or Alternating Convex Problem
% Earliest arrival tries the C3 jerk-limited chord. Fixed arrival retains
% the minimum-jerk quintic at the requested physical horizon.
preparedMotion         = struct('Success', false);
proof            = struct('Passed', false);
proofCache       = [];
if size(route_units, 1) == 2 && options.GoalTimeMode == "earliestArrival" && request.IsRest
    [controls_units, times_s, powers_units] = bmtpEngine.motion.createC3Chord( ...
        initialState.position_units, goalState.position_units, limits);
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(request, controls_units, times_s, powers_units);
    if preparedMotion.Success
        [proof, proofCache] = bmtpEngine.validation.checkFinalMotion(request, ...
            preparedMotion, roundoffReserve_units, ...
            obstacleTarget_units, proofCache, true);
    end
elseif options.GoalTimeMode == "fixedArrival"
    % A timed direct motion can pass even when the selected guide detours.
    % Check it before committing to the guide route. The chord and its
    % proof read no part of the seed, so one physical request that tries
    % several spatial guides constructs and proves them exactly once.
    directMotionKey = struct( ...
        'Regions_units', {request.Regions_units}, ...
        'Coverage',      request.Coverage, ...
        'InitialState',  initialState, ...
        'GoalState',     goalState, ...
        'Limits',        request.Limits, ...
        'Options',       request.Options, ...
        'Degree',        degree, ...
        'RoundoffReserve_units', roundoffReserve_units, ...
        'Target_units',  obstacleTarget_units);
    if isfield(directMotion, 'Key') && isequaln(directMotion.Key, directMotionKey)
        preparedMotion   = directMotion.PreparedMotion;
        proof      = directMotion.Proof;
        proofCache = directMotion.Cache;
    else
        [preparedMotion, proof, proofCache] = directFixedArrivalMotion( ...
            request, roundoffReserve_units, obstacleTarget_units);
        directMotion = struct( ...
            'Key',            directMotionKey, ...
            'PreparedMotion', preparedMotion, ...
            'Proof',    proof, ...
            'Cache',          proofCache);
    end
end
% A direct rest-to-rest earliest request with moving cells and no timed guide
% is the departure family: a proven zero-delay chord already supplies its
% first departure, otherwise the delayed chord is constructed from the same
% physical request. The seed label never selects this branch.
usesDepartureSchedule = size(route_units, 1) == 2 && ...
    options.GoalTimeMode == "earliestArrival" && request.IsRest && ...
    ~request.UsesTimeScopedSolver && isfield(coverage, 'ActiveTimeInterval_s');
if usesDepartureSchedule && ~(preparedMotion.Success && proof.Passed)
    [controls_units, times_s, powers_units, departure] = bmtpEngine.motion.createDelayedChord(request);
    diagnostics.DepartureSchedule = departure;
    if isempty(times_s)
        [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, struct( ...
            'Message',                  "No C3 chord departure fits the horizon.", ...
            'TerminationReason',        "noDepartureWindow", ...
            'OptimizerFeasible',        false, ...
            'FailureStage',             "timing", ...
            'FailureKind',              "noDepartureWindow", ...
            'AlternativeGuideEligible', true));
        return;
    end
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(request, controls_units, times_s, powers_units);
    if ~preparedMotion.Success
        [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, struct( ...
            'Message',                  preparedMotion.Message, ...
            'TerminationReason',        preparedMotion.TerminationReason, ...
            'OptimizerFeasible',        false, ...
            'FailureStage',             "reconstruction", ...
            'FailureKind',              string(preparedMotion.TerminationReason), ...
            'AlternativeGuideEligible', false));
        return;
    end
    [proof, proofCache] = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, roundoffReserve_units, obstacleTarget_units, proofCache);
    if ~proof.Passed
        geometryOnlyMiss = proof.CoverageMetadataConsistent && ...
            proof.WorkspacePassed && proof.DynamicsPassed && ...
            proof.ContinuityPassed && ...
            proof.VerifiedPairCount < proof.AllPairCount;
        if geometryOnlyMiss
            [candidate, diagnostics] = finishFailure( ...
                candidate, diagnostics, totalTimer, struct( ...
                'Message',                  "The direct departure family is obstructed " + ...
                                            "on this route.", ...
                'TerminationReason',        "departureUncertified", ...
                'OptimizerFeasible',        false, ...
                'FailureStage',             "proposal", ...
                'FailureKind',              "directDepartureObstructed", ...
                'AlternativeGuideEligible', true));
            return;
        end
        [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, struct( ...
            'Message',                  "The C3 departure proposal did not pass its proof.", ...
            'TerminationReason',        "departureUncertified", ...
            'OptimizerFeasible',        false, ...
            'FailureStage',             "certification", ...
            'FailureKind',              "directDepartureCertificateUnavailable", ...
            'AlternativeGuideEligible', false));
        return;
    end
end
if preparedMotion.Success && proof.Passed
    diagnostics.Converged                = true;
    diagnostics.OptimizerSpanCount       = 0;
    diagnostics.SegmentCount             = numel(preparedMotion.SegmentTime_s);
else
    if options.GoalTimeMode == "earliestArrival" && ~isfield(coverage, 'ActiveTimeInterval_s')
        [alternatingResult, diagnostics] = bmtpEngine.optimization.solveActivePairTrajectory( ...
            request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units);
    else
        % Only a variable clock needs the dedicated solver that rebuilds
        % obstacle/time overlap after every duration change. A given
        % clock uses the mature fixed-duration alternating SOCP; its active
        % intervals are already exact and do not move between iterations.
        usesTimedSolver = request.UsesVariableClock;
        if usesTimedSolver
            [alternatingResult, diagnostics] = bmtpEngine.optimization.solveTimedAlternatingTrajectory( ...
                request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units);
            if alternatingResult.Success
                [timedMotion, diagnostics] = bmtpEngine.pipeline.refineTimedTravel( ...
                    request, alternatingResult, diagnostics, ...
                    obstacleTarget_units, roundoffReserve_units);
                alternatingResult = timedMotion;
            end
        else
            [alternatingResult, diagnostics] = bmtpEngine.optimization.solveAlternatingTrajectory( ...
                request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units);
        end
    end
    if ~alternatingResult.Success
        % The optimizer produced no collision-free iterate for this guide. The
        % typed flag, not the explanatory reason, admits another guide upstream.
        candidate.OptimizerIterateUnavailable = true;
        stage         = string(optionalField(alternatingResult, "FailureStage", "optimization"));
        kind          = string(optionalField(alternatingResult, "FailureKind", "optimizerIterateUnavailable"));
        guideEligible = logical(optionalField(alternatingResult, "AlternativeGuideEligible", false));
        [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, struct( ...
            'Message',                  "No optimized collision-free iterate was found. " + ...
                                        alternatingResult.SolverMessage, ...
            'TerminationReason',        "noOptimizedFeasibleIterate", ...
            'OptimizerFeasible',        false, ...
            'FailureStage',             stage, ...
            'FailureKind',              kind, ...
            'AlternativeGuideEligible', guideEligible));
        return;
    end
    if isfield(alternatingResult, 'PreparedMotion') && alternatingResult.PreparedMotion.Success && ...
            isfield(alternatingResult, 'Proof') && alternatingResult.Proof.Passed
        % The proof belongs to this prepared motion, not to the raw controls above.
        preparedMotion = alternatingResult.PreparedMotion;
        proof    = alternatingResult.Proof;
    else
        % Endpoint correction and export can increase the derivative bounds.
        preparedMotion   = bmtpEngine.pipeline.prepareFinalMotion( ...
            request, alternatingResult.ControlPoint_units, alternatingResult.SegmentTime_s);
        proof      = struct('Passed', false);
        proofCache = [];
    end
end

%% Section 3: Prepare And Check The Final Motion
diagnostics.DilationScale = preparedMotion.DilationScale;
% Return reconstruction failure without proof because no complete motion exists to prove.
if ~preparedMotion.Success
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, struct( ...
        'Message',                  preparedMotion.Message, ...
        'TerminationReason',        preparedMotion.TerminationReason, ...
        'OptimizerFeasible',        true, ...
        'FailureStage',             "reconstruction", ...
        'FailureKind',              string(preparedMotion.TerminationReason), ...
        'AlternativeGuideEligible', false));
    return;
end

% Prove every final curve-region pair; sampled clearance alone is insufficient.
if ~proof.Passed
    [proof, proofCache] = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, roundoffReserve_units, obstacleTarget_units, proofCache);
end
% A safe curved span need not admit one affine separator. Exact subdivision
% can expose its clearance without moving the curve or changing tolerances.
for refinementIndex = 1:10
    if proof.Passed || ~proof.WorkspacePassed || ...
            ~proof.DynamicsPassed || ~proof.ContinuityPassed
        break
    end
    failedPairs   = ~reshape([proof.Planes.Verified], size(proof.Planes)) & ...
        proof.RegionActiveBySegment;
    splitMask     = any(failedPairs, 2);
    splitFraction = repmat(0.5, numel(splitMask), 1);
    if ~any(splitMask)
        break
    end
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(request, ...
        preparedMotion.ControlPoint_units, preparedMotion.SegmentTime_s, ...
        preparedMotion.GivenPower_units, splitMask, splitFraction);
    [proof, proofCache] = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, roundoffReserve_units, obstacleTarget_units, proofCache);
end
diagnostics.SegmentCount = numel(preparedMotion.SegmentTime_s);
diagnostics.FinalCollisionPairCount = ...
    proof.AllPairCount - proof.VerifiedPairCount;
diagnostics.MotionProof = preparedMotion.MotionProof;
diagnostics.SeparationProof  = proof;
candidate.SeparationProof    = proof;

% Convert the checked curve to the public motion format and sample it.
candidate                     = bmtpEngine.pipeline.createMotionOutput(candidate, request, preparedMotion);
candidate.OptimizerFeasible   = true;
% Reject optimizer output that fails the independent proof even when the numerical solver reported success.
if ~proof.Passed
    failureStage              = "certification";
    failureKind               = "planeCertificateUnavailable";
    alternativeGuideEligible = false;
    geometryProofPassed = proof.CoverageMetadataConsistent && ...
        proof.VerifiedPairCount == proof.AllPairCount;
    if geometryProofPassed && proof.WorkspacePassed && ...
            ~proof.DynamicsPassed && proof.ContinuityPassed
        % The curve and its geometry proof are intact, but this clock
        % is too short for the returned motion. Another physical clock may
        % be tried without accepting or repairing this candidate.
        failureStage              = "timing";
        failureKind               = "kinematicCertificateUnavailable";
        alternativeGuideEligible = true;
    elseif proof.WorkspacePassed && proof.DynamicsPassed && ...
            ~proof.ContinuityPassed
        failureStage = "reconstruction";
        failureKind  = "continuityCertificateUnavailable";
    elseif ~proof.WorkspacePassed
        failureKind = "workspaceCertificateUnavailable";
    end
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, struct( ...
        'Message',                  "The optimized motion failed its continuous collision, " + ...
                                    "dynamics, or C3 continuity proof.", ...
        'TerminationReason',        "planeCertificateUnavailable", ...
        'OptimizerFeasible',        true, ...
        'FailureStage',             failureStage, ...
        'FailureKind',              failureKind, ...
        'AlternativeGuideEligible', alternativeGuideEligible));
    return;
end

%% Section 4: Finalize The Directly Proven Candidate
[candidate.Message, candidate.TerminationReason] = deal( ...
    "A directly proven BMTP trajectory was found.", "goalReached");
[candidate.Success, diagnostics.Accepted]        = deal(true);
candidate.FailureStage                           = "";
candidate.FailureKind                            = "";
candidate.AlternativeGuideEligible               = false;
diagnostics.ElapsedTime_s                        = toc(totalTimer);
end

%% Section 5: Local Functions
function [preparedMotion, proof, cache] = directFixedArrivalMotion(request, roundoffReserve_units, target_units)
    % Construct and prove the direct chord at the given horizon.
    degree       = request.Degree;
    initialState = request.InitialState;
    goalState    = request.GoalState;
    proof = struct('Passed', false);
    cache       = [];
    % The rest-to-rest chord is the quintic smoothstep 10t^3-15t^4+6t^5.
    fraction = bmtpEngine.motion.powerToBernstein([0; 0; 0; 10; -15; 6], degree);
    controls_units = initialState.position_units + ...
        fraction .* (goalState.position_units - initialState.position_units);
    if ~request.IsRest
        h = request.MotionHorizon_s;
        power = [initialState.position_units; ...
            h * initialState.velocity_units_s; ...
            h ^ 2 * initialState.acceleration_units_s2 / 2; ...
            zeros(3, 2)];
        residual = [goalState.position_units - sum(power, 1); ...
            h * goalState.velocity_units_s - power(2, :) - 2 * power(3, :); ...
            h ^ 2 * goalState.acceleration_units_s2 - 2 * power(3, :)];
        power(4:6, :) = [1 1 1; 3 4 5; 6 12 20] \ residual;
        controls_units = bmtpEngine.motion.powerToBernstein(power, degree);
    end
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(request, ...
        reshape(controls_units, 1, degree + 1, 2), request.MotionHorizon_s);
    if preparedMotion.Success
        [proof, cache] = bmtpEngine.validation.checkFinalMotion(request, ...
            preparedMotion, roundoffReserve_units, target_units, cache, true);
    end
end

function candidate = createEmptyCandidate(initialState)
    % Use the same candidate fields on success and failure.
    dimensionCount = numel(initialState.position_units);
    candidate                                          = struct();
    candidate.Success                                  = false;
    candidate.OptimizerFeasible                        = false;
    candidate.OptimizerIterateUnavailable              = false;
    candidate.AlternativeGuideEligible                 = false;
    candidate.FailureStage                             = "notRun";
    candidate.FailureKind                              = "notRun";
    candidate.Message                                  = "The BMTP engine was not run.";
    candidate.TerminationReason                        = "notRun";
    candidate.ArrivalTime_s                            = NaN;
    candidate.TrajectoryDuration_s                     = NaN;
    candidate.MotionLength_units                       = Inf;
    candidate.IntegratedSquaredJerk_units2_s5          = Inf;
    candidate.MaximumConstraintViolation               = Inf;
    candidate.time_s                                   = zeros(0, 1);
    candidate.position_units                           = zeros(0, dimensionCount);
    candidate.velocity_units_s                         = zeros(0, dimensionCount);
    candidate.acceleration_units_s2                    = zeros(0, dimensionCount);
    candidate.jerk_units_s3                            = zeros(0, dimensionCount);
    candidate.Polynomial                               = struct();
    candidate.SeparationProof                         = struct();
end

function value = optionalField(record, name, defaultValue)
    % Read an optional field or use its default.
    value = defaultValue;
    if isfield(record, name) && ~isempty(record.(name))
        value = record.(name);
    end
end

function diagnostics = createEmptyDiagnostics(degree, segmentCount, regionCount)
    % Initialize solver, timing, and proof diagnostics.
    diagnostics = struct( ...
        "Accepted",                          false, ...
        "Degree",                            degree, ...
        "OriginalSeedSegmentCount",           segmentCount, ...
        "WarmRouteResampled",                 false, ...
        "OptimizerSpanCount",                 segmentCount, ...
        "SegmentCount",                       2 * segmentCount, ...
        "ExactRegionCount",                   regionCount, ...
        "IterationCount",                     0, ...
        "Converged",                          false, ...
        "ApplicablePairCount",                segmentCount * regionCount, ...
        "TrajectorySocpCount",                0, ...
        "FinalCollisionPairCount",            0, ...
        "PlaneSocpCount",                     0, ...
        "DilationScale",                      NaN, ...
        "MotionProof",                  struct(), ...
        "SeparationProof",                   struct(), ...
        "SolverMessage",                      "", ...
        "ElapsedTime_s",                      0, ...
        "ConicSolver",                        bmtpEngine.optimization.accumulateConicDiagnostics());
end

function [candidate, diagnostics] = finishFailure(candidate, diagnostics, timer, failure)
    % Return a failure without fabricating motion data. The failure record
    % declares Message, TerminationReason, OptimizerFeasible, FailureStage,
    % FailureKind, and AlternativeGuideEligible.
    candidate.Message                    = failure.Message;
    candidate.TerminationReason          = failure.TerminationReason;
    candidate.OptimizerFeasible          = failure.OptimizerFeasible;
    candidate.FailureStage               = string(failure.FailureStage);
    candidate.FailureKind                = string(failure.FailureKind);
    candidate.AlternativeGuideEligible   = logical(failure.AlternativeGuideEligible);
    diagnostics.FailureStage             = candidate.FailureStage;
    diagnostics.FailureKind              = candidate.FailureKind;
    diagnostics.AlternativeGuideEligible = candidate.AlternativeGuideEligible;
    [diagnostics.Accepted, diagnostics.ElapsedTime_s] = deal(false, toc(timer));
end
