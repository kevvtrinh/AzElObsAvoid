function [candidate, diagnostics] = solve(seed, regions_units, coverage, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = bmtpEngine.solve(seed, regions_units, ...
%       coverage, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Turn one proposed path into a smooth motion that respects motion limits.
%   - Adjust the curve and obstacle-separating boundaries in alternating
%     steps. The planner independently validates the resulting Bezier motion.
%**************************************************************************
% INPUTS
%   - seed (scalar struct)
%       position_units is N-by-2; tau strictly increases from zero to one.
%   - regions_units (R-by-1 cell array)
%       Each cell contains one finite convex N-by-2 exclusion polygon.
%   - coverage (scalar struct)
%       Requires Passed. Optional ActiveTimeInterval_s limits each region to
%       an absolute physical motion-time interval.
%   - initialState (scalar struct)
%       Normalized initial position, velocity, and acceleration.
%   - goalState (scalar struct)
%       Normalized goal position, velocity, and acceleration.
%   - limits (scalar struct)
%       Normalized workspace, velocity, acceleration, and jerk bounds.
%   - options (scalar struct)
%       Resolved goal-time policy, sampling, work limits, and tolerances.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Stable motion record. Expected infeasibility returns Success = false
%       and may set OptimizerIterateUnavailable = true. Invalid input throws.
%   - diagnostics (scalar struct)
%       Solver, timing, coverage, motion, and plane-certificate evidence.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds. Derivatives use
%     units/s, units/s^2, and units/s^3. Polynomial powers use normalized time.
%**************************************************************************

%% Section 1: Validate And Create The Exclusion Representation
totalTimer = tic;
% Validate the request and resolve shared solver settings.
request = bmtpEngine.pipeline.createSolveRequest(seed, regions_units, coverage, initialState, goalState, limits, options);
initialState = request.InitialState;
goalState    = request.GoalState;

% Create a kinematically feasible starting curve from the seed.
warmStart             = bmtpEngine.pipeline.createWarmStart(request);
degree                = request.Degree;
route_units           = warmStart.Route_units;
segmentCount          = warmStart.SegmentCount;
regionActiveBySegment = warmStart.RegionActiveBySegment;
candidate             = createEmptyCandidate(seed, initialState);
diagnostics           = createEmptyDiagnostics(degree, segmentCount, numel(regions_units));
diagnostics.OriginalSeedSegmentCount = warmStart.OriginalSeedSegmentCount;
diagnostics.WarmRouteResampled       = warmStart.WarmRouteResampled;
diagnostics.ApplicablePairCount      = nnz(regionActiveBySegment);
if isfield(coverage, 'BreakTime_s')
    diagnostics.Identifier = "bmtpTimeCellsDegree" + string(degree);
end
if options.GoalTimeMode == "fixedArrival"
    diagnostics.ConstraintRepresentation = "fixedClockElasticSocp";
end
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
certificate            = struct('Passed', false);
certificateCache       = [];
analyticIdentifier     = "minimumJerkQuintic";
analyticRepresentation = "analyticQuinticClock";
if size(route_units, 1) == 2 && options.GoalTimeMode == "earliestArrival" && request.IsRest
    [controls_units, times_s, powers_units] = bmtpEngine.motion.createC3Chord( ...
        initialState.position_units, goalState.position_units, limits);
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(request, controls_units, times_s, powers_units);
    if preparedMotion.Success
        [certificate, certificateCache] = bmtpEngine.validation.checkFinalMotion(request, ...
            preparedMotion, roundoffReserve_units, ...
            obstacleTarget_units, certificateCache, true);
    end
    analyticIdentifier     = "c3JerkLimitedChord";
    analyticRepresentation = "analyticC3Clock";
elseif options.GoalTimeMode == "fixedArrival"
    % A timed direct motion can pass even when the selected guide detours.
    % Check it before committing to the guide route. The chord and its
    % certificate read no part of the seed, so one physical request that tries
    % several spatial guides constructs and certifies them exactly once.
    [preparedMotion, certificate, certificateCache] = directFixedArrivalMotion( ...
        request, roundoffReserve_units, obstacleTarget_units);
end
% A direct rest-to-rest earliest request with moving cells and no timed guide
% is the departure family: a certified zero-delay chord already supplies its
% first departure, otherwise the delayed chord is constructed from the same
% physical request. The seed label never selects this branch.
usesDepartureSchedule = size(route_units, 1) == 2 && ...
    options.GoalTimeMode == "earliestArrival" && request.IsRest && ...
    ~request.UsesTimeScopedSolver && isfield(coverage, 'ActiveTimeInterval_s');
if usesDepartureSchedule && ~(preparedMotion.Success && certificate.Passed)
    [controls_units, times_s, powers_units, departure] = bmtpEngine.motion.createDelayedChord(request);
    diagnostics.DepartureSchedule = departure;
    if isempty(times_s)
        [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
            "No C3 chord departure fits the horizon.", "noDepartureWindow", false);
        return;
    end
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(request, controls_units, times_s, powers_units);
    [certificate, certificateCache] = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, roundoffReserve_units, obstacleTarget_units, certificateCache);
    if ~preparedMotion.Success || ~certificate.Passed
        [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
            "The C3 departure proposal did not pass certification.", "departureUncertified", false);
        return;
    end
    analyticIdentifier = "c3DepartureSchedule";
end
if preparedMotion.Success && certificate.Passed
    diagnostics.Identifier               = analyticIdentifier;
    diagnostics.ConstraintRepresentation = analyticRepresentation;
    diagnostics.Converged                = true;
    diagnostics.OptimizerSpanCount       = 0;
    diagnostics.SegmentCount             = numel(preparedMotion.SegmentTime_s);
else
    if options.GoalTimeMode == "earliestArrival" && ~isfield(coverage, 'ActiveTimeInterval_s')
        [alternatingResult, diagnostics] = bmtpEngine.optimization.solveActivePairTrajectory( ...
            request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units);
    else
        % Only a variable clock needs the dedicated solver that rebuilds
        % obstacle/time overlap after every duration change. A prescribed
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
        [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
            "No optimized collision-free iterate was found. " + alternatingResult.SolverMessage, ...
            "noOptimizedFeasibleIterate", false);
        return;
    end
    if isfield(alternatingResult, 'PreparedMotion') && alternatingResult.PreparedMotion.Success && ...
            isfield(alternatingResult, 'Certificate') && alternatingResult.Certificate.Passed
        % The certificate belongs to this prepared motion, not to the raw controls above.
        preparedMotion = alternatingResult.PreparedMotion;
        certificate    = alternatingResult.Certificate;
    else
        % Endpoint correction and export can increase the derivative bounds.
        preparedMotion   = bmtpEngine.pipeline.prepareFinalMotion( ...
            request, alternatingResult.ControlPoint_units, alternatingResult.SegmentTime_s);
        certificate      = struct('Passed', false);
        certificateCache = [];
    end
end

%% Section 3: Prepare And Check The Final Motion
diagnostics.DilationScale = preparedMotion.DilationScale;
% Return reconstruction failure without certification because no complete motion exists to certify.
if ~preparedMotion.Success
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
        preparedMotion.Message, preparedMotion.TerminationReason, true);
    return;
end

% Certify every final curve-region pair; sampled clearance alone is insufficient.
if ~certificate.Passed
    [certificate, certificateCache] = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, roundoffReserve_units, obstacleTarget_units, certificateCache);
end
% A safe curved span need not admit one affine separator. Exact subdivision
% can expose its clearance without moving the curve or changing tolerances.
for refinementIndex = 1:10
    if certificate.Passed || ~certificate.WorkspacePassed || ...
            ~certificate.DynamicsPassed || ~certificate.ContinuityPassed
        break
    end
    failedPairs   = ~reshape([certificate.Planes.Verified], size(certificate.Planes)) & ...
        certificate.RegionActiveBySegment;
    splitMask     = any(failedPairs, 2);
    splitFraction = repmat(0.5, numel(splitMask), 1);
    if ~any(splitMask)
        break
    end
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(request, ...
        preparedMotion.ControlPoint_units, preparedMotion.SegmentTime_s, ...
        preparedMotion.PrescribedPower_units, splitMask, splitFraction);
    [certificate, certificateCache] = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, roundoffReserve_units, obstacleTarget_units, certificateCache);
end
diagnostics.SegmentCount = numel(preparedMotion.SegmentTime_s);
diagnostics.FinalCollisionPairCount = ...
    certificate.AllPairCount - certificate.VerifiedPairCount;
diagnostics.MotionCertificate = preparedMotion.MotionCertificate;
diagnostics.PlaneCertificate  = certificate;
candidate.PlaneCertificate    = certificate;

% Convert the checked curve to the public motion format and sample it.
candidate                     = bmtpEngine.pipeline.createMotionOutput(candidate, request, preparedMotion);
candidate.OptimizerFeasible   = true;
% Reject optimizer output that fails the independent certificate even when the numerical solver reported success.
if ~certificate.Passed
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
        "The optimized motion failed continuous collision, dynamics, or C3 continuity certification.", ...
        "planeCertificateUnavailable", true);
    return;
end

%% Section 4: Finalize The Directly Certified Candidate
[candidate.Message, candidate.TerminationReason] = deal( ...
    "A directly certified BMTP trajectory was found.", "goalReached");
[candidate.Success, diagnostics.Accepted]        = deal(true);
diagnostics.ElapsedTime_s                        = toc(totalTimer);
end

%% Section 5: Local Functions
function [preparedMotion, certificate, cache] = directFixedArrivalMotion(request, reserve_units, target_units)
    % Construct and certify the direct chord at the prescribed horizon. Every
    % input below comes from the physical request, never from the seed, so the
    % previous evaluation is reused whenever the request repeats exactly.
    persistent previousRequest previousMotion previousCertificate previousCache
    degree       = request.Degree;
    initialState = request.InitialState;
    goalState    = request.GoalState;
    key = struct( ...
        'Regions_units', {request.Regions_units}, ...
        'Coverage',      request.Coverage, ...
        'InitialState',  initialState, ...
        'GoalState',     goalState, ...
        'Limits',        request.Limits, ...
        'Options',       request.Options, ...
        'Degree',        degree, ...
        'Reserve_units', reserve_units, ...
        'Target_units',  target_units);
    if isequaln(key, previousRequest)
        [preparedMotion, certificate, cache] = deal( ...
            previousMotion, previousCertificate, previousCache);
        return;
    end
    certificate = struct('Passed', false);
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
        [certificate, cache] = bmtpEngine.validation.checkFinalMotion(request, ...
            preparedMotion, reserve_units, target_units, cache, true);
    end
    [previousRequest, previousMotion, previousCertificate, previousCache] = ...
        deal(key, preparedMotion, certificate, cache);
end

function candidate = createEmptyCandidate(seed, initialState)
    % Use the same candidate fields on success and failure.
    seedSource     = string(optionalField(seed, "Source", ""));
    dimensionCount = numel(initialState.position_units);
    candidate                                          = struct();
    candidate.Success                                  = false;
    candidate.OptimizerFeasible                        = false;
    candidate.OptimizerIterateUnavailable              = false;
    candidate.Message                                  = "The BMTP kernel was not run.";
    candidate.TerminationReason                        = "notRun";
    candidate.SeedSource                               = seedSource;
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
    candidate.PlaneCertificate                         = struct();
end

function value = optionalField(record, name, defaultValue)
    % Read an optional field or use its default.
    value = defaultValue;
    if isfield(record, name) && ~isempty(record.(name))
        value = record.(name);
    end
end

function diagnostics = createEmptyDiagnostics(degree, segmentCount, regionCount)
    % Initialize solver, timing, and certificate diagnostics.
    diagnostics = struct( ...
        "Identifier",                        "bmtpStaticDegree" + string(degree), ...
        "ConstraintRepresentation",          "thirdOrderTimePowerSocp", ...
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
        "MotionCertificate",                  struct(), ...
        "PlaneCertificate",                   struct(), ...
        "SolverMessage",                      "", ...
        "LastAttemptMessage",                 "", ...
        "ElapsedTime_s",                      0, ...
        "ConicSolver",                        bmtpEngine.optimization.accumulateConicDiagnostics());
end

function [candidate, diagnostics] = finishFailure( ...
    candidate, diagnostics, timer, message, reason, optimizerFeasible)
    % Return a failure without fabricating motion data.
    [candidate.Message, candidate.TerminationReason, candidate.OptimizerFeasible] = ...
        deal(message, reason, optimizerFeasible);
    [diagnostics.Accepted, diagnostics.ElapsedTime_s] = deal(false, toc(timer));
end
