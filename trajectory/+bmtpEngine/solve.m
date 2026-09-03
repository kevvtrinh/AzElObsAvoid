function [candidate, diagnostics] = solve( ...
        seed, regions_deg, coverage, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       bmtpEngine.solve( ...
%       seed, regions_deg, coverage, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Optimize one topology seed with a composite Bezier trajectory,
%     third-order time-power cones, and alternating degree-one maximum-margin
%     separating planes.
%   - Use the compact degree-7 representation only when planner-supplied
%     coverage records conservative grouping of a complex exact outline.
%   - Accept motion only after direct Bernstein checks, every applicable
%     span/region plane certificate, and independent public validation.
%**************************************************************************
% INPUTS
%   - seed (scalar struct)
%       position_deg is N-by-2; tau strictly increases from zero to one.
%   - regions_deg (R-by-1 cell array)
%       Each cell contains one finite convex N-by-2 exclusion polygon.
%   - coverage (scalar struct)
%       Requires Passed. Optional RegionActiveTauInterval is R-by-2 and
%       limits each region to an absolute normalized motion-time interval.
%   - initialState, goalState (normalized scalar state structs)
%       Positions are fixed and endpoint velocity and acceleration are zero.
%   - limits (normalized scalar struct)
%       Workspace, velocity, acceleration, and jerk bounds.
%   - options (resolved scalar planner-options struct)
%       Goal-time policy, sampling interval, work limits, and tolerances.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Stable motion record. Expected infeasibility returns Success=false.
%   - diagnostics (scalar struct)
%       Solver, timing, coverage, motion, and plane-certificate evidence.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3. Polynomial powers use local normalized time.
%**************************************************************************

%% Section 1: Validate And Create The Exclusion Representation

totalTimer = tic;
% The conic stages require one checked, dimension-neutral request with a
% declared polynomial representation and shared solver tolerances. Resolve it
% before allocating candidates so invalid requirements never enter a solver.
request = bmtpEngine.createSolveRequest( ...
    seed, regions_deg, coverage, initialState, goalState, limits, options);

% Alternating motion and separating-line solves need a topology-consistent,
% kinematically feasible starting curve. Create that warm representation once;
% later stages retain its route reduction and active-pair evidence.
warmStart = bmtpEngine.createWarmStart(request);
degree = request.Degree;
splitCount = request.SplitCount;
route_deg = warmStart.Route_deg;
segmentCount = warmStart.SegmentCount;
regionActiveBySegment = warmStart.RegionActiveBySegment;
candidate = createEmptyCandidate(seed, initialState, options);
diagnostics = createEmptyDiagnostics( degree, splitCount, segmentCount, numel(regions_deg));
diagnostics.OriginalSeedSegmentCount = warmStart.OriginalSeedSegmentCount;
diagnostics.WarmRouteResampled = warmStart.WarmRouteResampled;
diagnostics.Coverage = coverage;
diagnostics.ApplicablePairCount = nnz(regionActiveBySegment);
[~, ~, roundoffReserve_deg] = bmtpEngine.createCoordinateTolerances( ...
    route_deg, limits.azimuthInterval_deg, ...
    limits.elevationInterval_deg, regions_deg);
normalNormLimit = 1 + 2 ^ 20 * eps;
obstacleTarget_deg = normalNormLimit * ...
    options.CollisionClearanceTolerance_deg + roundoffReserve_deg;

%% Section 2: Alternate Time-Power And Maximum-Margin SOCPs

% The trajectory controls and separating lines depend on one another, so
% neither can be solved once in isolation. Alternate those two decisions,
% retain every attempt in diagnostics, and return the best sampled-clear
% iterate for the later direct certificate.
[alternatingResult, diagnostics] = ...
    bmtpEngine.solveAlternatingTrajectory( ...
    request, warmStart, diagnostics, obstacleTarget_deg, ...
    roundoffReserve_deg);
if ~alternatingResult.Success
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
        "No optimized collision-free iterate was found. " + ...
        alternatingResult.SolverMessage, ...
        "noOptimizedFeasibleIterate", false);
    return;
end

% Establish a feasible homotopy with the proven time-minimizing kernel before
% asking for less travel. A path-length solve from an unconstrained chord can
% initialize a separating plane on the wrong side of a concave obstacle.
[selectedMotion, diagnostics] = bmtpEngine.refineTravel( ...
    request, warmStart, alternatingResult, diagnostics, ...
    obstacleTarget_deg, roundoffReserve_deg);
bestControl_deg = selectedMotion.ControlPoint_deg;
bestSegmentTime_s = selectedMotion.SegmentTime_s;

%% Section 3: Prepare And Check The Final Motion

% Endpoint derivatives are imposed after optimization, which can increase the
% derivative-control bounds. Project those endpoints, split the curve, and
% increase segment time before any final safety claim is attempted.
preparedMotion = bmtpEngine.prepareFinalMotion( ...
    request, bestControl_deg, bestSegmentTime_s);
diagnostics.EndpointProjectionApplied = true;
diagnostics.DilationScale = preparedMotion.DilationScale;
if ~preparedMotion.Success
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
        preparedMotion.Message, preparedMotion.TerminationReason, true);
    return;
end

% Sampled overlap tests guided the alternating solver but cannot approve its
% output. Check every applicable final curve-region pair directly and retain
% the complete certificate for independent planner validation.
certificate = bmtpEngine.checkFinalMotion( ...
    request, warmStart, preparedMotion, roundoffReserve_deg, ...
    obstacleTarget_deg);
diagnostics.MotionCertificate = preparedMotion.MotionCertificate;
diagnostics.PlaneCertificate = certificate;
candidate.PlaneCertificate = certificate;

% The checked control net is still an engine representation. Convert and
% sample it once into the stable motion record consumed by the planner,
% validator, diagnostics, and plotting without rerunning any solve.
candidate = bmtpEngine.createMotionOutput( ...
    candidate, request, preparedMotion);
[candidate.OptimizerFeasible, candidate.ArrivalAtHorizon] = deal( ...
    true, preparedMotion.ArrivalAtHorizon);
diagnostics.BestDuration_s = candidate.MotionDuration_s;
if ~certificate.Passed
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
        "The optimized motion requires independent collision validation.", ...
        "planeCertificateUnavailable", true);
    return;
end

%% Section 4: Finalize The Directly Certified Candidate

[candidate.Message, candidate.TerminationReason] = ...
    deal("A directly certified BMTP trajectory was found.", "goalReached");
[candidate.Success, diagnostics.Accepted] = deal(true);
[diagnostics.BestDuration_s, diagnostics.ElapsedTime_s] = ...
    deal(candidate.MotionDuration_s, toc(totalTimer));
candidate.SolverDiagnostics = diagnostics;
end

%% Section 5: Local Functions

function plane = emptyPlane()
% Define the stable inactive or verified degree-one plane record.
plane = struct("Active", false, "Verified", false, "ExitFlag", NaN, ...
    "Normal", zeros(2, 2), "Offset_deg", zeros(1, 2), "SignedGap_deg", NaN);
end

function candidate = createEmptyCandidate(seed, initialState, options)
% Define identical candidate fields for success and every failure path.
seedIndex = optionalField(seed, "Index", 0);
seedSource = string(optionalField(seed, "Source", ""));
corridorBoundary_deg = optionalField(seed, "CorridorBoundary_deg", zeros(0, 2));
[candidate, ~] = bmtpEngine.createMotionRecord( ...
    struct(), initialState, [], [], options.SampleTime_s, seedSource);
extra = struct("OptimizerFeasible", false, "ArrivalAtHorizon", false, ...
    "SeedIndex", seedIndex, "SeedSource", seedSource, "FinalTime_s", NaN, ...
    "MotionDuration_s", NaN, "MotionLength_deg", Inf, ...
    "IntegratedSquaredJerk_deg2_s5", Inf, "MaximumConstraintViolation", Inf, ...
    "SolverDiagnostics", struct());
for name = string(fieldnames(extra)).'
    candidate.(name) = extra.(name);
end
candidate.SeedCorridorBoundary_deg = corridorBoundary_deg;
candidate.Message = "The BMTP kernel was not run.";
end

function value = optionalField(record, name, defaultValue)
% Read an optional scalar field while preserving documented empty defaults.
value = defaultValue;
if isfield(record, name) && ~isempty(record.(name))
    value = record.(name);
end
end

function diagnostics = createEmptyDiagnostics( degree, splitCount, segmentCount, regionCount)
% Define bounded solver, timing, and certificate evidence before solving.
diagnostics = struct( "Identifier", "bmtpStaticDegree" + string(degree), ...
    "ConstraintRepresentation", "thirdOrderTimePowerSocp", ...
    "Representation", "C3CompositeBezier", "Attempted", true, ...
    "Accepted", false, "Degree", degree, ...
    "SubspansPerSeedEdge", splitCount, "OriginalSeedSegmentCount", segmentCount, ...
    "WarmRouteResampled", false, "OptimizerSpanCount", segmentCount, ...
    "SegmentCount", 2 * segmentCount, "ExactRegionCount", regionCount, ...
    "IterationCount", 0, "Converged", false, ...
    "PlaneReuseApplied", false, "PlaneReuseCount", 0, ...
    "TaggedPairCount", 0, "ApplicablePairCount", segmentCount * regionCount, ...
    "TrajectorySocpCount", 0, "FinalCollisionPairCount", 0, ...
    "PlaneSocpCount", 0, "UnverifiedPlaneInitializationCount", 0, ...
    "FinalTrajectoryExitFlag", NaN, ...
    "FailedPlaneSegmentIndex", 0, "FailedPlaneRegionIndex", 0, ...
    "FailedPlane", emptyPlane(), "WarmStartDuration_s", NaN, ...
    "BestDuration_s", NaN, "RetainedBestTrialDuration_s", NaN, ...
    "EndpointProjectionApplied", false, ...
    "TrialDuration_s", NaN(35, 1), "TrialWasCollisionFree", false(35, 1), ...
    "CollisionPairCountHistory", NaN(35, 1), ...
    "DilationScale", NaN, "MotionCertificate", struct(), "Coverage", struct(), ...
    "PlaneCertificate", struct(), "SolverMessage", "", "ElapsedTime_s", 0);
end

function [candidate, diagnostics] = finishFailure( ...
        candidate, diagnostics, timer, message, reason, optimizerFeasible)
% Finalize one expected failure without manufacturing trajectory data.
[candidate.Message, candidate.TerminationReason, candidate.OptimizerFeasible] = ...
    deal(message, reason, optimizerFeasible);
[diagnostics.Accepted, diagnostics.ElapsedTime_s] = deal(false, toc(timer));
candidate.SolverDiagnostics = diagnostics;
end
