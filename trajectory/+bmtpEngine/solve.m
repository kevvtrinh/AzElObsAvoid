function [candidate, diagnostics] = solve(seed, regions_units, coverage, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       bmtpEngine.solve( ...
%       seed, regions_units, coverage, initialState, goalState, limits, options)
%
% PURPOSE
%   Turn one proposed path into a smooth motion that respects motion limits.
%   Adjust the curve and obstacle-separating boundaries in alternating steps.
%   Use degree-eight Bezier segments; the planner independently validates the result.
%
% INPUTS
%   - seed (scalar struct)
%       position_units is N-by-2; tau strictly increases from zero to one.
%   - regions_units (R-by-1 cell array)
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
%
% OUTPUTS
%   - candidate (scalar struct)
%       Stable motion record. Expected infeasibility returns Success=false.
%   - diagnostics (scalar struct)
%       Solver, timing, coverage, motion, and plane-certificate evidence.
%
% UNITS
%   - Position is coordinate units and time is seconds. Derivatives use units/s,
%     units/s^2, and units/s^3. Polynomial powers use local normalized time.
%

%% Section 1: Validate And Create The Exclusion Representation

totalTimer = tic;
% Validate the request and resolve shared solver settings.
request = bmtpEngine.createSolveRequest(seed, regions_units, coverage, initialState, goalState, limits, options);

% Create a kinematically feasible starting curve from the seed.
warmStart             = bmtpEngine.createWarmStart(request);
degree                = request.Degree;
splitCount            = request.SplitCount;
route_units             = warmStart.Route_units;
segmentCount          = warmStart.SegmentCount;
regionActiveBySegment = warmStart.RegionActiveBySegment;
candidate             = createEmptyCandidate(seed, initialState, options);
diagnostics           = createEmptyDiagnostics(degree, splitCount, segmentCount, numel(regions_units));
diagnostics.OriginalSeedSegmentCount = warmStart.OriginalSeedSegmentCount;
diagnostics.WarmRouteResampled       = warmStart.WarmRouteResampled;
diagnostics.Coverage                 = coverage;
diagnostics.ApplicablePairCount      = nnz(regionActiveBySegment);
[~, ~, roundoffReserve_units] = bmtpEngine.createCoordinateTolerances(route_units, limits.xInterval_units, limits.yInterval_units, regions_units);
normalNormLimit    = 1 + 2 ^ 20 * eps;
obstacleTarget_units = normalNormLimit * options.CollisionClearanceTolerance_units + roundoffReserve_units;

%% Section 2: Alternate Time-Power And Maximum-Margin SOCPs

% Alternate trajectory and separating-line solves.
% Keep the best sampled-clear candidate for final certification.
[alternatingResult, diagnostics] = bmtpEngine.solveAlternatingTrajectory(request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units);
% Return the alternating optimizer's stable failure result instead of attempting final-motion preparation on invalid controls.
if ~alternatingResult.Success
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, "No optimized collision-free iterate was found. " + alternatingResult.SolverMessage, "noOptimizedFeasibleIterate", false);
    return;
end

% Find a feasible route before minimizing travel; starting from the direct
% chord can place separating planes on the wrong side of concave obstacles.
[selectedMotion, diagnostics] = bmtpEngine.refineTravel(request, warmStart, alternatingResult, diagnostics, obstacleTarget_units, roundoffReserve_units);
bestControl_units   = selectedMotion.ControlPoint_units;
bestSegmentTime_s = selectedMotion.SegmentTime_s;

%% Section 3: Prepare And Check The Final Motion

% Endpoint corrections can increase derivative peaks.
% Correct endpoints and adjust segment times before final checks.
preparedMotion = bmtpEngine.prepareFinalMotion(request, bestControl_units, bestSegmentTime_s);
diagnostics.EndpointProjectionApplied = true;
diagnostics.DilationScale             = preparedMotion.DilationScale;
% Return reconstruction failure without certification because no complete motion exists to certify.
if ~preparedMotion.Success
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, preparedMotion.Message, preparedMotion.TerminationReason, true);
    return;
end

% Certify every final curve-region pair; sampled clearance alone is insufficient.
certificate = bmtpEngine.checkFinalMotion(request, warmStart, preparedMotion, roundoffReserve_units, obstacleTarget_units);
diagnostics.MotionCertificate = preparedMotion.MotionCertificate;
diagnostics.PlaneCertificate  = certificate;
candidate.PlaneCertificate = certificate;

% Convert the checked curve to the public motion format and sample it.
candidate = bmtpEngine.createMotionOutput(candidate, request, preparedMotion);
[candidate.OptimizerFeasible, candidate.ArrivalAtHorizon] = deal(true, preparedMotion.ArrivalAtHorizon);
diagnostics.BestDuration_s = candidate.TrajectoryDuration_s;
% Reject optimizer output that fails the independent certificate even when the numerical solver reported success.
if ~certificate.Passed
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, "The optimized motion requires independent collision validation.", "planeCertificateUnavailable", true);
    return;
end

%% Section 4: Finalize The Directly Certified Candidate

[candidate.Message, candidate.TerminationReason]        = deal("A directly certified BMTP trajectory was found.", "goalReached");
[candidate.Success, diagnostics.Accepted]               = deal(true);
[diagnostics.BestDuration_s, diagnostics.ElapsedTime_s] = deal(candidate.TrajectoryDuration_s, toc(totalTimer));
candidate.SolverDiagnostics = diagnostics;
end

%% Section 5: Local Functions

function plane = emptyPlane()
    % Initialize an inactive separating-plane record.
    plane = struct();
    plane.Active        = false;
    plane.Verified      = false;
    plane.ExitFlag      = NaN;
    plane.Normal        = zeros(2, 2);
    plane.Offset_units    = zeros(1, 2);
    plane.SignedGap_units = NaN;
end

function candidate = createEmptyCandidate(seed, initialState, options)
    % Use the same candidate fields on success and failure.
    seedIndex            = optionalField(seed, "Index", 0);
    seedSource           = string(optionalField(seed, "Source", ""));
    obstacleEnvelope_units = optionalField(seed, "ObstacleEnvelope_units", zeros(0, 2));
    [candidate, ~] = bmtpEngine.createMotionRecord(struct(), initialState, [], [], options.SampleTime_s, seedSource);
    candidate.ArrivalAtHorizon              = false;
    candidate.SeedIndex                     = seedIndex;
    candidate.MotionLength_units              = Inf;
    candidate.IntegratedSquaredJerk_units2_s5 = Inf;
    candidate.SeedCorridorBoundary_units      = obstacleEnvelope_units;
    candidate.Message                       = "The BMTP kernel was not run.";
end

function value = optionalField(record, name, defaultValue)
    % Read an optional field or use its default.
    value = defaultValue;
    if isfield(record, name) && ~isempty(record.(name))
        value = record.(name);
    end
end

function diagnostics = createEmptyDiagnostics(degree, splitCount, segmentCount, regionCount)
    % Initialize solver, timing, and certificate diagnostics.
    diagnostics = struct("Identifier", "bmtpStaticDegree" + string(degree), ...
        "ConstraintRepresentation", "thirdOrderTimePowerSocp", ...
        "Representation", "C3CompositeBezier", "Attempted", true, ...
        "Accepted", false, "Degree", degree, ...
        "SubspansPerSeedEdge", splitCount, "OriginalSeedSegmentCount", segmentCount, ...
        "WarmRouteResampled", false, "OptimizerSpanCount", segmentCount, ...
        "SegmentCount", 2 * segmentCount, "ExactRegionCount", regionCount, ...
        "IterationCount", 0, "Converged", false, ...
        "PlaneReuseApplied", false, "PlaneReuseCount", 0, ...
        "RetainedHorizonRetryCount", 0, ...
        "TaggedPairCount", 0, "ApplicablePairCount", segmentCount * regionCount, ...
        "TrajectorySocpCount", 0, "FinalCollisionPairCount", 0, ...
        "PlaneSocpCount", 0, "UnverifiedPlaneInitializationCount", 0, ...
        "FinalTrajectoryExitFlag", NaN, ...
        "FailedPlaneSegmentIndex", 0, "FailedPlaneRegionIndex", 0, ...
        "FailedPlane", emptyPlane(), "WarmStartDuration_s", NaN, ...
        "BestDuration_s", NaN, "RetainedBestTrialDuration_s", NaN, ...
        "TravelRefinementAttempted", false, ...
        "TravelRefinementInitialLength_units", NaN, ...
        "TravelRefinementFinalLength_units", NaN, ...
        "TravelRefinementInitialDuration_s", NaN, ...
        "TravelRefinementFinalDuration_s", NaN, ...
        "TravelRefinementAccepted", false, ...
        "EndpointProjectionApplied", false, ...
        "TrialDuration_s", NaN(35, 1), "TrialWasCollisionFree", false(35, 1), ...
        "CollisionPairCountHistory", NaN(35, 1), ...
        "DilationScale", NaN, "MotionCertificate", struct(), "Coverage", struct(), ...
        "PlaneCertificate", struct(), "SolverMessage", "", "ElapsedTime_s", 0);
end

function [candidate, diagnostics] = finishFailure(candidate, diagnostics, timer, message, reason, optimizerFeasible)
    % Return a failure without fabricating motion data.
    [candidate.Message, candidate.TerminationReason, candidate.OptimizerFeasible] = deal(message, reason, optimizerFeasible);
    [diagnostics.Accepted, diagnostics.ElapsedTime_s]                             = deal(false, toc(timer));
    candidate.SolverDiagnostics = diagnostics;
end
