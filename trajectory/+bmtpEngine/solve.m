function [candidate, diagnostics, clockGuide] = solve(seed, regions_units, coverage, initialState, goalState, limits, options, stage)
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
%   - stage (optional internal policy): complete, kinematicBound, delayedChord, or route.
%       Allows the planner to defer spatial search until the bound is tested.
%
% OUTPUTS
%   - candidate (scalar struct)
%       Stable motion record. Expected infeasibility returns Success=false.
%   - diagnostics (scalar struct)
%       Solver, timing, coverage, motion, and plane-certificate evidence.
%   - clockGuide (scalar struct): searched clock projection, when available.
%
% UNITS
%   - Position is coordinate units and time is seconds. Derivatives use units/s,
%     units/s^2, and units/s^3. Polynomial powers use local normalized time.
%

%% Section 1: Validate And Create The Exclusion Representation

totalTimer = tic;
if nargin<8, stage = "complete"; end
assert(any(stage==["complete","kinematicBound","delayedChord","route"]),'bmtpEngine:InvalidStage','Unknown internal solve stage.');
clockGuide = struct();
% Validate the request and resolve shared solver settings.
request = bmtpEngine.createSolveRequest(seed, regions_units, coverage, initialState, goalState, limits, options);

% Create a kinematically feasible starting curve from the seed.
warmStart             = bmtpEngine.createWarmStart(request);
degree                = request.Degree;
splitCount            = request.SplitCount;
route_units             = warmStart.Route_units;
segmentCount          = warmStart.SegmentCount;
regionActiveBySegment = warmStart.RegionActiveBySegment;
candidate             = createEmptyCandidate(seed, initialState);
diagnostics           = createEmptyDiagnostics(degree, splitCount, segmentCount, numel(regions_units));
diagnostics.OriginalSeedSegmentCount = warmStart.OriginalSeedSegmentCount;
diagnostics.WarmRouteResampled       = warmStart.WarmRouteResampled;
diagnostics.Coverage                 = coverage;
diagnostics.ApplicablePairCount      = nnz(regionActiveBySegment);
if isfield(coverage,'BreakTime_s')
    diagnostics.Identifier = "bmtpTimeCellsDegree"+string(degree);
end
if options.GoalTimeMode=="fixedArrival"
    diagnostics.ConstraintRepresentation = "fixedClockElasticSocp";
end
endRegions_units = cell(0,1);
if isfield(coverage,'EndRegions_units'), endRegions_units = coverage.EndRegions_units; end
[~, ~, roundoffReserve_units] = bmtpEngine.createCoordinateTolerances(route_units, limits.xInterval_units, limits.yInterval_units, regions_units,endRegions_units);
normalNormLimit    = 1 + 2 ^ 20 * eps;
obstacleTarget_units = normalNormLimit * options.CollisionClearanceTolerance_units + roundoffReserve_units;

%% Section 2: Solve The Direct Curve Or Alternating Convex Problem

% The unconstrained fixed-time minimum-jerk solution is a quintic on the
% endpoint chord. Degree elevation preserves it exactly in the shared basis.
preparedMotion = struct('Success',false);
certificate = struct('Passed',false);
analyticIdentifier = "minimumJerkQuintic";
analyticRepresentation = "analyticFixedTime";
if stage=="delayedChord"
    [controls_units,durations_s,powers_units,schedule] = bmtpEngine.createDelayedChord(request);
    diagnostics.DepartureSchedule = schedule;
    if schedule.Available
        preparedMotion = bmtpEngine.prepareFinalMotion(request,controls_units,durations_s,powers_units);
        if preparedMotion.Success
            certificate = bmtpEngine.checkFinalMotion(request,warmStart,preparedMotion,roundoffReserve_units,obstacleTarget_units);
        end
    end
    analyticIdentifier = "delayedJerkLimitedChord";
    analyticRepresentation = "analyticDelayedChord";
elseif stage~="route" && size(route_units,1)==2 && options.GoalTimeMode == "fixedArrival"
    fraction = zeros(degree+1,1);
    coefficients = [10 -15 6];
    for k = 0:degree
        for power = 3:min(k,5)
            fraction(k+1) = fraction(k+1)+coefficients(power-2)*nchoosek(k,power)/nchoosek(degree,power);
        end
    end
    controls_units = initialState.position_units + fraction.*(goalState.position_units-initialState.position_units);
    preparedMotion = bmtpEngine.prepareFinalMotion(request,reshape(controls_units,1,degree+1,2),request.MotionHorizon_s);
    if preparedMotion.Success
        certificate = bmtpEngine.checkFinalMotion(request,warmStart,preparedMotion,roundoffReserve_units,obstacleTarget_units);
    end
elseif stage~="route" && options.GoalTimeMode=="earliestArrival"
    [controls_units,durations_s] = bmtpEngine.createJerkLimitedChord(initialState.position_units,goalState.position_units,limits,degree);
    preparedMotion = bmtpEngine.prepareFinalMotion(request,controls_units,durations_s);
    if preparedMotion.Success
        certificate = bmtpEngine.checkFinalMotion(request,warmStart,preparedMotion,roundoffReserve_units,obstacleTarget_units);
    end
    analyticIdentifier = "jerkLimitedChord";
    analyticRepresentation = "analyticMinimumTime";
end
boundAccepted = false;
boundStats = bmtpEngine.accumulateConicDiagnostics();
boundRecord = struct('Attempted',false,'Passed',false,'Time_s',NaN,'ElapsedTime_s',0,'TrajectorySocpCount',0);
if any(stage==["complete","kinematicBound"]) && options.GoalTimeMode=="earliestArrival" && ~(preparedMotion.Success && certificate.Passed)
    % At the independent-axis lower bound, the limiting axis is analytic.
    % Optimize only the remaining freedom on a source-derived clock guide.
    boundTimer = tic;
    boundWarm = bmtpEngine.createReachabilityWarmStart(request);
    clockGuide = boundWarm.ClockGuide;
    boundRecord.Attempted = true;
    boundRecord.Time_s = boundWarm.Duration_s;
    if boundWarm.ClockGuide.IsConnected && boundWarm.Duration_s<=request.MotionHorizon_s && ...
            nnz(boundWarm.AxisMinimumTime_s==boundWarm.Duration_s)<2
        boundRequest = request;
        boundRequest.Options.GoalTimeMode = "fixedArrival";
        boundRequest.MotionHorizon_s = boundWarm.Duration_s;
        boundDiagnostics = createEmptyDiagnostics(degree,0,boundWarm.SegmentCount,numel(regions_units));
        boundDiagnostics.Coverage = coverage;
        boundDiagnostics.WarmRouteResampled = true;
        boundDiagnostics.ApplicablePairCount = nnz(boundWarm.RegionActiveBySegment);
        boundPowers_units = boundWarm.FixedPower_units;
        if ~isfield(coverage,'ActiveTimeInterval_s')
            [boundResult,boundDiagnostics] = bmtpEngine.solveStaticCorridor(request,boundWarm,boundDiagnostics,obstacleTarget_units,roundoffReserve_units);
            boundPowers_units = boundResult.PositionPower_units;
        else
            [boundResult,boundDiagnostics] = bmtpEngine.solveAlternatingTrajectory(boundRequest,boundWarm,boundDiagnostics,obstacleTarget_units,roundoffReserve_units);
        end
        boundStats = boundDiagnostics.ConicSolver;
        if boundResult.Success
            boundMotion = bmtpEngine.prepareFinalMotion(request,boundResult.ControlPoint_units,boundResult.SegmentTime_s,boundPowers_units);
            if boundMotion.Success
                boundCertificate = bmtpEngine.checkFinalMotion(request,boundWarm,boundMotion,roundoffReserve_units,obstacleTarget_units);
                if boundCertificate.Passed
                    preparedMotion = boundMotion;
                    certificate = boundCertificate;
                    diagnostics = boundDiagnostics;
                    boundAccepted = true;
                    analyticIdentifier = "kinematicBoundBmtp";
                    analyticRepresentation = "fixedClockElasticSocp";
                    if ~isfield(coverage,'ActiveTimeInterval_s')
                        analyticIdentifier = "monotoneStaticCorridor";
                        analyticRepresentation = "integratedQuinticCorridor";
                    end
                end
            end
        end
    end
    boundRecord.Passed = boundAccepted && abs(sum(preparedMotion.SegmentTime_s)-boundWarm.Duration_s)<=options.ArrivalTimeTolerance_s;
    boundRecord.ElapsedTime_s = toc(boundTimer);
    boundRecord.TrajectorySocpCount = boundStats.CallCount;
end
if preparedMotion.Success && certificate.Passed
    diagnostics.Identifier = analyticIdentifier;
    diagnostics.ConstraintRepresentation = analyticRepresentation;
    if ~boundAccepted
        diagnostics.Converged = true;
        diagnostics.OptimizerSpanCount = 0;
    end
    diagnostics.SegmentCount = numel(preparedMotion.SegmentTime_s);
else
    if any(stage==["kinematicBound","delayedChord"])
        diagnostics.TrajectorySocpCount = boundStats.CallCount;
        diagnostics.ConicSolver = boundStats;
        diagnostics.LowerBoundAttempt = boundRecord;
        [candidate,diagnostics] = finishFailure(candidate,diagnostics,totalTimer, ...
            "The "+stage+" motion was not certified.",stage+"Uncertified",false);
        return;
    end
    prescribedPower_units = [];
    if options.GoalTimeMode=="earliestArrival" && ~isfield(coverage,'ActiveTimeInterval_s')
        [alternatingResult,diagnostics] = bmtpEngine.solveCubicTrajectory(request,warmStart,diagnostics,obstacleTarget_units,roundoffReserve_units);
        prescribedPower_units = alternatingResult.PositionPower_units;
    else
        [alternatingResult, diagnostics] = bmtpEngine.solveAlternatingTrajectory(request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units);
    end
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount+boundStats.CallCount;
    diagnostics.ConicSolver.CallCount = diagnostics.ConicSolver.CallCount+boundStats.CallCount;
    diagnostics.ConicSolver.TotalTime_s = diagnostics.ConicSolver.TotalTime_s+boundStats.TotalTime_s;
    diagnostics.LowerBoundAttempt = boundRecord;
    if ~alternatingResult.Success
        [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, "No optimized collision-free iterate was found. " + alternatingResult.SolverMessage, "noOptimizedFeasibleIterate", false);
        return;
    end
    % Endpoint correction and export can increase the derivative bounds.
    preparedMotion = bmtpEngine.prepareFinalMotion(request, alternatingResult.ControlPoint_units, alternatingResult.SegmentTime_s,prescribedPower_units);
    certificate = struct('Passed',false);
end
diagnostics.LowerBoundAttempt = boundRecord;

%% Section 3: Prepare And Check The Final Motion

diagnostics.EndpointProjectionApplied = true;
diagnostics.DilationScale             = preparedMotion.DilationScale;
% Return reconstruction failure without certification because no complete motion exists to certify.
if ~preparedMotion.Success
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, preparedMotion.Message, preparedMotion.TerminationReason, true);
    return;
end

% Certify every final curve-region pair; sampled clearance alone is insufficient.
if ~certificate.Passed
    certificate = bmtpEngine.checkFinalMotion(request, warmStart, preparedMotion, roundoffReserve_units, obstacleTarget_units);
end
diagnostics.FinalCollisionPairCount = certificate.AllPairCount;
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

function candidate = createEmptyCandidate(seed, initialState)
    % Use the same candidate fields on success and failure.
    seedIndex            = optionalField(seed, "Index", 0);
    seedSource           = string(optionalField(seed, "Source", ""));
    obstacleEnvelope_units = optionalField(seed, "ObstacleEnvelope_units", zeros(0, 2));
    dimensionCount = numel(initialState.position_units);
    candidate = struct();
    candidate.Success = false;
    candidate.OptimizerFeasible = false;
    candidate.Message = "The BMTP kernel was not run.";
    candidate.TerminationReason = "notRun";
    candidate.SeedIndex = seedIndex;
    candidate.SeedSource = seedSource;
    candidate.ArrivalTime_s = NaN;
    candidate.TrajectoryDuration_s = NaN;
    candidate.ArrivalAtHorizon = false;
    candidate.MotionLength_units = Inf;
    candidate.IntegratedSquaredJerk_units2_s5 = Inf;
    candidate.MaximumConstraintViolation = Inf;
    candidate.time_s = zeros(0, 1);
    candidate.position_units = zeros(0, dimensionCount);
    candidate.velocity_units_s = zeros(0, dimensionCount);
    candidate.acceleration_units_s2 = zeros(0, dimensionCount);
    candidate.jerk_units_s3 = zeros(0, dimensionCount);
    candidate.Polynomial = struct();
    candidate.PlaneCertificate = struct();
    candidate.SeedCorridorBoundary_units = obstacleEnvelope_units;
    candidate.SolverDiagnostics = struct();
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
        "Representation", "C2CompositeBezier", "Attempted", true, ...
        "Accepted", false, "Degree", degree, ...
        "SubspansPerSeedEdge", splitCount, "OriginalSeedSegmentCount", segmentCount, ...
        "WarmRouteResampled", false, "OptimizerSpanCount", segmentCount, ...
        "SegmentCount", 2 * segmentCount, "ExactRegionCount", regionCount, ...
        "IterationCount", 0, "Converged", false, ...
        "ApplicablePairCount", segmentCount * regionCount, ...
        "TrajectorySocpCount", 0, "FinalCollisionPairCount", 0, ...
        "PlaneSocpCount", 0, "UnverifiedPlaneInitializationCount", 0, ...
        "FinalTrajectoryExitFlag", NaN, ...
        "FailedPlaneSegmentIndex", 0, "FailedPlaneRegionIndex", 0, ...
        "FailedPlane", emptyPlane(), "WarmStartDuration_s", NaN, ...
        "BestDuration_s", NaN, ...
        "EndpointProjectionApplied", false, ...
        "TrialDuration_s", NaN(35, 1), "TrialWasCollisionFree", false(35, 1), ...
        "CollisionPairCountHistory", NaN(35, 1), ...
        "DilationScale", NaN, "MotionCertificate", struct(), "Coverage", struct(), ...
        "PlaneCertificate", struct(), "SolverMessage", "", "ElapsedTime_s", 0, ...
        "ConicSolver",bmtpEngine.accumulateConicDiagnostics());
end

function [candidate, diagnostics] = finishFailure(candidate, diagnostics, timer, message, reason, optimizerFeasible)
    % Return a failure without fabricating motion data.
    [candidate.Message, candidate.TerminationReason, candidate.OptimizerFeasible] = deal(message, reason, optimizerFeasible);
    [diagnostics.Accepted, diagnostics.ElapsedTime_s]                             = deal(false, toc(timer));
    candidate.SolverDiagnostics = diagnostics;
end
