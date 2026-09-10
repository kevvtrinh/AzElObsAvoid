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
%   Use quintic Bezier segments; the planner independently validates the result.
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
%       Position, velocity, and acceleration are prescribed at both endpoints.
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
if isfield(options,'C3ProfileLibrary') && ~isempty(options.C3ProfileLibrary)
    request=bmtpEngine.createSolveRequest(seed,regions_units,coverage,initialState,goalState,limits,options);
    warm=bmtpEngine.createWarmStart(request);
    [warm,profileRecord]=bmtpEngine.lookupC3Profile(request,warm);
    ordinaryOptions=options;
    ordinaryOptions.C3ProfileLibrary=[];
    accepted=false;
    if profileRecord.Matched
        trialOptions=ordinaryOptions;
        trialOptions.C3ProfileWarmStart=warm;
        attemptTimer=tic;
        [candidate,diagnostics]=bmtpEngine.solve(seed,regions_units,coverage,initialState,goalState,limits,trialOptions);
        profileRecord.AttemptTime_s=toc(attemptTimer);
        if isfield(diagnostics,'ProfileSelectedEntryIndex')
            profileRecord.EntryIndex=diagnostics.ProfileSelectedEntryIndex;
            match=profileRecord.Shortlist(profileRecord.Shortlist(:,1)==profileRecord.EntryIndex,:);
            profileRecord.RouteError=match(2); profileRecord.LimitError=match(3); profileRecord.Reflected=logical(match(4));
            profileRecord.InitialTrials=diagnostics.ProfileInitialTrials;
        end
        profileRecord.Attempted=diagnostics.Identifier=="quinticJerkClock";
        profileRecord.AttemptSucceeded=candidate.Success;
        profileRecord.AttemptReason=candidate.TerminationReason;
        profileRecord.AttemptSolverMessage=candidate.Message;
        profileRecord.AttemptConicCalls=diagnostics.ConicSolver.CallCount;
        profileRecord.AttemptArrival_s=candidate.ArrivalTime_s;
        profileRecord.AttemptLength_units=candidate.MotionLength_units;
        accepted=candidate.Success;
        if accepted && profileRecord.Attempted && candidate.ArrivalTime_s>options.C3ProfileMaxArrival_s
            accepted=false;
            profileRecord.AttemptReason="profileArrivalCapExceeded";
        end
        if accepted && profileRecord.Attempted && candidate.MotionLength_units>options.C3ProfileMaxLength_units
            accepted=false;
            profileRecord.AttemptReason="profileLengthCapExceeded";
        end
        profileRecord.Accepted=accepted && profileRecord.Attempted;
    end
    if ~accepted
        profileRecord.FallbackUsed=true;
        fallbackTimer=tic;
        [candidate,diagnostics]=bmtpEngine.solve(seed,regions_units,coverage,initialState,goalState,limits,ordinaryOptions);
        profileRecord.FallbackTime_s=toc(fallbackTimer);
    end
    diagnostics.ProfileLibrary=profileRecord;
    diagnostics.ElapsedTime_s=toc(totalTimer);
    candidate.SolverDiagnostics=diagnostics;
    return;
end
% Validate the request and resolve shared solver settings.
request = bmtpEngine.createSolveRequest(seed, regions_units, coverage, initialState, goalState, limits, options);
initialState = request.InitialState; goalState = request.GoalState;

% Create a kinematically feasible starting curve from the seed.
warmStart             = bmtpEngine.createWarmStart(request);
if isfield(options,'C3ProfileWarmStart'), warmStart=options.C3ProfileWarmStart; end
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

% Earliest arrival tries the C3 jerk-limited chord. Fixed arrival retains
% the minimum-jerk quintic at the requested physical horizon.
preparedMotion = struct('Success',false);
certificateEventTime_s=[];
certificate = struct('Passed',false); certificateCache=[];
analyticIdentifier = "minimumJerkQuintic";
analyticRepresentation = "analyticQuinticClock";
if size(route_units,1)==2 && options.GoalTimeMode=="earliestArrival" && request.IsRest
    [controls_units,times_s,powers_units] = bmtpEngine.createC3Chord(initialState.position_units,goalState.position_units,limits);
    preparedMotion = bmtpEngine.prepareFinalMotion(request,controls_units,times_s,powers_units);
    if preparedMotion.Success
        [certificate,certificateCache]=bmtpEngine.checkFinalMotion(request,warmStart,preparedMotion,roundoffReserve_units,obstacleTarget_units,certificateCache);
    end
    analyticIdentifier = "c3JerkLimitedChord";
    analyticRepresentation = "analyticC3Clock";
elseif options.GoalTimeMode=="fixedArrival"
    % A timed direct motion may pass through a spatial guide's swept hull.
    % Check it before committing to that guide's detour.
    directDuration_s = request.MotionHorizon_s;
    fraction = zeros(degree+1,1);
    coefficients = [10 -15 6];
    for k = 0:degree
        for power = 3:min(k,5)
            fraction(k+1) = fraction(k+1)+coefficients(power-2)*nchoosek(k,power)/nchoosek(degree,power);
        end
    end
    controls_units = initialState.position_units + fraction.*(goalState.position_units-initialState.position_units);
    if ~request.IsRest
        h = request.MotionHorizon_s;
        power = [initialState.position_units;h*initialState.velocity_units_s;h^2*initialState.acceleration_units_s2/2;zeros(3,2)];
        residual = [goalState.position_units-sum(power,1); ...
            h*goalState.velocity_units_s-power(2,:)-2*power(3,:); ...
            h^2*goalState.acceleration_units_s2-2*power(3,:)];
        power(4:6,:) = [1 1 1;3 4 5;6 12 20]\residual;
        controls_units = zeros(degree+1,2);
        for k = 0:degree
            for j = 0:min(k,5)
                controls_units(k+1,:) = controls_units(k+1,:)+nchoosek(k,j)/nchoosek(degree,j)*power(j+1,:);
            end
        end
    end
    preparedMotion = bmtpEngine.prepareFinalMotion(request,reshape(controls_units,1,degree+1,2),directDuration_s);
    if preparedMotion.Success
        [certificate,certificateCache]=bmtpEngine.checkFinalMotion(request,warmStart,preparedMotion,roundoffReserve_units,obstacleTarget_units,certificateCache);
    end
end
% A certified zero-delay chord already supplies this schedule's first departure.
% Preserve its absolute clock and certificate instead of computing them again.
if seed.Source=="departureSchedule" && ~(preparedMotion.Success && certificate.Passed)
    [controls_units,times_s,powers_units,departure] = bmtpEngine.createDelayedChord(request);
    diagnostics.DepartureSchedule = departure;
    if isempty(times_s)
        [candidate,diagnostics] = finishFailure(candidate,diagnostics,totalTimer,"No C3 chord departure fits the horizon.","noDepartureWindow",false);
        return;
    end
    preparedMotion = bmtpEngine.prepareFinalMotion(request,controls_units,times_s,powers_units);
    [certificate,certificateCache]=bmtpEngine.checkFinalMotion(request,warmStart,preparedMotion,roundoffReserve_units,obstacleTarget_units,certificateCache);
    if ~preparedMotion.Success || ~certificate.Passed
        [candidate,diagnostics] = finishFailure(candidate,diagnostics,totalTimer,"The C3 departure proposal did not pass certification.","departureUncertified",false);
        return;
    end
    analyticIdentifier = "c3DepartureSchedule";
end
boundRecord = struct('Attempted',false,'Passed',false,'Time_s',NaN,'ElapsedTime_s',0,'TrajectorySocpCount',0);
if preparedMotion.Success && certificate.Passed
    diagnostics.Identifier = analyticIdentifier;
    diagnostics.ConstraintRepresentation = analyticRepresentation;
    diagnostics.Converged = true;
    diagnostics.OptimizerSpanCount = 0;
    diagnostics.SegmentCount = numel(preparedMotion.SegmentTime_s);
else
    prescribedPower_units = [];
    if options.GoalTimeMode=="earliestArrival" && ~isfield(coverage,'ActiveTimeInterval_s')
        alternatingResult=struct('Success',false);
        if request.IsRest
            % Reuse exact source facets for monotone routes. The locked axis
            % uses smoothed quintic spans; free-axis jerk is C0 across joins.
            distance_units=abs(goalState.position_units-initialState.position_units);
            axisTime_s=max([1.875*distance_units./limits.maxVelocity_units_s; ...
                sqrt((10/sqrt(3))*distance_units./limits.maxAcceleration_units_s2); ...
                (60*distance_units./limits.maxJerk_units_s3).^(1/3)],[],1);
            [~,corridorTimes_s,corridorPower_units]=bmtpEngine.createC3Chord( ...
                initialState.position_units,goalState.position_units,limits);
            corridorWarm=warmStart;
            corridorWarm.SegmentCount=numel(corridorTimes_s);
            corridorWarm.SegmentTime_s=corridorTimes_s;
            corridorWarm.FixedPower_units=corridorPower_units;
            corridorWarm.AxisMinimumTime_s=axisTime_s;
            corridorWarm.ClockGuide=struct('Route_units',route_units);
            [alternatingResult,diagnostics]=bmtpEngine.solveStaticCorridor(request,corridorWarm,diagnostics,obstacleTarget_units,roundoffReserve_units);
        end
        if ~alternatingResult.Success
            previousConic=diagnostics.ConicSolver;
            previousSolveCount=diagnostics.TrajectorySocpCount;
            [alternatingResult,diagnostics] = bmtpEngine.solveQuinticTrajectory(request,warmStart,diagnostics,obstacleTarget_units,roundoffReserve_units);
            diagnostics.ConicSolver.CallCount=diagnostics.ConicSolver.CallCount+previousConic.CallCount;
            diagnostics.ConicSolver.TotalTime_s=diagnostics.ConicSolver.TotalTime_s+previousConic.TotalTime_s;
            diagnostics.TrajectorySocpCount=diagnostics.TrajectorySocpCount+previousSolveCount;
        end
        prescribedPower_units = alternatingResult.PositionPower_units;
    else
        [alternatingResult, diagnostics] = bmtpEngine.solveAlternatingTrajectory(request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units);
    end
    if ~alternatingResult.Success
        [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, "No optimized collision-free iterate was found. " + alternatingResult.SolverMessage, "noOptimizedFeasibleIterate", false);
        return;
    end
    if isfield(alternatingResult,'CertificateEventTime_s')
        certificateEventTime_s=alternatingResult.CertificateEventTime_s;
    end
    % Endpoint correction and export can increase the derivative bounds.
    preparedMotion = bmtpEngine.prepareFinalMotion(request, alternatingResult.ControlPoint_units, alternatingResult.SegmentTime_s,prescribedPower_units);
    certificate = struct('Passed',false); certificateCache=[];
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
    [certificate,certificateCache]=bmtpEngine.checkFinalMotion(request, warmStart, preparedMotion, roundoffReserve_units, obstacleTarget_units,certificateCache);
end
% A safe curved span need not admit one affine separator. Exact subdivision
% can expose its clearance without moving the curve or changing tolerances.
for refinement=1:10
    if certificate.Passed || ~certificate.DynamicsPassed || ~certificate.ContinuityPassed, break; end
    failed=~reshape([certificate.Planes.Verified],size(certificate.Planes)) & certificate.RegionActiveBySegment;
    splitMask=any(failed,2); splitFraction=repmat(0.5,numel(splitMask),1);
    breaks_s=[0;cumsum(preparedMotion.SegmentTime_s)];
    for span=find(splitMask).'
        events_s=certificateEventTime_s(certificateEventTime_s>breaks_s(span)+64*eps(breaks_s(end)) & ...
            certificateEventTime_s<breaks_s(span+1)-64*eps(breaks_s(end)));
        if ~isempty(events_s)
            [~,event]=min(abs(events_s-mean(breaks_s(span:span+1))));
            splitFraction(span)=(events_s(event)-breaks_s(span))/preparedMotion.SegmentTime_s(span);
        end
    end
    preparedMotion=bmtpEngine.prepareFinalMotion(request,preparedMotion.ControlPoint_units, ...
        preparedMotion.SegmentTime_s,preparedMotion.PrescribedPower_units,splitMask,splitFraction);
    [certificate,certificateCache]=bmtpEngine.checkFinalMotion(request,warmStart,preparedMotion,roundoffReserve_units,obstacleTarget_units,certificateCache);
end
diagnostics.SegmentCount=numel(preparedMotion.SegmentTime_s);
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
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, "The optimized motion failed continuous collision, dynamics, or C3 continuity certification.", "planeCertificateUnavailable", true);
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
        "Representation", "C3CompositeQuintic", "Attempted", true, ...
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
