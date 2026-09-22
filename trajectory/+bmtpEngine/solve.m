function [motionCandidate, solverDiagnostics, directMotion] = solve( ...
    startingPath, planningEnvironment, motionRequest, directMotion)
%% Section 0: Header & Readme
% SYNTAX
%   [motionCandidate, solverDiagnostics, directMotion] = bmtpEngine.solve( ...
%       startingPath, planningEnvironment, motionRequest, directMotion)
%**************************************************************************
% PURPOSE
%   - Turn one proposed path into a smooth motion that respects motion limits.
%   - Alternate between adjusting the curve and lines that keep it clear of
%     obstacles. The planner independently validates the returned motion.
%**************************************************************************
% INPUTS
%   - startingPath (scalar struct)
%       position_units stores route points as N rows of [x y]. The matching
%       tau values describe progress from 0 at the start to 1 at the goal.
%   - planningEnvironment (scalar struct)
%       regions_units contains the convex obstacle regions. coverage records
%       their count and, for timed regions, active intervals and end polygons.
%       The independent validator checks that these regions account for the
%       supplied obstacles throughout the motion.
%   - motionRequest (scalar struct)
%       What to plan: initialState and goalState (normalized position,
%       velocity, and acceleration with their times), limits (normalized
%       workspace, velocity, acceleration, and jerk bounds), and options
%       (resolved goal-time policy, sampling, work limits, and tolerances).
%   - directMotion (scalar struct)
%       Saved direct motion and checks for this request, or struct() initially.
%**************************************************************************
% OUTPUTS
%   - motionCandidate (scalar struct)
%       Stable motion record. Expected infeasibility returns Success = false
%       and may set OptimizerIterateUnavailable = true. Invalid input throws.
%   - solverDiagnostics (scalar struct)
%       Solver progress, timing, motion checks, and obstacle-separation checks.
%   - directMotion (scalar struct)
%       Direct motion, its checks, and their inputs, saved for later reuse.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds. Derivatives use
%     units/s, units/s^2, and units/s^3. Polynomial powers use normalized time.
%**************************************************************************

%% Section 1: Prepare The Solver Inputs And Starting Curve
totalTimer = tic;

% Check the request and collect shared solver settings.
solverRequest = bmtpEngine.pipeline.createSolveRequest(startingPath, planningEnvironment, motionRequest);

initialState  = solverRequest.InitialState;
goalState     = solverRequest.GoalState;
regions_units = solverRequest.Regions_units;
coverage      = solverRequest.Coverage;
limits        = solverRequest.Limits;
options       = solverRequest.Options;

% Convert the route into an initial Bezier curve for the optimizer.
% Its control points define the curve; the completed motion is checked later.
startingCurve = bmtpEngine.pipeline.createWarmStart(solverRequest);

curveDegree           = solverRequest.Degree;
route_units           = startingCurve.Route_units;
segmentCount          = startingCurve.SegmentCount;
regionActiveBySegment = startingCurve.RegionActiveBySegment;

motionCandidate   = createEmptyCandidate(initialState);
solverDiagnostics = createEmptyDiagnostics(curveDegree, segmentCount, numel(regions_units));

solverDiagnostics.OriginalSeedSegmentCount     = startingCurve.OriginalSeedSegmentCount;
solverDiagnostics.WarmRouteResampled           = startingCurve.WarmRouteResampled;
solverDiagnostics.ApplicablePairCount          = nnz(regionActiveBySegment);
solverDiagnostics.MaximumAlternatingIterations = solverRequest.MaximumAlternatingIterations;

endRegions_units = cell(0, 1);
if isfield(coverage, 'EndRegions_units')
    endRegions_units = coverage.EndRegions_units;
end
[~, roundoffReserve_units] = bmtpEngine.validation.createCoordinateTolerances( ...
    route_units, limits.xInterval_units, limits.yInterval_units, ...
    regions_units, endRegions_units);

% Obstacle margins are already included in the regions. This small reserve
% accounts for rounding error in separating-line calculations, including
% the allowed deviation of a unit normal from length 1.
maximumNormalLength      = 1 + 2 ^ 20 * eps;
requiredSeparation_units = maximumNormalLength * options.CollisionClearanceTolerance_units + roundoffReserve_units;

%% Section 2: Try Direct Motion Between The Endpoints
% For an earliest-arrival route with two points and both ends at rest, try
% a straight path that respects speed, acceleration, and jerk limits.
% For fixed arrival, try a degree-5 curve with the requested time and states.
preparedMotion        = struct('Success', false);
motionValidation      = struct('Passed', false);
motionValidationCache = [];
if size(route_units, 1) == 2 && options.GoalTimeMode == "earliestArrival" && solverRequest.IsRest
    [controlPoint_units, segmentTime_s, suppliedPowerCoefficients_units] = bmtpEngine.motion.createC3Chord( ...
        initialState.position_units, goalState.position_units, limits);
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion( ...
        solverRequest, controlPoint_units, segmentTime_s, suppliedPowerCoefficients_units);
    if preparedMotion.Success
        [motionValidation, motionValidationCache] = bmtpEngine.validation.checkFinalMotion(solverRequest, ...
            preparedMotion, roundoffReserve_units, ...
            requiredSeparation_units, motionValidationCache, true);
    end
elseif options.GoalTimeMode == "fixedArrival"
    % Moving obstacles may leave the direct path clear at the required
    % times even when a snapshot suggests a detour. Reuse a previous direct
    % motion check only when all request and geometry inputs still match.
    directMotionKey = struct( ...
        'Regions_units',         {solverRequest.Regions_units}, ...
        'Coverage',              solverRequest.Coverage, ...
        'InitialState',          initialState, ...
        'GoalState',             goalState, ...
        'Limits',                solverRequest.Limits, ...
        'Options',               solverRequest.Options, ...
        'Degree',                curveDegree, ...
        'RoundoffReserve_units', roundoffReserve_units, ...
        'Target_units',          requiredSeparation_units);
    if isfield(directMotion, 'Key') && isequaln(directMotion.Key, directMotionKey)
        preparedMotion        = directMotion.PreparedMotion;
        motionValidation      = directMotion.Proof;
        motionValidationCache = directMotion.Cache;
    else
        [preparedMotion, motionValidation, motionValidationCache] = createFixedArrivalMotion( ...
            solverRequest, roundoffReserve_units, requiredSeparation_units);
        directMotion = struct( ...
            'Key',            directMotionKey, ...
            'PreparedMotion', preparedMotion, ...
            'Proof',          motionValidation, ...
            'Cache',          motionValidationCache);
    end
end

%% Section 3: Check Whether Waiting Before Departure Helps
% For a direct path with moving obstacles and zero endpoint velocity and
% acceleration, waiting before departure may avoid a collision. Try that only
% when immediate departure did not pass and the route has no assigned times.
canTryWaitingBeforeDeparture = size(route_units, 1) == 2 && ...
    options.GoalTimeMode == "earliestArrival" && solverRequest.IsRest && ...
    ~solverRequest.UsesTimeScopedSolver && isfield(coverage, 'ActiveTimeInterval_s');
if canTryWaitingBeforeDeparture && ~(preparedMotion.Success && motionValidation.Passed)
    [controlPoint_units, segmentTime_s, suppliedPowerCoefficients_units, departureTiming] = ...
        bmtpEngine.motion.createDelayedChord(solverRequest);
    solverDiagnostics.DepartureSchedule = departureTiming;
    if isempty(segmentTime_s)
        [motionCandidate, solverDiagnostics] = finishFailure(motionCandidate, solverDiagnostics, totalTimer, struct( ...
            'Message',                  "No C3 chord departure fits the horizon.", ...
            'TerminationReason',        "noDepartureWindow", ...
            'OptimizerFeasible',        false, ...
            'FailureStage',             "timing", ...
            'FailureKind',              "noDepartureWindow", ...
            'AlternativeGuideEligible', true));
        return;
    end
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion( ...
        solverRequest, controlPoint_units, segmentTime_s, suppliedPowerCoefficients_units);
    if ~preparedMotion.Success
        [motionCandidate, solverDiagnostics] = finishFailure(motionCandidate, solverDiagnostics, totalTimer, struct( ...
            'Message',                  preparedMotion.Message, ...
            'TerminationReason',        preparedMotion.TerminationReason, ...
            'OptimizerFeasible',        false, ...
            'FailureStage',             "reconstruction", ...
            'FailureKind',              string(preparedMotion.TerminationReason), ...
            'AlternativeGuideEligible', false));
        return;
    end
    [motionValidation, motionValidationCache] = bmtpEngine.validation.checkFinalMotion( ...
        solverRequest, preparedMotion, roundoffReserve_units, requiredSeparation_units, motionValidationCache);
    if ~motionValidation.Passed
        % Another route may help if obstacle separation is the only failed
        % check. Failures in the motion itself must stop this attempt.
        onlyObstacleSeparationFailed = motionValidation.CoverageMetadataConsistent && ...
            motionValidation.WorkspacePassed && motionValidation.DynamicsPassed && ...
            motionValidation.ContinuityPassed && ...
            motionValidation.VerifiedPairCount < motionValidation.AllPairCount;
        if onlyObstacleSeparationFailed
            [motionCandidate, solverDiagnostics] = finishFailure( ...
                motionCandidate, solverDiagnostics, totalTimer, struct( ...
                'Message',                  "The direct departure family is obstructed " + ...
                    "on this route.", ...
                'TerminationReason',        "departureUncertified", ...
                'OptimizerFeasible',        false, ...
                'FailureStage',             "proposal", ...
                'FailureKind',              "directDepartureObstructed", ...
                'AlternativeGuideEligible', true));
            return;
        end
        [motionCandidate, solverDiagnostics] = finishFailure(motionCandidate, solverDiagnostics, totalTimer, struct( ...
            'Message',                  "The C3 departure proposal did not pass its proof.", ...
            'TerminationReason',        "departureUncertified", ...
            'OptimizerFeasible',        false, ...
            'FailureStage',             "certification", ...
            'FailureKind',              "directDepartureCertificateUnavailable", ...
            'AlternativeGuideEligible', false));
        return;
    end
end

%% Section 4: Optimize The Route If Direct Motion Did Not Pass
if preparedMotion.Success && motionValidation.Passed
    solverDiagnostics.Converged          = true;
    solverDiagnostics.OptimizerSpanCount = 0;
    solverDiagnostics.SegmentCount       = numel(preparedMotion.SegmentTime_s);
else
    % Static earliest arrival uses the solver that adds curve/obstacle pairs
    % as they need separation constraints. Other cases use timed or fixed
    % segment durations, selected below.
    if options.GoalTimeMode == "earliestArrival" && ~isfield(coverage, 'ActiveTimeInterval_s')
        [optimizationResult, solverDiagnostics] = bmtpEngine.optimization.solveActivePairTrajectory( ...
            solverRequest, startingCurve, solverDiagnostics, requiredSeparation_units, roundoffReserve_units);
    else
        % Changing travel times changes which moving obstacles a segment
        % encounters. The timed solver recalculates those overlaps each time;
        % the fixed-duration solver keeps the assigned segment times.
        segmentTimesCanChange = solverRequest.UsesVariableClock;
        if segmentTimesCanChange
            [optimizationResult, solverDiagnostics] = bmtpEngine.optimization.solveTimedAlternatingTrajectory( ...
                solverRequest, startingCurve, solverDiagnostics, requiredSeparation_units, roundoffReserve_units);
            if optimizationResult.Success
                % Try to shorten the control-point polygon while keeping
                % the selected arrival time.
                [refinedTimedResult, solverDiagnostics] = bmtpEngine.pipeline.refineTimedTravel( ...
                    solverRequest, optimizationResult, solverDiagnostics, ...
                    requiredSeparation_units, roundoffReserve_units);
                optimizationResult = refinedTimedResult;
            end
        else
            [optimizationResult, solverDiagnostics] = bmtpEngine.optimization.solveAlternatingTrajectory( ...
                solverRequest, startingCurve, solverDiagnostics, requiredSeparation_units, roundoffReserve_units);
        end
    end
    if ~optimizationResult.Success
        % No optimizer result passed the constraints for this route.
        % Record the failure type so the planner can decide whether another
        % route or arrival time is allowed.
        motionCandidate.OptimizerIterateUnavailable = true;
        failureStage = string(readFailureField( ...
            optimizationResult, "FailureStage", "optimization"));
        failureKind = string(readFailureField( ...
            optimizationResult, "FailureKind", "optimizerIterateUnavailable"));
        canTryAnotherPlanningAttempt = logical(readFailureField( ...
            optimizationResult, "AlternativeGuideEligible", false));
        [motionCandidate, solverDiagnostics] = finishFailure(motionCandidate, solverDiagnostics, totalTimer, struct( ...
            'Message',                  "No optimized collision-free iterate was found. " + ...
                optimizationResult.SolverMessage, ...
            'TerminationReason',        "noOptimizedFeasibleIterate", ...
            'OptimizerFeasible',        false, ...
            'FailureStage',             failureStage, ...
            'FailureKind',              failureKind, ...
            'AlternativeGuideEligible', canTryAnotherPlanningAttempt));
        return;
    end
    if isfield(optimizationResult, 'PreparedMotion') && optimizationResult.PreparedMotion.Success && ...
            isfield(optimizationResult, 'Proof') && optimizationResult.Proof.Passed
        % Reuse these checks with the exact prepared motion they checked.
        preparedMotion   = optimizationResult.PreparedMotion;
        motionValidation = optimizationResult.Proof;
    else
        % Recheck after preparing the final curve: setting endpoint values
        % and converting coefficients can change velocity, acceleration, or jerk.
        preparedMotion = bmtpEngine.pipeline.prepareFinalMotion( ...
            solverRequest, optimizationResult.ControlPoint_units, optimizationResult.SegmentTime_s);
        motionValidation      = struct('Passed', false);
        motionValidationCache = [];
    end
end

%% Section 5: Check The Complete Prepared Motion
solverDiagnostics.DilationScale = preparedMotion.DilationScale;
% Stop if preparation rejected the motion, for example because it exceeds
% the available time or needs too large a change at its curve joins.
if ~preparedMotion.Success
    [motionCandidate, solverDiagnostics] = finishFailure(motionCandidate, solverDiagnostics, totalTimer, struct( ...
        'Message',                  preparedMotion.Message, ...
        'TerminationReason',        preparedMotion.TerminationReason, ...
        'OptimizerFeasible',        true, ...
        'FailureStage',             "reconstruction", ...
        'FailureKind',              string(preparedMotion.TerminationReason), ...
        'AlternativeGuideEligible', false));
    return;
end

% Check the entire curve against each obstacle region that applies.
% Clear sampled points alone do not establish safety between those points.
if ~motionValidation.Passed
    [motionValidation, motionValidationCache] = bmtpEngine.validation.checkFinalMotion( ...
        solverRequest, preparedMotion, roundoffReserve_units, requiredSeparation_units, motionValidationCache);
end

% A separating line keeps a curve segment on one side and an obstacle on
% the other. If one line cannot verify a whole segment, split the segment and
% check its pieces. Splitting preserves the curve and the same tolerances.
for refinementIndex = 1:10
    if motionValidation.Passed || ~motionValidation.WorkspacePassed || ...
            ~motionValidation.DynamicsPassed || ~motionValidation.ContinuityPassed
        break
    end
    % A row is one curve segment; a column is one obstacle region. Split
    % each segment that still has an active pair without a separation proof.
    unverifiedObstaclePairs = ~reshape([motionValidation.Planes.Verified], size(motionValidation.Planes)) & ...
        motionValidation.RegionActiveBySegment;
    splitSegment  = any(unverifiedObstaclePairs, 2);
    splitProgress = repmat(0.5, numel(splitSegment), 1);
    if ~any(splitSegment)
        break
    end
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(solverRequest, ...
        preparedMotion.ControlPoint_units, preparedMotion.SegmentTime_s, ...
        preparedMotion.GivenPower_units, splitSegment, splitProgress);
    [motionValidation, motionValidationCache] = bmtpEngine.validation.checkFinalMotion( ...
        solverRequest, preparedMotion, roundoffReserve_units, requiredSeparation_units, motionValidationCache);
end
solverDiagnostics.SegmentCount            = numel(preparedMotion.SegmentTime_s);
solverDiagnostics.FinalCollisionPairCount = ...
    motionValidation.AllPairCount - motionValidation.VerifiedPairCount;
solverDiagnostics.MotionProof     = preparedMotion.MotionProof;
solverDiagnostics.SeparationProof = motionValidation;
motionCandidate.SeparationProof   = motionValidation;

% Convert the checked curve to the public motion format and sample it.
motionCandidate = bmtpEngine.pipeline.createMotionOutput(motionCandidate, solverRequest, preparedMotion);
motionCandidate.OptimizerFeasible = true;

% Solver success is not enough: the completed motion must also pass
% the continuous collision, workspace, speed, acceleration, jerk, and join checks.
if ~motionValidation.Passed
    failureStage = "certification";
    failureKind  = "planeCertificateUnavailable";
    canTryAnotherPlanningAttempt = false;
    geometryProofPassed = motionValidation.CoverageMetadataConsistent && ...
        motionValidation.VerifiedPairCount == motionValidation.AllPairCount;
    if geometryProofPassed && motionValidation.WorkspacePassed && ...
            ~motionValidation.DynamicsPassed && motionValidation.ContinuityPassed
        % The path is clear, but speed, acceleration, or jerk exceeds a
        % limit. Report this timing failure so the planner can decide whether
        % another arrival time is allowed.
        failureStage = "timing";
        failureKind  = "kinematicCertificateUnavailable";
        canTryAnotherPlanningAttempt = true;
    elseif motionValidation.WorkspacePassed && motionValidation.DynamicsPassed && ...
            ~motionValidation.ContinuityPassed
        failureStage = "reconstruction";
        failureKind  = "continuityCertificateUnavailable";
    elseif ~motionValidation.WorkspacePassed
        failureKind = "workspaceCertificateUnavailable";
    end
    [motionCandidate, solverDiagnostics] = finishFailure(motionCandidate, solverDiagnostics, totalTimer, struct( ...
        'Message',                  "The optimized motion failed its continuous collision, " + ...
            "dynamics, or C3 continuity proof.", ...
        'TerminationReason',        "planeCertificateUnavailable", ...
        'OptimizerFeasible',        true, ...
        'FailureStage',             failureStage, ...
        'FailureKind',              failureKind, ...
        'AlternativeGuideEligible', canTryAnotherPlanningAttempt));
    return;
end

%% Section 6: Return The Motion That Passed The Engine Checks
[motionCandidate.Message, motionCandidate.TerminationReason] = deal( ...
    "A directly proven BMTP trajectory was found.", "goalReached");
[motionCandidate.Success, solverDiagnostics.Accepted]        = deal(true);
motionCandidate.FailureStage             = "";
motionCandidate.FailureKind              = "";
motionCandidate.AlternativeGuideEligible = false;
solverDiagnostics.ElapsedTime_s          = toc(totalTimer);
end

%% Section 7: Local Functions
function [preparedMotion, motionValidation, motionValidationCache] = createFixedArrivalMotion( ...
    solverRequest, roundoffReserve_units, requiredSeparation_units)
    % Build and check a single curve that meets the requested arrival time.
    % Nonzero endpoint velocity or acceleration can bend this curve.
    curveDegree           = solverRequest.Degree;
    initialState          = solverRequest.InitialState;
    goalState             = solverRequest.GoalState;
    motionValidation      = struct('Passed', false);
    motionValidationCache = [];

    % With zero endpoint velocity and acceleration, progress along the
    % straight path is 10 x tau^3 - 15 x tau^4 + 6 x tau^5, for tau from 0 to 1.
    controlPointFractions = bmtpEngine.motion.powerToBernstein([0; 0; 0; 10; -15; 6], curveDegree);
    controlPoint_units    = initialState.position_units + ...
        controlPointFractions .* (goalState.position_units - initialState.position_units);
    if ~solverRequest.IsRest
        % The first three polynomial coefficients set the initial position,
        % velocity, and acceleration. Solve for the remaining three at the goal.
        duration_s          = solverRequest.MotionHorizon_s;
        positionPower_units = [initialState.position_units; ...
            duration_s * initialState.velocity_units_s; ...
            duration_s ^ 2 * initialState.acceleration_units_s2 / 2; ...
            zeros(3, 2)];
        remainingEndpointValues_units = [goalState.position_units - sum(positionPower_units, 1); ...
            duration_s * goalState.velocity_units_s - positionPower_units(2, :) - 2 * positionPower_units(3, :); ...
            duration_s ^ 2 * goalState.acceleration_units_s2 - 2 * positionPower_units(3, :)];

        % Rows enforce final position, velocity x duration, and acceleration
        % x duration^2. Columns multiply the u^3, u^4, and u^5 coefficients.
        positionPower_units(4:6, :) = [1 1 1; 3 4 5; 6 12 20] \ remainingEndpointValues_units;
        controlPoint_units          = bmtpEngine.motion.powerToBernstein(positionPower_units, curveDegree);
    end
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(solverRequest, ...
        reshape(controlPoint_units, 1, curveDegree + 1, 2), solverRequest.MotionHorizon_s);
    if preparedMotion.Success
        [motionValidation, motionValidationCache] = bmtpEngine.validation.checkFinalMotion(solverRequest, ...
            preparedMotion, roundoffReserve_units, requiredSeparation_units, motionValidationCache, true);
    end
end

function motionCandidate = createEmptyCandidate(initialState)
    % Use the same candidate fields on success and failure.
    dimensionCount = numel(initialState.position_units);
    motionCandidate = struct();
    motionCandidate.Success                         = false;
    motionCandidate.OptimizerFeasible               = false;
    motionCandidate.OptimizerIterateUnavailable     = false;
    motionCandidate.AlternativeGuideEligible        = false;
    motionCandidate.FailureStage                    = "notRun";
    motionCandidate.FailureKind                     = "notRun";
    motionCandidate.Message                         = "The BMTP engine was not run.";
    motionCandidate.TerminationReason               = "notRun";
    motionCandidate.ArrivalTime_s                   = NaN;
    motionCandidate.TrajectoryDuration_s            = NaN;
    motionCandidate.MotionLength_units              = Inf;
    motionCandidate.IntegratedSquaredJerk_units2_s5 = Inf;
    motionCandidate.MaximumConstraintViolation      = Inf;
    motionCandidate.time_s                          = zeros(0, 1);
    motionCandidate.position_units                  = zeros(0, dimensionCount);
    motionCandidate.velocity_units_s                = zeros(0, dimensionCount);
    motionCandidate.acceleration_units_s2           = zeros(0, dimensionCount);
    motionCandidate.jerk_units_s3                   = zeros(0, dimensionCount);
    motionCandidate.Polynomial                      = struct();
    motionCandidate.SeparationProof                 = struct();
end

function fieldValue = readFailureField(failureDetails, fieldName, defaultValue)
    % Some solvers omit optional failure details. Use the supplied default
    % when the requested field is absent or empty.
    fieldValue = defaultValue;
    if isfield(failureDetails, fieldName) && ~isempty(failureDetails.(fieldName))
        fieldValue = failureDetails.(fieldName);
    end
end

function solverDiagnostics = createEmptyDiagnostics(curveDegree, segmentCount, regionCount)
    % Keep the same diagnostic fields for early failures and completed solves.
    % Final preparation initially splits each curve segment into two pieces.
    solverDiagnostics = struct( ...
        "Accepted",                 false, ...
        "Degree",                   curveDegree, ...
        "OriginalSeedSegmentCount", segmentCount, ...
        "WarmRouteResampled",       false, ...
        "OptimizerSpanCount",       segmentCount, ...
        "SegmentCount",             2 * segmentCount, ...
        "ExactRegionCount",         regionCount, ...
        "IterationCount",           0, ...
        "Converged",                false, ...
        "ApplicablePairCount",      segmentCount * regionCount, ...
        "TrajectorySocpCount",      0, ...
        "FinalCollisionPairCount",  0, ...
        "PlaneSocpCount",           0, ...
        "DilationScale",            NaN, ...
        "MotionProof",              struct(), ...
        "SeparationProof",          struct(), ...
        "SolverMessage",            "", ...
        "ElapsedTime_s",            0, ...
        "ConicSolver",              bmtpEngine.optimization.accumulateConicDiagnostics());
end

function [motionCandidate, solverDiagnostics] = finishFailure(motionCandidate, solverDiagnostics, totalTimer, failureDetails)
    % Put the failure reason in both the motion result and solver diagnostics.
    % The planner uses this reason to decide whether another attempt is allowed.
    motionCandidate.Message                    = failureDetails.Message;
    motionCandidate.TerminationReason          = failureDetails.TerminationReason;
    motionCandidate.OptimizerFeasible          = failureDetails.OptimizerFeasible;
    motionCandidate.FailureStage               = string(failureDetails.FailureStage);
    motionCandidate.FailureKind                = string(failureDetails.FailureKind);
    motionCandidate.AlternativeGuideEligible   = logical(failureDetails.AlternativeGuideEligible);
    solverDiagnostics.FailureStage             = motionCandidate.FailureStage;
    solverDiagnostics.FailureKind              = motionCandidate.FailureKind;
    solverDiagnostics.AlternativeGuideEligible = motionCandidate.AlternativeGuideEligible;
    [solverDiagnostics.Accepted, solverDiagnostics.ElapsedTime_s] = deal(false, toc(totalTimer));
end
