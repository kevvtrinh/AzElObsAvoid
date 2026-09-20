function [result, diagnostics] = solveAlternatingTrajectory( ...
    request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.optimization.solveAlternatingTrajectory(request, ...
%       warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Alternate one trajectory SOCP with exact all-pair separating-line SOCPs
%     until a completely verified motion is found. Timed cells constrain only
%     their exact overlap with each fixed-duration motion span.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Checked BMTP request with a fixed arrival clock.
%   - warmStart (scalar struct)
%       Seed controls, span durations, and the active region-by-segment mask.
%   - diagnostics (scalar struct)
%       Solver diagnostics accumulated so far in this solve.
%   - obstacleTarget_units (finite scalar)
%       Required obstacle-side separation target.
%   - roundoffReserve_units (finite scalar)
%       Numerical separation reserve.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Best all-pair-verified controls and per-segment durations, with the
%       constraint planes the returned motion satisfied. A plane's gap is the
%       one measured when it was built or last verified; a motion accepted by
%       the constraint-row proof keeps those planes unchanged, and the final
%       motion is proven again independently. An expected infeasible solve
%       returns Success = false with empty controls. Invalid input throws an
%       error.
%   - diagnostics (scalar struct)
%       Solver counts, residual pair counts, and termination data.
%**************************************************************************
% UNITS
%   - Position and clearance are coordinate units; time is seconds.
%**************************************************************************

%% Section 1: Initialize Every Segment-Obstacle Pair
assert(request.Options.GoalTimeMode == "fixedArrival", ...
    'bmtpEngine:InvalidFixedClockSolve', ...
    'The all-pair alternating solver requires a fixed arrival clock.');
segmentCount                  = warmStart.SegmentCount;
regionCount                   = numel(request.Regions_units);
request.RegionActiveBySegment = warmStart.RegionActiveBySegment;

diagnostics.ConicSolver                        = bmtpEngine.optimization.accumulateConicDiagnostics();
diagnostics.ExistingPlanePairVerificationCount = 0;
diagnostics.ConstraintRowPairVerificationCount = 0;
diagnostics.FullPlaneUpdateSkippedCount        = 0;

emptyPlane = bmtpEngine.separation.createEmptyPlane();
planes     = repmat(emptyPlane, segmentCount, regionCount);
[planes, allPlanesActive, ~, diagnostics] = updatePlanes(warmStart.ControlPoint_units, ...
    warmStart.SegmentTime_s, planes, request, diagnostics, ...
    obstacleTarget_units, roundoffReserve_units, false);

selectedControl_units = zeros(0, request.Degree + 1, 2);
selectedSegmentTime_s = NaN;
solverMessage         = "The all-pair alternating iteration limit was reached.";
failureStage          = "optimization";
failureKind           = "iterationLimit";
alternativeGuideEligible = true;
meshRefinementCount   = 0;
useIntrinsicVariation = false;
stageIterationCount = zeros(1, 2);
constraintBase      = struct();

%% Section 2: Alternate The Complete Formulation
if allPlanesActive
    while stageIterationCount(1 + useIntrinsicVariation) < ...
            request.MaximumAlternatingIterations
        stageIndex = 1 + useIntrinsicVariation;
        stageIterationCount(stageIndex) = stageIterationCount(stageIndex) + 1;
        diagnostics.IterationCount = sum(stageIterationCount);
        step = struct( ...
            'SegmentCount',              segmentCount, ...
            'Planes',                    planes, ...
            'RoundoffReserve_units',     roundoffReserve_units, ...
            'MaximumMotionDuration_s',   request.MotionHorizon_s, ...
            'SegmentRatio',              warmStart.SegmentRatio, ...
            'FixedClock',                true, ...
            'IntrinsicVariationEnabled', useIntrinsicVariation, ...
            'ConstraintBase',            constraintBase);
        [trialControl_units, trialTime_s, exitFlag, output, constraintBase] = ...
            bmtpEngine.optimization.solveTrajectoryStep(request, step);
        diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + output.SolveCount;
        diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
        diagnostics.IntrinsicJerkVariation = output.IntrinsicJerkVariation;
        if isfield(output, 'ConstraintGenerationApplied') && ...
                output.ConstraintGenerationApplied
            diagnostics.LoadedPlanePairCount           = output.LoadedPlanePairCount;
            diagnostics.ConstraintGenerationRoundCount = output.ConstraintGenerationRoundCount;
            diagnostics.ConstraintGenerationComplete   = output.ConstraintGenerationComplete;
            diagnostics.MaximumPlaneConstraintResidual = output.MaximumPlaneConstraintResidual;
        end
        if isfield(output, 'MaximumClearanceSlack_units')
            diagnostics.MaximumClearanceSlack_units = output.MaximumClearanceSlack_units;
        end
        if ~bmtpEngine.optimization.hasUsableConicIterate(trialControl_units, exitFlag)
            solverMessage = "Trajectory SOCP failed: " + string(output.message);
            diagnostics.LastTrajectoryExitFlag = exitFlag;
            if exitFlag == 0
                failureStage  = "optimization";
                failureKind   = "trajectorySolverIterationLimit";
                alternativeGuideEligible = true;
            elseif exitFlag == -2
                failureStage  = "proposal";
                failureKind   = "trajectorySubproblemInfeasible";
                alternativeGuideEligible = true;
            else
                failureStage  = "numericalSolver";
                failureKind   = "optimizerIterateUnavailable";
                alternativeGuideEligible = false;
            end
            break;
        end

        rowProofComplete = isfield(output, 'ConstraintGenerationApplied') && ...
            output.ConstraintGenerationApplied && output.ConstraintGenerationComplete && ...
            output.RetainedPlaneCount == output.OriginalPlaneCount && ...
            isfield(output, 'MaximumClearanceSlack_units') && ...
            ~isempty(output.MaximumClearanceSlack_units) && ...
            output.MaximumClearanceSlack_units + ...
            max(0, output.MaximumPlaneConstraintResidual) <= roundoffReserve_units;
        if rowProofComplete
            % Every plane's obstacle side was fixed and proven when it was
            % constructed. Zero-reserve elastic slack plus complete exact row
            % separation proves the trajectory side for every pair directly,
            % so no per-pair verification runs and the planes keep the gaps
            % measured on the curve they were built from.
            updatedPlanes     = planes;
            verifiedPairs     = true(size(request.RegionActiveBySegment));
            verifiedPairCount = nnz(request.RegionActiveBySegment);
            diagnostics.ConstraintRowPairVerificationCount = ...
                diagnostics.ConstraintRowPairVerificationCount + verifiedPairCount;
        else
            [updatedPlanes, ~, verifiedPairs, diagnostics, verifiedPairCount] = ...
                updatePlanes(trialControl_units, trialTime_s, planes, request, ...
                diagnostics, obstacleTarget_units, roundoffReserve_units, true);
            diagnostics.ExistingPlanePairVerificationCount = ...
                diagnostics.ExistingPlanePairVerificationCount + verifiedPairCount;
        end
        allPlanesActive = all(verifiedPairs, 'all');
        if ~allPlanesActive
            [updatedPlanes, allPlanesActive, verifiedPairs, diagnostics] = ...
                updatePlanes(trialControl_units, trialTime_s, planes, request, ...
                diagnostics, obstacleTarget_units, roundoffReserve_units, false);
        else
            diagnostics.FullPlaneUpdateSkippedCount = diagnostics.FullPlaneUpdateSkippedCount + 1;
        end
        planes = updatedPlanes;
        unverifiedPairCount = nnz(~verifiedPairs);
        diagnostics.FinalCollisionPairCount = unverifiedPairCount;
        if ~allPlanesActive
            solverMessage = "A complete separating-line update failed.";
            failureStage  = "proposal";
            failureKind   = "separatingLineUpdateUnavailable";
            break;
        end

        if unverifiedPairCount > 0
            if meshRefinementCount < 3
                splitMask = any(~verifiedPairs, 2);
                [refinedControl_units, refinedTime_s] = bisectSelectedSpans( ...
                    trialControl_units, trialTime_s, splitMask);
                meshRefinementCount = meshRefinementCount + 1;
                segmentCount = numel(refinedTime_s);
                diagnostics.OptimizerSpanCount = segmentCount;
                warmStart.SegmentRatio = refinedTime_s / mean(refinedTime_s);
                request.RegionActiveBySegment = true(segmentCount, regionCount);
                if isfield(request.Coverage, 'ActiveTimeInterval_s')
                    breaks_s    = request.InitialState.time_s + [0; cumsum(refinedTime_s)];
                    intervals_s = request.Coverage.ActiveTimeInterval_s;
                    request.RegionActiveBySegment = breaks_s(1:end - 1) < intervals_s(:, 2).' & ...
                        breaks_s(2:end) > intervals_s(:, 1).';
                end
                diagnostics.ApplicablePairCount = nnz(request.RegionActiveBySegment);
                useIntrinsicVariation = false;
                constraintBase        = struct();
                planes = repmat(emptyPlane, segmentCount, regionCount);
                [planes, allPlanesActive, ~, diagnostics] = updatePlanes(refinedControl_units, ...
                    refinedTime_s, planes, request, diagnostics, ...
                    obstacleTarget_units, roundoffReserve_units, false);
                if ~allPlanesActive
                    solverMessage = "A refined separating-line initialization failed.";
                    failureStage  = "proposal";
                    failureKind   = "refinedSeparatingLineInitializationUnavailable";
                    break
                end
            end
            continue;
        end

        % Collision feasibility chooses the earliest clock. Only after a
        % complete corridor is established does the unchanged integrated-snap
        % objective refine that clock's motion. Earlier infeasible clocks
        % cannot benefit from paying for this tie-break.
        variationStageIsRequired = request.Degree == 5 && segmentCount > 8;
        if variationStageIsRequired && ~useIntrinsicVariation
            useIntrinsicVariation = true;
            constraintBase        = struct();
            continue;
        end

        selectedControl_units       = trialControl_units;
        selectedSegmentTime_s       = trialTime_s;
        diagnostics.Converged      = output.OptimizationConverged;
        solverMessage              = "A complete all-pair-verified iterate was found.";
        failureStage               = "";
        failureKind                = "";
        alternativeGuideEligible  = false;
        break;
    end
else
    solverMessage = "The visibility seed did not produce a complete separating-line initialization.";
    failureStage  = "proposal";
    failureKind   = "separatingLineInitializationUnavailable";
end

%% Section 3: Return The Best Fully Verified Iterate
diagnostics.SolverMessage = solverMessage;
result = struct();
result.Success            = ~isempty(selectedControl_units);
result.SolverMessage      = solverMessage;
result.FailureStage       = failureStage;
result.FailureKind        = failureKind;
result.AlternativeGuideEligible = alternativeGuideEligible;
result.ControlPoint_units = selectedControl_units;
result.SegmentTime_s      = selectedSegmentTime_s;
result.Planes             = planes;
result.TaggedPairs        = reshape([planes.Active], size(planes));
end

%% Section 4: Local Functions
function [refinedControl_units, refinedTime_s] = bisectSelectedSpans(controlPoint_units, segmentTime_s, splitMask)
    % Split each masked span into two exact Bezier halves of equal duration.
    % Unmasked spans keep their controls and duration unchanged, so the
    % refined motion is the same curve on a finer mesh.
    refinedSpanCount     = numel(segmentTime_s) + nnz(splitMask);
    refinedControl_units = zeros(refinedSpanCount, size(controlPoint_units, 2), 2);
    refinedTime_s        = zeros(refinedSpanCount, 1);
    targetIndex          = 0;
    for spanIndex = 1:numel(segmentTime_s)
        if splitMask(spanIndex)
            for halfIndex = 1:2
                targetIndex = targetIndex + 1;
                refinedControl_units(targetIndex, :, :) = bmtpEngine.motion.restrictBezier( ...
                    squeeze(controlPoint_units(spanIndex, :, :)), [(halfIndex - 1) / 2, halfIndex / 2]);
                refinedTime_s(targetIndex) = segmentTime_s(spanIndex) / 2;
            end
        else
            targetIndex = targetIndex + 1;
            refinedControl_units(targetIndex, :, :) = controlPoint_units(spanIndex, :, :);
            refinedTime_s(targetIndex) = segmentTime_s(spanIndex);
        end
    end
end

function [planes, allActive, verifiedPairs, diagnostics, verifiedPairCount] = updatePlanes( ...
    controlPoint_units, segmentTime_s, planes, request, diagnostics, ...
    target_units, roundoffReserve_units, verifyOnly)
    % Update every active curve-region pair without sampled discovery or pruning.
    % Inactive pairs count as verified so the caller sees only real failures.
    % The first unusable pair stops the sweep with allActive false.
    verifiedPairs     = ~request.RegionActiveBySegment;
    allActive         = true;
    verifiedPairCount = 0;
    breaks_s = request.InitialState.time_s + [0; cumsum(segmentTime_s)];
    for segmentIndex = 1:size(planes, 1)
        for regionIndex = 1:size(planes, 2)
            if ~request.RegionActiveBySegment(segmentIndex, regionIndex)
                continue
            end
            controls_units = squeeze(controlPoint_units(segmentIndex, :, :));
            timeFraction   = [0, 1];
            interval_s     = breaks_s(segmentIndex:segmentIndex + 1).';
            if isfield(request.Coverage, 'ActiveTimeInterval_s')
                active_s   = request.Coverage.ActiveTimeInterval_s(regionIndex, :);
                interval_s = [max(interval_s(1), active_s(1)), min(interval_s(2), active_s(2))];
                if interval_s(2) <= interval_s(1)
                    % Adjacent closed cells can meet a span at one instant.
                    % That zero-measure contact creates no trajectory
                    % constraint and must not become a degenerate scope.
                    plane              = planes(segmentIndex, regionIndex);
                    plane.Active       = false;
                    plane.Verified     = false;
                    plane.TimeFraction = [0, 1];
                    planes(segmentIndex, regionIndex) = plane;
                    verifiedPairs(segmentIndex, regionIndex) = true;
                    continue
                end
                timeFraction = max(0, min(1, ...
                    (interval_s - breaks_s(segmentIndex)) / segmentTime_s(segmentIndex)));
                controls_units = bmtpEngine.motion.restrictBezier(controls_units, timeFraction);
            end
            vertices_units = bmtpEngine.separation.regionOnInterval(request.Regions_units{regionIndex}, ...
                request.Coverage, regionIndex, interval_s);
            if verifyOnly
                plane  = planes(segmentIndex, regionIndex);
                normal = plane.Normal(1, :);
                plane.Offset_units = target_units - [min(vertices_units(:, :, 1) * normal.'), ...
                    min(vertices_units(:, :, end) * normal.')];
                plane.TimeFraction = timeFraction;
                plane = bmtpEngine.separation.verifySeparatingLine( ...
                    plane, controls_units, vertices_units, roundoffReserve_units, target_units);
                verifiedPairCount = verifiedPairCount + 1;
            else
                [plane, exitFlag, output] = bmtpEngine.separation.solveSeparatingLine( ...
                    controls_units, vertices_units, target_units, roundoffReserve_units);
                plane.TimeFraction = timeFraction;
                diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + ...
                    ~(isfield(output, 'IsAnalytic') && output.IsAnalytic);
                diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
            end
            planes(segmentIndex, regionIndex) = plane;
            verifiedPairs(segmentIndex, regionIndex) = plane.Verified;
            if verifyOnly
                if ~plane.Verified
                    allActive = false;
                    return
                end
            elseif exitFlag <= 0 || ~plane.Active
                allActive = false;
                return;
            end
        end
    end
end
