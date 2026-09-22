function [result, diagnostics] = solveAlternatingTrajectory( ...
    solverRequest, warmStart, diagnostics, separationTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.optimization.solveAlternatingTrajectory( ...
%       solverRequest, warmStart, diagnostics, separationTarget_units, roundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Alternate between improving the curve and finding separating lines
%     for every applicable curve-segment/obstacle pair. The arrival time stays
%     fixed. Each obstacle constrains only the time when it overlaps a segment.
%   - A separating line keeps the curve and obstacle on opposite sides. The
%     caller still checks the complete motion and runs public validation.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked BMTP inputs with a fixed arrival time.
%   - warmStart (scalar struct)
%       Starting curve controls, segment durations, and applicable obstacle pairs.
%   - diagnostics (scalar struct)
%       Solver diagnostics accumulated so far in this solve.
%   - separationTarget_units (finite scalar)
%       Required distance from each obstacle to its separating line.
%   - roundoffReserve_units (finite scalar)
%       Numerical separation reserve.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Selected controls, durations, and separating lines after all applicable
%       obstacle pairs pass. Each line's recorded gap comes from its latest
%       construction or direct check; acceptance through constraint rows does
%       not recalculate that gap. The final motion is checked again by the caller.
%       Expected infeasibility returns Success = false with empty controls.
%       Invalid input throws an error.
%   - diagnostics (scalar struct)
%       Solver counts, remaining unverified pairs, and termination data.
%**************************************************************************
% UNITS
%   - Position and clearance are coordinate units; time is seconds.
%**************************************************************************

%% Section 1: Initialize Every Applicable Obstacle Pair

% Build lines around the starting curve before the first trajectory solve.
% Any pair without a usable line prevents this attempt from starting.
assert(solverRequest.Options.GoalTimeMode == "fixedArrival", ...
    'bmtpEngine:InvalidFixedClockSolve', ...
    'The all-pair alternating solver requires a fixed arrival clock.');
segmentCount = warmStart.SegmentCount;
regionCount  = numel(solverRequest.Regions_units);
solverRequest.RegionActiveBySegment = warmStart.RegionActiveBySegment;

diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics();

diagnostics.ExistingPlanePairVerificationCount = 0;
diagnostics.ConstraintRowPairVerificationCount = 0;
diagnostics.FullPlaneUpdateSkippedCount        = 0;

emptyPlane       = bmtpEngine.separation.createEmptyPlane();
separatingPlanes = repmat(emptyPlane, segmentCount, regionCount);
[separatingPlanes, allRequiredLinesAvailable, ~, diagnostics] = ...
    updateSeparatingLines(warmStart.ControlPoint_units, ...
    warmStart.SegmentTime_s, separatingPlanes, solverRequest, diagnostics, ...
    separationTarget_units, roundoffReserve_units, false);

selectedControl_units      = zeros(0, solverRequest.Degree + 1, 2);
selectedSegmentTime_s      = NaN;
solverMessage              = "The all-pair alternating iteration limit was reached.";
failureStage               = "optimization";
failureKind                = "iterationLimit";
alternativeGuideEligible   = true;
segmentSplitCount          = 0;
includeJerkVariation       = false;
stageIterationCount        = zeros(1, 2);
savedTrajectoryConstraints = struct();

%% Section 2: Improve The Curve And Check Every Obstacle Pair

% First seek obstacle separation. A later stage may add a jerk-variation
% objective. Each stage has its own iteration count, so smoothing does not
% consume the iterations available to establish separation.
if allRequiredLinesAvailable
    while stageIterationCount(1 + includeJerkVariation) < ...
            solverRequest.MaximumAlternatingIterations
        stageIndex = 1 + includeJerkVariation;
        stageIterationCount(stageIndex) = stageIterationCount(stageIndex) + 1;
        diagnostics.IterationCount = sum(stageIterationCount);
        % Keep segment-duration ratios and the arrival time fixed for this solve.
        trajectoryStep = struct( ...
            'SegmentCount',              segmentCount, ...
            'Planes',                    separatingPlanes, ...
            'RoundoffReserve_units',     roundoffReserve_units, ...
            'MaximumMotionDuration_s',   solverRequest.MotionHorizon_s, ...
            'SegmentRatio',              warmStart.SegmentRatio, ...
            'FixedClock',                true, ...
            'IntrinsicVariationEnabled', includeJerkVariation, ...
            'ConstraintBase',            savedTrajectoryConstraints);
        [trialControl_units, trialSegmentTime_s, exitFlag, solverOutput, savedTrajectoryConstraints] = ...
            bmtpEngine.optimization.solveTrajectoryStep(solverRequest, trajectoryStep);
        diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + solverOutput.SolveCount;
        diagnostics.ConicSolver         = bmtpEngine.optimization.accumulateConicDiagnostics( ...
            diagnostics.ConicSolver, solverOutput);
        diagnostics.IntrinsicJerkVariation = solverOutput.IntrinsicJerkVariation;
        if isfield(solverOutput, 'ConstraintGenerationApplied') && ...
                solverOutput.ConstraintGenerationApplied
            diagnostics.LoadedPlanePairCount           = solverOutput.LoadedPlanePairCount;
            diagnostics.ConstraintGenerationRoundCount = solverOutput.ConstraintGenerationRoundCount;
            diagnostics.ConstraintGenerationComplete   = solverOutput.ConstraintGenerationComplete;
            diagnostics.MaximumPlaneConstraintResidual = solverOutput.MaximumPlaneConstraintResidual;
        end
        if isfield(solverOutput, 'MaximumClearanceSlack_units')
            diagnostics.MaximumClearanceSlack_units = solverOutput.MaximumClearanceSlack_units;
        end
        if ~bmtpEngine.optimization.hasUsableConicIterate(trialControl_units, exitFlag)
            solverMessage = "Trajectory SOCP failed: " + string(solverOutput.message);
            diagnostics.LastTrajectoryExitFlag = exitFlag;
            if exitFlag == 0
                failureStage             = "optimization";
                failureKind              = "trajectorySolverIterationLimit";
                alternativeGuideEligible = true;
            elseif exitFlag == -2
                failureStage             = "proposal";
                failureKind              = "trajectorySubproblemInfeasible";
                alternativeGuideEligible = true;
            else
                failureStage             = "numericalSolver";
                failureKind              = "optimizerIterateUnavailable";
                alternativeGuideEligible = false;
            end
            break;
        end

        % Slack lets the solver temporarily relax a line constraint. All
        % original lines must still be checked, and maximum slack + maximum
        % row violation must fit within the reserved trajectory-side gap
        % before those rows establish separation.
        allLineConstraintsProved = isfield(solverOutput, 'ConstraintGenerationApplied') && ...
            solverOutput.ConstraintGenerationApplied && solverOutput.ConstraintGenerationComplete && ...
            solverOutput.RetainedPlaneCount == solverOutput.OriginalPlaneCount && ...
            isfield(solverOutput, 'MaximumClearanceSlack_units') && ...
            ~isempty(solverOutput.MaximumClearanceSlack_units) && ...
            solverOutput.MaximumClearanceSlack_units + ...
            max(0, solverOutput.MaximumPlaneConstraintResidual) <= roundoffReserve_units;
        if allLineConstraintsProved
            % The obstacle side was checked when each line was built. The
            % complete constraint rows now prove the curve side, so the lines
            % can be retained without separately checking every pair again.
            updatedSeparatingPlanes = separatingPlanes;
            verifiedPairs           = true(size(solverRequest.RegionActiveBySegment));
            verifiedPairCount       = nnz(solverRequest.RegionActiveBySegment);
            diagnostics.ConstraintRowPairVerificationCount = ...
                diagnostics.ConstraintRowPairVerificationCount + verifiedPairCount;
        else
            % Try the existing line directions before solving for new lines.
            [updatedSeparatingPlanes, ~, verifiedPairs, diagnostics, verifiedPairCount] = ...
                updateSeparatingLines(trialControl_units, trialSegmentTime_s, separatingPlanes, solverRequest, ...
                diagnostics, separationTarget_units, roundoffReserve_units, true);
            diagnostics.ExistingPlanePairVerificationCount = ...
                diagnostics.ExistingPlanePairVerificationCount + verifiedPairCount;
        end
        allRequiredLinesAvailable = all(verifiedPairs, 'all');
        if ~allRequiredLinesAvailable
            % At least one existing line failed; rebuild the complete set.
            [updatedSeparatingPlanes, allRequiredLinesAvailable, verifiedPairs, diagnostics] = ...
                updateSeparatingLines(trialControl_units, trialSegmentTime_s, separatingPlanes, solverRequest, ...
                diagnostics, separationTarget_units, roundoffReserve_units, false);
        else
            diagnostics.FullPlaneUpdateSkippedCount = diagnostics.FullPlaneUpdateSkippedCount + 1;
        end
        separatingPlanes    = updatedSeparatingPlanes;
        unverifiedPairCount = nnz(~verifiedPairs);
        diagnostics.FinalCollisionPairCount = unverifiedPairCount;
        if ~allRequiredLinesAvailable
            solverMessage = "A complete separating-line update failed.";
            failureStage  = "proposal";
            failureKind   = "separatingLineUpdateUnavailable";
            break;
        end

        % A line can be available without yet proving its whole curve piece
        % clear. Split those segments into equal-time halves, up to three
        % times, so each smaller piece can use its own separating line.
        if unverifiedPairCount > 0
            if segmentSplitCount < 3
                splitSegment = any(~verifiedPairs, 2);
                [refinedControl_units, refinedSegmentTime_s] = splitSelectedSegments( ...
                    trialControl_units, trialSegmentTime_s, splitSegment);
                segmentSplitCount = segmentSplitCount + 1;
                segmentCount      = numel(refinedSegmentTime_s);
                diagnostics.OptimizerSpanCount = segmentCount;
                warmStart.SegmentRatio = refinedSegmentTime_s / mean(refinedSegmentTime_s);
                % Recalculate which obstacle lifetimes overlap the new segments.
                solverRequest.RegionActiveBySegment = true(segmentCount, regionCount);
                if isfield(solverRequest.Coverage, 'ActiveTimeInterval_s')
                    segmentBoundaryTime_s = ...
                        solverRequest.InitialState.time_s + [0; cumsum(refinedSegmentTime_s)];
                    obstacleActiveIntervals_s = solverRequest.Coverage.ActiveTimeInterval_s;
                    solverRequest.RegionActiveBySegment = segmentBoundaryTime_s(1:end - 1) < obstacleActiveIntervals_s(:, 2).' & ...
                        segmentBoundaryTime_s(2:end) > obstacleActiveIntervals_s(:, 1).';
                end
                diagnostics.ApplicablePairCount = nnz(solverRequest.RegionActiveBySegment);
                includeJerkVariation            = false;
                savedTrajectoryConstraints      = struct();
                separatingPlanes                = repmat(emptyPlane, segmentCount, regionCount);
                [separatingPlanes, allRequiredLinesAvailable, ~, diagnostics] = ...
                    updateSeparatingLines(refinedControl_units, ...
                    refinedSegmentTime_s, separatingPlanes, solverRequest, diagnostics, ...
                    separationTarget_units, roundoffReserve_units, false);
                if ~allRequiredLinesAvailable
                    solverMessage = "A refined separating-line initialization failed.";
                    failureStage  = "proposal";
                    failureKind   = "refinedSeparatingLineInitializationUnavailable";
                    break
                end
            end
            continue;
        end

        % For quintic curves with more than eight segments, improve jerk
        % smoothness only after all obstacle pairs pass. Snap measures how
        % quickly jerk changes; this stage keeps the same arrival time.
        jerkVariationStageIsRequired = solverRequest.Degree == 5 && segmentCount > 8;
        if jerkVariationStageIsRequired && ~includeJerkVariation
            includeJerkVariation       = true;
            savedTrajectoryConstraints = struct();
            continue;
        end

        selectedControl_units    = trialControl_units;
        selectedSegmentTime_s    = trialSegmentTime_s;
        diagnostics.Converged    = solverOutput.OptimizationConverged;
        solverMessage            = "A complete all-pair-verified iterate was found.";
        failureStage             = "";
        failureKind              = "";
        alternativeGuideEligible = false;
        break;
    end
else
    solverMessage = "The visibility seed did not produce a complete separating-line initialization.";
    failureStage  = "proposal";
    failureKind   = "separatingLineInitializationUnavailable";
end

%% Section 3: Return The Selected Candidate And Its Separating Lines

diagnostics.SolverMessage = solverMessage;
result = struct();
result.Success                  = ~isempty(selectedControl_units);
result.SolverMessage            = solverMessage;
result.FailureStage             = failureStage;
result.FailureKind              = failureKind;
result.AlternativeGuideEligible = alternativeGuideEligible;
result.ControlPoint_units       = selectedControl_units;
result.SegmentTime_s            = selectedSegmentTime_s;
result.Planes                   = separatingPlanes;
result.TaggedPairs              = reshape([separatingPlanes.Active], size(separatingPlanes));
end

%% Section 4: Local Functions

function [refinedControl_units, refinedSegmentTime_s] = splitSelectedSegments( ...
        controlPoint_units, segmentTime_s, splitSegment)
    % Split each selected segment into two equal-time halves of the same
    % Bezier curve. Other segments retain their controls and durations.
    refinedSegmentCount  = numel(segmentTime_s) + nnz(splitSegment);
    refinedControl_units = zeros(refinedSegmentCount, size(controlPoint_units, 2), 2);
    refinedSegmentTime_s = zeros(refinedSegmentCount, 1);
    outputSegmentIndex   = 0;
    for segmentIndex = 1:numel(segmentTime_s)
        if splitSegment(segmentIndex)
            for halfIndex = 1:2
    outputSegmentIndex   = outputSegmentIndex + 1;
                refinedControl_units(outputSegmentIndex, :, :) = bmtpEngine.motion.restrictBezier( ...
                    squeeze(controlPoint_units(segmentIndex, :, :)), [(halfIndex - 1) / 2, halfIndex / 2]);
                refinedSegmentTime_s(outputSegmentIndex) = segmentTime_s(segmentIndex) / 2;
            end
        else
            outputSegmentIndex = outputSegmentIndex + 1;
            refinedControl_units(outputSegmentIndex, :, :) = controlPoint_units(segmentIndex, :, :);
            refinedSegmentTime_s(outputSegmentIndex) = segmentTime_s(segmentIndex);
        end
    end
end

function [separatingPlanes, allRequiredLinesAvailable, verifiedPairs, diagnostics, ...
        verifiedPairCount] = updateSeparatingLines( ...
        controlPoint_units, segmentTime_s, separatingPlanes, solverRequest, diagnostics, ...
        separationTarget_units, roundoffReserve_units, verifyExistingLines)
    % Check or rebuild the line for every applicable curve/obstacle pair.
    % Pairs whose time intervals do not overlap count as passed. Stop at the
    % first failed existing-line check or unavailable replacement line.
    verifiedPairs             = ~solverRequest.RegionActiveBySegment;
    allRequiredLinesAvailable = true;
    verifiedPairCount         = 0;
    segmentBoundaryTime_s     = solverRequest.InitialState.time_s + [0; cumsum(segmentTime_s)];
    for segmentIndex = 1:size(separatingPlanes, 1)
        for regionIndex = 1:size(separatingPlanes, 2)
            if ~solverRequest.RegionActiveBySegment(segmentIndex, regionIndex)
                continue
            end
            segmentControlPoint_units = squeeze(controlPoint_units(segmentIndex, :, :));
            overlapFractions          = [0, 1];
            overlapInterval_s         = segmentBoundaryTime_s(segmentIndex:segmentIndex + 1).';
            if isfield(solverRequest.Coverage, 'ActiveTimeInterval_s')
                obstacleActiveInterval_s = solverRequest.Coverage.ActiveTimeInterval_s(regionIndex, :);
                overlapInterval_s = [max(overlapInterval_s(1), obstacleActiveInterval_s(1)), ...
                    min(overlapInterval_s(2), obstacleActiveInterval_s(2))];
                if overlapInterval_s(2) <= overlapInterval_s(1)
                    % These intervals meet at most at one instant. This
                    % pair adds no time interval to constrain here; disable
                    % its line instead of constructing a zero-duration piece.
                    plane              = separatingPlanes(segmentIndex, regionIndex);
                    plane.Active       = false;
                    plane.Verified     = false;
                    plane.TimeFraction = [0, 1];
                    separatingPlanes(segmentIndex, regionIndex) = plane;
                    verifiedPairs(segmentIndex, regionIndex) = true;
                    continue
                end
                % Check only the curve portion during this obstacle interval.
                overlapFractions = max(0, min(1, ...
                    (overlapInterval_s - segmentBoundaryTime_s(segmentIndex)) / segmentTime_s(segmentIndex)));
                segmentControlPoint_units = ...
                    bmtpEngine.motion.restrictBezier(segmentControlPoint_units, overlapFractions);
            end
            obstacleVertices_units = bmtpEngine.separation.regionOnInterval(solverRequest.Regions_units{regionIndex}, ...
                solverRequest.Coverage, regionIndex, overlapInterval_s);
            if verifyExistingLines
                % Keep the existing direction, place the line against the
                % obstacle at both interval ends, then check the curve side.
                plane              = separatingPlanes(segmentIndex, regionIndex);
                lineNormal         = plane.Normal(1, :);
                plane.Offset_units = separationTarget_units - [min(obstacleVertices_units(:, :, 1) * lineNormal.'), ...
                    min(obstacleVertices_units(:, :, end) * lineNormal.')];
                plane.TimeFraction = overlapFractions;
                plane              = bmtpEngine.separation.verifySeparatingLine( ...
                    plane, segmentControlPoint_units, obstacleVertices_units, roundoffReserve_units, separationTarget_units);
                verifiedPairCount = verifiedPairCount + 1;
            else
                [plane, exitFlag, solverOutput] = bmtpEngine.separation.solveSeparatingLine( ...
                    segmentControlPoint_units, obstacleVertices_units, separationTarget_units, roundoffReserve_units);
                plane.TimeFraction         = overlapFractions;
                diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + ...
                    ~(isfield(solverOutput, 'IsAnalytic') && solverOutput.IsAnalytic);
                diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics( ...
                    diagnostics.ConicSolver, solverOutput);
            end
            separatingPlanes(segmentIndex, regionIndex) = plane;
            verifiedPairs(segmentIndex, regionIndex) = plane.Verified;
            if verifyExistingLines
                if ~plane.Verified
                    allRequiredLinesAvailable = false;
                    return
                end
            elseif exitFlag <= 0 || ~plane.Active
                allRequiredLinesAvailable = false;
                return;
            end
        end
    end
end
