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
[separatingPlanes, solverRequest.RegionActiveBySegment, allRequiredLinesAvailable, ~, diagnostics] = ...
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
fixedSegmentTime_s          = warmStart.SegmentTime_s(:);

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
        % Solve on the same segment times used to build these obstacle lines.
        trajectoryStep = struct( ...
            'Formulation',               "physicalClock", ...
            'SegmentCount',              segmentCount, ...
            'Planes',                    separatingPlanes, ...
            'RoundoffReserve_units',     roundoffReserve_units, ...
            'MaximumMotionDuration_s',   [], ...
            'MinimumMotionDuration_s',   0, ...
            'SegmentRatio',              [], ...
            'SegmentTime_s',             fixedSegmentTime_s, ...
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
            solverMessage                      = "Trajectory SOCP failed: " + string(solverOutput.message);
            diagnostics.LastTrajectoryExitFlag = exitFlag;
            failureStage                       = solverOutput.FailureStage;
            failureKind                        = solverOutput.FailureKind;
            alternativeGuideEligible           = solverOutput.AlternativeGuideEligible;
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
            [updatedSeparatingPlanes, solverRequest.RegionActiveBySegment, ~, verifiedPairs, ...
                diagnostics, verifiedPairCount] = ...
                updateSeparatingLines(trialControl_units, trialSegmentTime_s, separatingPlanes, solverRequest, ...
                diagnostics, separationTarget_units, roundoffReserve_units, true);
            diagnostics.ExistingPlanePairVerificationCount = ...
                diagnostics.ExistingPlanePairVerificationCount + verifiedPairCount;
        end
        allRequiredLinesAvailable = all(verifiedPairs, 'all');
        if ~allRequiredLinesAvailable
            % At least one existing line failed; rebuild the complete set.
            [updatedSeparatingPlanes, solverRequest.RegionActiveBySegment, ...
                allRequiredLinesAvailable, verifiedPairs, diagnostics] = ...
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
                [refinedControl_units, refinedSegmentTime_s] = bmtpEngine.motion.subdivideMotion( ...
                    trialControl_units, trialSegmentTime_s, [], splitSegment);
                segmentSplitCount = segmentSplitCount + 1;
                segmentCount      = numel(refinedSegmentTime_s);
                diagnostics.OptimizerSpanCount = segmentCount;
                fixedSegmentTime_s = refinedSegmentTime_s(:);
                includeJerkVariation            = false;
                savedTrajectoryConstraints      = struct();
                separatingPlanes                = repmat(emptyPlane, segmentCount, regionCount);
                [separatingPlanes, solverRequest.RegionActiveBySegment, ...
                    allRequiredLinesAvailable, ~, diagnostics] = ...
                    updateSeparatingLines(refinedControl_units, ...
                    refinedSegmentTime_s, separatingPlanes, solverRequest, diagnostics, ...
                    separationTarget_units, roundoffReserve_units, false);
                diagnostics.ApplicablePairCount = nnz(solverRequest.RegionActiveBySegment);
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
result = bmtpEngine.optimization.createOptimizationResult(solverMessage, failureStage, failureKind, ...
    alternativeGuideEligible, selectedControl_units, selectedSegmentTime_s, separatingPlanes, ...
    reshape([separatingPlanes.Active], size(separatingPlanes)));
end

%% Section 4: Local Functions

function [separatingPlanes, regionActiveBySegment, allRequiredLinesAvailable, verifiedPairs, diagnostics, ...
        checkedPairCount] = updateSeparatingLines( ...
        controlPoint_units, segmentTime_s, separatingPlanes, solverRequest, diagnostics, ...
        separationTarget_units, roundoffReserve_units, verifyExistingLines)
    % Check or rebuild the line for every applicable curve/obstacle pair,
    % stopping at the first failed existing-line check or unavailable
    % replacement line. Pairs whose time intervals do not overlap count as
    % passed. Count any numerical line solves toward this solve's totals.
    lineUpdate = struct( ...
        'Planes',             separatingPlanes, ...
        'VerifyExisting',     verifyExistingLines, ...
        'StopAtFirstFailure', true);
    [separatingPlanes, regionActiveBySegment, ~, lineReport] = ...
        bmtpEngine.separation.createTimeScopedPlanes( ...
        controlPoint_units, segmentTime_s, solverRequest, separationTarget_units, ...
        roundoffReserve_units, lineUpdate);
    verifiedPairs    = lineReport.VerifiedPairs;
    checkedPairCount = lineReport.CheckedPairCount;
    if verifyExistingLines
        allRequiredLinesAvailable = all(verifiedPairs, 'all');
    else
        allRequiredLinesAvailable = lineReport.UnavailablePairCount == 0;
    end
    diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + lineReport.SocpCount;
    diagnostics.ConicSolver    = bmtpEngine.optimization.accumulateConicDiagnostics( ...
        diagnostics.ConicSolver, struct('SolveCount', lineReport.SocpCount, ...
        'TotalTime_s', lineReport.SolverTime_s));
end
