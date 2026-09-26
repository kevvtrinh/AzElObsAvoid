function [result, diagnostics] = refineTravel(solverRequest, retainedResult, ...
    retainedPreparedMotion, retainedProof, diagnostics, ...
    separationTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.optimization.refineTravel(solverRequest, ...
%       retainedResult, retainedPreparedMotion, retainedProof, diagnostics, ...
%       separationTarget_units, roundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Shorten a retained earliest-arrival control polygon at its fixed solver
%     clock. Keep the shorter motion only when its full prepared check passes
%     and its prepared arrival stays within the stated timing allowance.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked BMTP states, limits, obstacle coverage, and solver options.
%   - retainedResult (scalar struct)
%       Nine-field optimizer result with retained solver controls and lines.
%   - retainedPreparedMotion, retainedProof (scalar structs or [])
%       The retained motion's completed preparation and full check, passed
%       or not, or [] when the caller has not performed them.
%   - diagnostics (scalar struct)
%       Solver diagnostics accumulated by the arrival-time loop.
%   - separationTarget_units, roundoffReserve_units (numeric scalars)
%       Obstacle-side clearance and trajectory-side numerical reserve.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Selected optimizer result with PreparedMotion and Proof. An expected
%       rejected shortening keeps the retained result; invalid input throws.
%   - diagnostics (scalar struct)
%       Updated solve counts, selected pair counts, and refinement measurements.
%**************************************************************************
% UNITS
%   - Position and length are coordinate units; time is seconds.
%**************************************************************************

%% Section 1: Establish The Retained Prepared Arrival And Polygon Length

retainedControl_units = retainedResult.ControlPoint_units;
retainedSegmentTime_s = retainedResult.SegmentTime_s(:);
segmentCount          = size(retainedControl_units, 1);
assert(retainedResult.Success && numel(retainedSegmentTime_s) == segmentCount, ...
    'bmtpEngine:InvalidTravelRefinement', ...
    'Travel refinement requires a retained motion with one duration per segment.');

% A completed preparation and check of this exact motion, passed or not,
% is used as it is. Only when the caller performed neither is that work done
% here, once, and it is returned for the caller to reuse.
if isempty(retainedPreparedMotion) || isempty(retainedProof)
    [retainedPreparedMotion, retainedProof] = bmtpEngine.evaluateCandidate( ...
        solverRequest, retainedControl_units, retainedSegmentTime_s, ...
        roundoffReserve_units, separationTarget_units);
end

initialTime_s       = solverRequest.InitialState.time_s;
retainedArrival_s   = initialTime_s + sum(retainedSegmentTime_s);
if retainedPreparedMotion.Success
    retainedArrival_s = retainedPreparedMotion.FinalTime_s;
end
arrivalAllowance_s = 1e-6 * (retainedArrival_s - initialTime_s);
retainedLength_units = sum(vecnorm(diff(retainedControl_units, 1, 2), 2, 3), 'all');

result                = retainedResult;
result.PreparedMotion = retainedPreparedMotion;
result.Proof          = retainedProof;
selectedLength_units  = retainedLength_units;
selectedArrival_s     = retainedArrival_s;
separatingPlanes      = retainedResult.Planes;

if ~isfield(diagnostics, 'TrajectorySocpCount')
    diagnostics.TrajectorySocpCount = 0;
end
if ~isfield(diagnostics, 'PlaneSocpCount')
    diagnostics.PlaneSocpCount = 0;
end
diagnostics.TravelRefinementAttempted           = true;
diagnostics.TravelRefinementAccepted            = false;
diagnostics.TravelRefinementInitialLength_units = retainedLength_units;
diagnostics.TravelRefinementFinalLength_units   = retainedLength_units;
diagnostics.TravelRefinementInitialDuration_s   = retainedArrival_s - initialTime_s;
diagnostics.TravelRefinementFinalDuration_s     = retainedArrival_s - initialTime_s;
diagnostics.TravelRefinementExitFlag            = NaN;
diagnostics.TravelRefinementOptimizationConverged = false;

%% Section 2: Solve At The Retained Clock And Check Each Proposal

hasTimeScopedCoverage = isfield(solverRequest.Coverage, 'ActiveTimeInterval_s');
if solverRequest.UsesVariableClock
    formulationName             = "scaledClock";
    intrinsicVariationEnabled   = false;
else
    formulationName             = "physicalClock";
    intrinsicVariationEnabled   = true;
end
regionActiveBySegment = true(segmentCount, numel(solverRequest.Regions_units));
savedTrajectoryConstraints = struct();
refinementAttemptLimit     = 8;

for refinementIndex = 1:refinementAttemptLimit
    trajectoryStep = struct( ...
        'Formulation',               formulationName, ...
        'SegmentCount',              segmentCount, ...
        'Planes',                    separatingPlanes, ...
        'RoundoffReserve_units',     roundoffReserve_units, ...
        'MaximumMotionDuration_s',   [], ...
        'MinimumMotionDuration_s',   0, ...
        'SegmentRatio',              [], ...
        'SegmentTime_s',             retainedSegmentTime_s, ...
        'FixedClock',                true, ...
        'IntrinsicVariationEnabled', intrinsicVariationEnabled, ...
        'ConstraintBase',            savedTrajectoryConstraints);
    [refinedControl_units, refinedSegmentTime_s, refinementExitFlag, solverOutput, savedTrajectoryConstraints] = ...
        bmtpEngine.optimization.solveTrajectoryStep(solverRequest, trajectoryStep);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + solverOutput.SolveCount;
    diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics( ...
        diagnostics.ConicSolver, solverOutput);
    diagnostics.TravelRefinementExitFlag = refinementExitFlag;
    diagnostics.TravelRefinementOptimizationConverged = solverOutput.OptimizationConverged;
    % The full prepared check below is the authority on every proposal, so
    % no solver-side status short-circuits it here.
    if ~bmtpEngine.optimization.hasUsableConicIterate(refinedControl_units, refinementExitFlag)
        break
    end

    unresolvedPairs = false(segmentCount, numel(solverRequest.Regions_units));
    if ~hasTimeScopedCoverage
        % Sampling can reject an obvious static overlap before the full
        % polynomial and clearance checks. Moving coverage needs its clock.
        unresolvedPairs = bmtpEngine.separation.findSampledObstacleOverlaps( ...
            refinedControl_units, solverRequest.Regions_units, ...
            solverRequest.RegionMinimum_units, solverRequest.RegionMaximum_units, ...
            regionActiveBySegment);
    end
    if ~any(unresolvedPairs, 'all')
        [refinedPreparedMotion, refinedProof] = bmtpEngine.evaluateCandidate( ...
            solverRequest, refinedControl_units, refinedSegmentTime_s, ...
            roundoffReserve_units, separationTarget_units);

        refinedLength_units = sum(vecnorm(diff(refinedControl_units, 1, 2), 2, 3), 'all');
        preparedArrivalIsAllowed = refinedPreparedMotion.Success && ...
            refinedPreparedMotion.FinalTime_s <= retainedArrival_s + arrivalAllowance_s;
        polygonIsShorter = refinedLength_units < retainedLength_units;
        if refinedProof.Passed && preparedArrivalIsAllowed && polygonIsShorter
            result.ControlPoint_units = refinedControl_units;
            result.SegmentTime_s      = refinedSegmentTime_s;
            result.Planes             = separatingPlanes;
            result.TaggedPairs        = reshape([separatingPlanes.Active], size(separatingPlanes));
            result.PreparedMotion     = refinedPreparedMotion;
            result.Proof              = refinedProof;
            result.SolverMessage      = "A travel-shortened feasible iterate was retained.";
            selectedLength_units      = refinedLength_units;
            selectedArrival_s         = refinedPreparedMotion.FinalTime_s;
            diagnostics.TravelRefinementAccepted = true;
            break
        end

        % A separating line can fix an unresolved obstacle pair. It cannot
        % fix a workspace, motion-rate, or curve-join failure.
        nonCollisionChecksPassed = refinedPreparedMotion.Success && ...
            refinedProof.WorkspacePassed && refinedProof.DynamicsPassed && ...
            refinedProof.ContinuityPassed;
        if ~nonCollisionChecksPassed || refinedProof.Passed
            break
        end
        unverifiedPieces = ~reshape([refinedProof.Planes.Verified], size(refinedProof.Planes)) & ...
            refinedProof.RegionActiveBySegment;
        for pieceIndex = reshape(find(any(unverifiedPieces, 2)), 1, [])
            segmentIndex = refinedPreparedMotion.SourceSegmentIndex(pieceIndex);
            unresolvedPairs(segmentIndex, :) = unresolvedPairs(segmentIndex, :) | ...
                unverifiedPieces(pieceIndex, :);
        end
        if ~hasTimeScopedCoverage
            unresolvedPairs = unresolvedPairs & regionActiveBySegment;
        end
    end

    % A line around the retained curve can be added only for a pair that
    % lacks one. Time-scoped lines were built for every pair on the solver
    % clock; preparation can stretch that clock, so a pair can still appear
    % on the prepared clock. Nothing here can build such a line, so the
    % retained motion stands and the message says why.
    newPairs = unresolvedPairs & ~reshape([separatingPlanes.Active], size(separatingPlanes));
    if ~any(newPairs, 'all')
        break
    end
    if hasTimeScopedCoverage
        result.SolverMessage = retainedResult.SolverMessage + ...
            " Travel refinement found a time-scoped obstacle pair without a retained line.";
        break
    end
    if refinementIndex == refinementAttemptLimit
        break
    end

    lineUpdateFailed = false;
    for pairIndex = reshape(find(newPairs), 1, [])
        [segmentIndex, regionIndex] = ind2sub(size(newPairs), pairIndex);
        [plane, planeExitFlag, planeOutput] = bmtpEngine.separation.solveMaximumMarginLine( ...
            squeeze(retainedControl_units(segmentIndex, :, :)), ...
            solverRequest.Regions_units{regionIndex}, separationTarget_units, ...
            roundoffReserve_units, solverRequest.TrajectoryOptions);
        diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + ...
            ~(isfield(planeOutput, 'IsAnalytic') && planeOutput.IsAnalytic);
        diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics( ...
            diagnostics.ConicSolver, planeOutput);
        if (planeExitFlag <= 0 && planeExitFlag ~= -7) || ~plane.Active || ~plane.Verified
            lineUpdateFailed = true;
            break
        end
        separatingPlanes(segmentIndex, regionIndex) = plane;
    end
    if lineUpdateFailed
        break
    end
end

%% Section 3: Return The Selected Motion And Matching Proof

% Preparation may split solver segments. Keep its controls, times, and lines
% together when preparation accepted the motion and its full check passed. A
% proof can pass on a polynomial that preparation itself rejected, so the
% proof alone does not promote prepared controls.
if result.PreparedMotion.Success && result.Proof.Passed
    result.ControlPoint_units = result.PreparedMotion.ControlPoint_units;
    result.SegmentTime_s      = result.PreparedMotion.SegmentTime_s;
    result.Planes             = result.Proof.Planes;
    result.TaggedPairs        = result.Proof.RegionActiveBySegment;
    diagnostics.FinalCollisionPairCount = result.Proof.AllPairCount - result.Proof.VerifiedPairCount;
else
    diagnostics.FinalCollisionPairCount = 0;
end
diagnostics.ApplicablePairCount                = nnz(result.TaggedPairs);
diagnostics.TaggedPairCount                    = nnz(result.TaggedPairs);
diagnostics.SolverMessage                      = result.SolverMessage;
diagnostics.TravelRefinementFinalLength_units  = selectedLength_units;
diagnostics.TravelRefinementFinalDuration_s    = selectedArrival_s - initialTime_s;
end
