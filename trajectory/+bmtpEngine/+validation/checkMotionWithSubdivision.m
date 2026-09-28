function [preparedMotion, motionCheck, savedPairChecks, unverifiedPairsBySegment] = checkMotionWithSubdivision( ...
    solverRequest, preparedMotion, roundoffReserve_units, separationTarget_units, savedPairChecks)
%% Section 0: Header & Readme
% SYNTAX
%   [preparedMotion, motionCheck, savedPairChecks] = ...
%       bmtpEngine.validation.checkMotionWithSubdivision(solverRequest, preparedMotion, ...
%       roundoffReserve_units, separationTarget_units)
%   [preparedMotion, motionCheck, savedPairChecks] = ...
%       bmtpEngine.validation.checkMotionWithSubdivision(solverRequest, preparedMotion, ...
%       roundoffReserve_units, separationTarget_units, savedPairChecks)
%   [preparedMotion, motionCheck, savedPairChecks, unverifiedPairsBySegment] = ...
%       bmtpEngine.validation.checkMotionWithSubdivision(solverRequest, preparedMotion, ...
%       roundoffReserve_units, separationTarget_units, savedPairChecks)
%**************************************************************************
% PURPOSE
%   - Check a prepared motion and split pieces whose obstacle separation
%     remains unresolved. Preserve its polynomial, timing, and endpoint states.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked motion limits, obstacle regions, coverage, and tolerances.
%   - preparedMotion (scalar struct)
%       Motion from createMotion, including its complete coefficients
%       and SourceSegmentIndex linking each piece to an original segment.
%   - roundoffReserve_units, separationTarget_units (numeric scalars)
%       Numerical reserve and required obstacle-side clearance.
%   - savedPairChecks (scalar struct or empty, optional; default [])
%       Previous checks. Unchanged pieces can reuse matching separation proofs.
%**************************************************************************
% OUTPUTS
%   - preparedMotion (scalar struct)
%       The same physical motion, possibly represented by smaller pieces.
%   - motionCheck, savedPairChecks (scalar structs)
%       Final checks and reusable pair results for exactly those pieces.
%       Passed remains false if any required check cannot be established.
%   - unverifiedPairsBySegment (S-by-R logical array)
%       Unproved obstacle pairs mapped from checked pieces back to the S
%       original input segments and R regions.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Check The Prepared Motion

if nargin < 5
    savedPairChecks = [];
end
[motionCheck, savedPairChecks] = bmtpEngine.validation.checkFinalMotion( ...
    solverRequest, preparedMotion, roundoffReserve_units, separationTarget_units, savedPairChecks);

%% Section 2: Split Only Pieces With Unresolved Separation

% One line may fail to separate a curved segment from an obstacle even when
% the curve is clear. Smaller pieces can each admit their own line. Splitting
% cannot repair workspace, motion-rate, or continuity failures.
for refinementIndex = 1:10
    if ~preparedMotion.Success || motionCheck.Passed || ~motionCheck.WorkspacePassed || ...
            ~motionCheck.DynamicsPassed || ~motionCheck.ContinuityPassed
        break
    end
    unverifiedPairs = ~reshape([motionCheck.Planes.Verified], size(motionCheck.Planes)) & ...
        motionCheck.RegionActiveBySegment;
    splitSegment = any(unverifiedPairs, 2);
    if ~any(splitSegment)
        break
    end

    [~, preparedMotion.SegmentTime_s, preparedMotion.GivenPower_units, parentSegmentIndex] = ...
        bmtpEngine.motion.subdivideMotion(preparedMotion.ControlPoint_units, ...
        preparedMotion.SegmentTime_s, preparedMotion.GivenPower_units, splitSegment);
    preparedMotion.SourceSegmentIndex = preparedMotion.SourceSegmentIndex(parentSegmentIndex);
    % Both separation and exported motion use the split polynomial. Converting
    % it to controls does not impose endpoint states or correct joins.
    preparedMotion.ControlPoint_units = bmtpEngine.motion.powerToBernstein(preparedMotion.GivenPower_units);

    % Check each split polynomial's exact rate peaks. Splitting keeps the
    % physical motion, its assigned durations, and its final time unchanged.
    requiredTime_s = bmtpEngine.motion.findRequiredPolynomialTime( ...
        preparedMotion.GivenPower_units, preparedMotion.SegmentTime_s, solverRequest.Limits);
    preparedMotion.RequiredSegmentTime_s = requiredTime_s;
    preparedMotion.MotionProof = struct( ...
        'Passed',           all(preparedMotion.SegmentTime_s >= requiredTime_s), ...
        'SegmentTime_s',    preparedMotion.SegmentTime_s, ...
        'MaximumViolation', max([0; requiredTime_s - preparedMotion.SegmentTime_s]));
    [motionCheck, savedPairChecks] = bmtpEngine.validation.checkFinalMotion( ...
        solverRequest, preparedMotion, roundoffReserve_units, separationTarget_units, savedPairChecks);
end

%% Section 3: Map Unproved Pieces Back To Their Original Segments

% Splitting keeps each piece's source segment. If several pieces from the
% same segment lack a proof, keep every unresolved obstacle pair.
sourceSegmentCount = max(preparedMotion.SourceSegmentIndex);
regionCount = numel(solverRequest.Regions_units);
unverifiedPairsBySegment = false(sourceSegmentCount, regionCount);
unverifiedPieces = ~reshape([motionCheck.Planes.Verified], size(motionCheck.Planes)) & ...
    motionCheck.RegionActiveBySegment;
for pieceIndex = reshape(find(any(unverifiedPieces, 2)), 1, [])
    sourceSegmentIndex = preparedMotion.SourceSegmentIndex(pieceIndex);
    unverifiedPairsBySegment(sourceSegmentIndex, :) = ...
        unverifiedPairsBySegment(sourceSegmentIndex, :) | unverifiedPieces(pieceIndex, :);
end
end
