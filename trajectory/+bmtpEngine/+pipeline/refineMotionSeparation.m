function [preparedMotion, motionCheck, savedPairChecks] = refineMotionSeparation( ...
    solverRequest, preparedMotion, roundoffReserve_units, separationTarget_units, savedPairChecks)
%% Section 0: Header & Readme
% SYNTAX
%   [preparedMotion, motionCheck, savedPairChecks] = ...
%       bmtpEngine.pipeline.refineMotionSeparation(solverRequest, preparedMotion, ...
%       roundoffReserve_units, separationTarget_units)
%   [preparedMotion, motionCheck, savedPairChecks] = ...
%       bmtpEngine.pipeline.refineMotionSeparation(solverRequest, preparedMotion, ...
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
%       Motion from prepareFinalMotion, including its complete coefficients
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

    % Smaller control polygons can tighten duration bounds. Record those
    % bounds without changing any assigned duration or the saved final time.
    requiredTime_s = bmtpEngine.motion.findRequiredSegmentTime( ...
        preparedMotion.ControlPoint_units, solverRequest.Limits);
    preparedMotion.RequiredSegmentTime_s = requiredTime_s;
    preparedMotion.MotionProof = struct( ...
        'Passed',           all(preparedMotion.SegmentTime_s >= requiredTime_s), ...
        'SegmentTime_s',    preparedMotion.SegmentTime_s, ...
        'MaximumViolation', max([0; requiredTime_s - preparedMotion.SegmentTime_s]));
    [motionCheck, savedPairChecks] = bmtpEngine.validation.checkFinalMotion( ...
        solverRequest, preparedMotion, roundoffReserve_units, separationTarget_units, savedPairChecks);
end
end
