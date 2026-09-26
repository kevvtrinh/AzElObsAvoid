function [preparedMotion, motionCheck, savedPairChecks] = evaluateCandidate( ...
    solverRequest, controlPoint_units, segmentTime_s, roundoffReserve_units, ...
    separationTarget_units, suppliedPowerCoefficients_units, checkMode, savedPairChecks)
%% Section 0: Header & Readme
% SYNTAX
%   [preparedMotion, motionCheck] = bmtpEngine.evaluateCandidate( ...
%       solverRequest, controlPoint_units, segmentTime_s, ...
%       roundoffReserve_units, separationTarget_units)
%   [preparedMotion, motionCheck, savedPairChecks] = bmtpEngine.evaluateCandidate( ...
%       solverRequest, controlPoint_units, segmentTime_s, ...
%       roundoffReserve_units, separationTarget_units, ...
%       suppliedPowerCoefficients_units, checkMode, savedPairChecks)
%**************************************************************************
% PURPOSE
%   - Build one motion from selected controls and times, then check that
%     exact motion. Keep any proof subdivision with the motion it checked.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked states, limits, options, and prepared obstacles.
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Selected Bezier controls.
%   - segmentTime_s (S-by-1 numeric vector)
%       Selected segment durations in seconds.
%   - roundoffReserve_units, separationTarget_units (numeric scalars)
%       Numerical reserve and required obstacle-side clearance.
%   - suppliedPowerCoefficients_units (numeric array or [], optional)
%       Exact polynomial coefficients supplied by a direct chord.
%   - checkMode (string scalar, optional)
%       "refineSeparation" checks the complete motion and splits only to
%       prove clearance; "completeOnce" checks without splitting; and
%       "firstUnverified" stops at the first unproved obstacle pair.
%   - savedPairChecks (scalar struct or [], optional)
%       Reusable checks of unchanged pieces and matching geometry.
%**************************************************************************
% OUTPUTS
%   - preparedMotion (scalar struct)
%       Exact polynomial, segment times, and preparation outcome.
%   - motionCheck (scalar struct)
%       Check of that motion, or Passed = false if preparation failed.
%   - savedPairChecks (scalar struct or [])
%       Reusable pair checks for this exact motion.
%**************************************************************************
% UNITS
%   - Position and clearance are coordinate units; time is seconds.
%**************************************************************************

%% Section 1: Build The Motion From The Selected Candidate
if nargin < 6
    suppliedPowerCoefficients_units = [];
end
if nargin < 7
    checkMode = "refineSeparation";
end
if nargin < 8
    savedPairChecks = [];
end
if ~any(checkMode == ["refineSeparation", "completeOnce", "firstUnverified"])
    error('bmtpEngine:InvalidCheckMode', 'Unknown BMTP candidate check mode.');
end

preparedMotion = bmtpEngine.motion.createMotion( ...
    solverRequest, controlPoint_units, segmentTime_s, suppliedPowerCoefficients_units);
motionCheck = struct('Passed', false);
if ~preparedMotion.Success
    return
end

%% Section 2: Check This Motion And Keep Its Matching Proof
if checkMode == "refineSeparation"
    [preparedMotion, motionCheck, savedPairChecks] = ...
        bmtpEngine.validation.checkMotionWithSubdivision( ...
        solverRequest, preparedMotion, roundoffReserve_units, ...
        separationTarget_units, savedPairChecks);
else
    stopOnFirstUnverified = checkMode == "firstUnverified";
    [motionCheck, savedPairChecks] = bmtpEngine.validation.checkFinalMotion( ...
        solverRequest, preparedMotion, roundoffReserve_units, ...
        separationTarget_units, savedPairChecks, stopOnFirstUnverified);
end
end
