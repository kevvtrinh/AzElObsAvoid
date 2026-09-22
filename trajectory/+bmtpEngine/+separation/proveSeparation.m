function [lineOffset_units, signedGap_units, separationIsVerified] = proveSeparation( ...
    minimumObstacleSide_units, maximumTrajectorySide_units, maximumNormalLength, ...
    lineOffset_units, offsetRoundoffAllowance_units, roundoffReserve_units, separationTarget_units)
%% Section 0: Header & Readme
% SYNTAX
%   [lineOffset_units, signedGap_units, separationIsVerified] = ...
%       bmtpEngine.separation.proveSeparation(minimumObstacleSide_units, ...
%       maximumTrajectorySide_units, maximumNormalLength, lineOffset_units, ...
%       offsetRoundoffAllowance_units, roundoffReserve_units, separationTarget_units)
%**************************************************************************
% PURPOSE
%   - Shift a separating line when possible, then require the obstacle and
%     curve to remain on opposite sides with their required clearance.
%     The caller supplies bounds that cover the complete time interval.
%**************************************************************************
% INPUTS
%   - minimumObstacleSide_units (numeric scalar or R-by-1 array)
%       Lower bound on normal x obstacle position + offset for each line.
%   - maximumTrajectorySide_units (numeric scalar or R-by-1 array)
%       Upper bound on normal x curve position + offset for each line.
%   - maximumNormalLength (numeric scalar or R-by-1 array)
%       Largest length of each line's normal vector over the interval.
%   - lineOffset_units (1-by-2 or R-by-2 numeric array)
%       Each line's offset at the interval start and end.
%   - offsetRoundoffAllowance_units (numeric scalar or R-by-1 array)
%       Extra room sought at both ends of an allowed offset-shift interval.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Required numerical gap on the curve side.
%   - separationTarget_units (nonnegative numeric scalar)
%       Required line-side value for the obstacle.
%**************************************************************************
% OUTPUTS
%   - lineOffset_units (1-by-2 or R-by-2 numeric array)
%       Offsets after the common shift at both interval ends.
%   - signedGap_units (numeric scalar or R-by-1 array)
%       Lower bound on obstacle-side value minus curve-side value.
%   - separationIsVerified (logical scalar or R-by-1 array)
%       True only where every final clearance and normal-length check passes.
%**************************************************************************
% UNITS
%   - Sides, offsets, gaps, reserves, and targets are coordinate units.
%**************************************************************************

%% Section 1: Find An Offset Shift That Preserves Both Required Gaps

% Adding the same shift to both offsets also adds it to both side bounds.
% The obstacle requires shift >= target - obstacleSide.
% The curve requires shift <= -reserve - curveSide. Both must hold.
minimumOffsetShift_units       = separationTarget_units - minimumObstacleSide_units;
maximumOffsetShift_units       = -roundoffReserve_units - maximumTrajectorySide_units;
minimumShiftWithRoundoff_units = minimumOffsetShift_units + offsetRoundoffAllowance_units;
maximumShiftWithRoundoff_units = maximumOffsetShift_units - offsetRoundoffAllowance_units;
offsetShift_units              = zeros(size(minimumOffsetShift_units));

% Leave extra room for rounding at both ends when possible. Keep zero
% shift if it fits; otherwise use the nearest shift in that reduced interval.
shiftIsPossible     = minimumOffsetShift_units <= maximumOffsetShift_units;
shiftAllowsRoundoff = shiftIsPossible & minimumShiftWithRoundoff_units <= maximumShiftWithRoundoff_units;
offsetShift_units(shiftAllowsRoundoff) = min(max(0, minimumShiftWithRoundoff_units(shiftAllowsRoundoff)), ...
    maximumShiftWithRoundoff_units(shiftAllowsRoundoff));

% If an allowed interval exists but is too narrow for both rounding
% allowances, use its midpoint. The final checks below still must pass.
shiftIntervalIsNarrow = shiftIsPossible & ~shiftAllowsRoundoff;
offsetShift_units(shiftIntervalIsNarrow) = 0.5 * ( ...
    minimumOffsetShift_units(shiftIntervalIsNarrow) + ...
    maximumOffsetShift_units(shiftIntervalIsNarrow));

lineOffset_units            = lineOffset_units + offsetShift_units;
minimumObstacleSide_units   = minimumObstacleSide_units + offsetShift_units;
maximumTrajectorySide_units = maximumTrajectorySide_units + offsetShift_units;

%% Section 2: Check The Shifted Line And Physical Clearance

% Divide the line-side gap by the largest normal length to bound physical
% clearance. Account for numerical reserves before comparing that bound
% with the required clearance. Normal length may exceed 1 only by the given
% rounding allowance; realmin prevents division by zero.
signedGap_units         = minimumObstacleSide_units - maximumTrajectorySide_units;
normalLengthLimit       = 1 + 2 ^ 20 * eps;
requiredClearance_units = (separationTarget_units - roundoffReserve_units) / normalLengthLimit;
provenClearance_units   = (signedGap_units - 2 * roundoffReserve_units) ./ ...
    max(maximumNormalLength, realmin);

obstacleSideIsValid    = minimumObstacleSide_units >= separationTarget_units;
trajectorySideIsValid  = maximumTrajectorySide_units <= -roundoffReserve_units;
signedGapIsValid       = signedGap_units >= separationTarget_units + roundoffReserve_units;
provenClearanceIsValid = provenClearance_units >= requiredClearance_units;
normalLengthIsValid    = maximumNormalLength <= normalLengthLimit;

separationIsVerified = obstacleSideIsValid & trajectorySideIsValid & signedGapIsValid & ...
    provenClearanceIsValid & normalLengthIsValid;
end
