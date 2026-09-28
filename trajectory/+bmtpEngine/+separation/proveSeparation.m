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
%   - Place a candidate line between an obstacle and a motion curve, then
%     check its margins and physical clearance. The supplied side bounds
%     cover the full interval, so the result applies between sample times.
%**************************************************************************
% INPUTS
%   - minimumObstacleSide_units (numeric scalar or R-by-1 array)
%       Proven lower bound on dot(normal, obstacle position) + offset.
%   - maximumTrajectorySide_units (numeric scalar or R-by-1 array)
%       Proven upper bound on dot(normal, curve position) + offset.
%   - maximumNormalLength (numeric scalar or R-by-1 array)
%       Upper bound on each line normal's length over the interval.
%   - lineOffset_units (1-by-2 or R-by-2 numeric array)
%       Each line's offset at the interval start and end; both may shift
%       by the same amount without changing the line's normal.
%   - offsetRoundoffAllowance_units (numeric scalar or R-by-1 array)
%       Extra room sought inside each end of the allowed shift range.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Numerical margin required on the curve side of the line.
%   - separationTarget_units (nonnegative numeric scalar)
%       Minimum required obstacle-side value at the line.
%**************************************************************************
% OUTPUTS
%   - lineOffset_units (1-by-2 or R-by-2 numeric array)
%       Offsets after the same shift is applied at both interval ends.
%   - signedGap_units (numeric scalar or R-by-1 array)
%       Conservative lower bound on obstacle-side minus curve-side value.
%   - separationIsVerified (logical scalar or R-by-1 array)
%       True only where both side margins, gap, physical clearance, and
%       normal-length checks all pass.
%**************************************************************************
% UNITS
%   - Side values, offsets, gaps, reserves, and targets are coordinate
%     units. Normal lengths are dimensionless.
%**************************************************************************

%% Section 1: Find An Offset Shift That Preserves Both Required Gaps

% Adding the same shift to both offsets adds it to both side bounds.
% The obstacle needs shift >= target - obstacle side; the curve needs
% shift <= -reserve - curve side. For obstacle side 0.8, target 1,
% curve side -0.4, and reserve 0.1, the allowed shift is [0.2, 0.3].
minimumOffsetShift_units       = separationTarget_units - minimumObstacleSide_units;
maximumOffsetShift_units       = -roundoffReserve_units - maximumTrajectorySide_units;
minimumShiftWithRoundoff_units = minimumOffsetShift_units + offsetRoundoffAllowance_units;
maximumShiftWithRoundoff_units = maximumOffsetShift_units - offsetRoundoffAllowance_units;
offsetShift_units              = zeros(size(minimumOffsetShift_units));

% Leave extra room for rounding inside both ends of that allowed range.
% Keep zero shift if it fits; otherwise choose the allowed shift nearest 0.
shiftIsPossible     = minimumOffsetShift_units <= maximumOffsetShift_units;
shiftAllowsRoundoff = shiftIsPossible & minimumShiftWithRoundoff_units <= maximumShiftWithRoundoff_units;
offsetShift_units(shiftAllowsRoundoff) = min(max(0, minimumShiftWithRoundoff_units(shiftAllowsRoundoff)), ...
    maximumShiftWithRoundoff_units(shiftAllowsRoundoff));

% If the allowed range is too narrow for both rounding allowances, use its
% midpoint. The final checks still decide whether that shifted line passes.
shiftIntervalIsNarrow = shiftIsPossible & ~shiftAllowsRoundoff;
offsetShift_units(shiftIntervalIsNarrow) = 0.5 * ( ...
    minimumOffsetShift_units(shiftIntervalIsNarrow) + ...
    maximumOffsetShift_units(shiftIntervalIsNarrow));

lineOffset_units            = lineOffset_units + offsetShift_units;
minimumObstacleSide_units   = minimumObstacleSide_units + offsetShift_units;
maximumTrajectorySide_units = maximumTrajectorySide_units + offsetShift_units;

%% Section 2: Check The Shifted Line And Physical Clearance

% The signed gap is unchanged by a common offset shift. Subtract rounding
% reserves from that gap, then divide by the largest possible normal
% length to bound physical clearance. Allow a tiny normal-length roundoff
% above 1; realmin prevents division by zero.
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
