function [offset_units, signedGap_units, verified] = proveSeparation( ...
        minimumObstacleSide_units, maximumTrajectorySide_units, maximumNormalNorm, ...
        offset_units, roundoff_units, reserve_units, target_units)
%% Section 0: Header & Readme
% SYNTAX
%   [offset_units, signedGap_units, verified] = ...
%       bmtpEngine.separation.proveSeparation(minimumObstacleSide_units, ...
%       maximumTrajectorySide_units, maximumNormalNorm, offset_units, ...
%       roundoff_units, reserve_units, target_units)
%**************************************************************************
% PURPOSE
%   - Apply the shared separating-line acceptance decision.
%**************************************************************************
% INPUTS
%   - minimumObstacleSide_units (numeric scalar or R-by-1 array)
%       Exact obstacle-side Bernstein bounds for each plane.
%   - maximumTrajectorySide_units (numeric scalar or R-by-1 array)
%       Exact trajectory-side Bernstein bounds for each plane.
%   - maximumNormalNorm (numeric scalar or R-by-1 array)
%       Maximum normal norm for each plane.
%   - offset_units (1-by-2 or R-by-2 numeric array)
%       Endpoint offsets for each plane.
%   - roundoff_units (numeric scalar or R-by-1 array)
%       Admissible correction reserve for each plane.
%   - reserve_units (nonnegative numeric scalar)
%       Required trajectory-side reserve.
%   - target_units (nonnegative numeric scalar)
%       Required obstacle-side target.
%**************************************************************************
% OUTPUTS
%   - offset_units (1-by-2 or R-by-2 numeric array)
%       Endpoint offsets shifted into the admissible correction interval.
%   - signedGap_units (numeric scalar or R-by-1 array)
%       Proven obstacle-minus-trajectory gap after that shift.
%   - verified (logical scalar or R-by-1 array)
%       True only where every acceptance inequality holds together.
%**************************************************************************
% UNITS
%   - Sides, offsets, gaps, reserves, and targets are coordinate units.
%**************************************************************************

%% Section 1: Shift The Plane Into The Admissible Correction Interval
minimumCorrection_units = target_units - minimumObstacleSide_units;
maximumCorrection_units = -reserve_units - maximumTrajectorySide_units;
robustMinimum_units      = minimumCorrection_units + roundoff_units;
robustMaximum_units      = maximumCorrection_units - roundoff_units;
correction_units         = zeros(size(minimumCorrection_units));

correctionIsPossible = minimumCorrection_units <= maximumCorrection_units;
correctionIsRobust   = correctionIsPossible & robustMinimum_units <= robustMaximum_units;
correction_units(correctionIsRobust) = min(max(0, robustMinimum_units(correctionIsRobust)), ...
    robustMaximum_units(correctionIsRobust));

% A correction interval narrower than roundoff still exists; take its midpoint.
correctionIntervalIsNarrow = correctionIsPossible & ~correctionIsRobust;
correction_units(correctionIntervalIsNarrow) = 0.5 * ( ...
    minimumCorrection_units(correctionIntervalIsNarrow) + ...
    maximumCorrection_units(correctionIntervalIsNarrow));

offset_units                = offset_units + correction_units;
minimumObstacleSide_units   = minimumObstacleSide_units + correction_units;
maximumTrajectorySide_units = maximumTrajectorySide_units + correction_units;

%% Section 2: Require Every Acceptance Inequality Together
signedGap_units          = minimumObstacleSide_units - maximumTrajectorySide_units;
normalNormLimit          = 1 + 2 ^ 20 * eps;
clearanceTarget_units    = (target_units - reserve_units) / normalNormLimit;
provenClearance_units = (signedGap_units - 2 * reserve_units) ./ max(maximumNormalNorm, realmin);

obstacleSideIsValid       = minimumObstacleSide_units >= target_units;
trajectorySideIsValid     = maximumTrajectorySide_units <= -reserve_units;
signedGapIsValid          = signedGap_units >= target_units + reserve_units;
provenClearanceIsValid = provenClearance_units >= clearanceTarget_units;
normalNormIsValid         = maximumNormalNorm <= normalNormLimit;

verified = obstacleSideIsValid & trajectorySideIsValid & signedGapIsValid & ...
    provenClearanceIsValid & normalNormIsValid;
end
