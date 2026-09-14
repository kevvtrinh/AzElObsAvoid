function [offset_units, signedGap_units, verified] = certifySeparation( ...
        minimumObstacleSide_units, maximumTrajectorySide_units, maximumNormalNorm, ...
        offset_units, roundoff_units, reserve_units, target_units)
%% Section 0: Header & Readme
% SYNTAX: [offset_units,signedGap_units,verified] = bmtpEngine.certifySeparation(
%   minimumObstacleSide_units, maximumTrajectorySide_units, maximumNormalNorm, offset_units,
%   roundoff_units, reserve_units, target_units)
% PURPOSE: Own the one separating-line acceptance decision. Shift the plane into the interval both
%   sides admit, then require obstacle side, trajectory side, signed gap, certified clearance and
%   normal norm together. The scalar and batched verifiers share this so neither can drift.
% INPUTS: minimumObstacleSide_units, maximumTrajectorySide_units, maximumNormalNorm (R-by-1 or
%   scalar) Exact Bernstein bounds already evaluated for each plane. offset_units (R-by-2 or 1-by-2)
%   Endpoint offsets to correct. roundoff_units (R-by-1 or scalar) 16*eps of the plane coordinate
%   scale; it is read only where a correction is admissible. reserve_units, target_units
%   (nonnegative scalars) Trajectory-side reserve and obstacle-side target.
% OUTPUTS: offset_units (shape of the supplied offsets) Corrected endpoint offsets.
%   signedGap_units (R-by-1 or scalar) Certified separation. verified (logical, same shape).
% UNITS: Sides, offsets, gaps, reserves and targets are coordinate units; normals are dimensionless.

%% Section 1: Shift The Plane Into The Admissible Correction Interval
minimumCorrection_units = target_units-minimumObstacleSide_units;
maximumCorrection_units = -reserve_units-maximumTrajectorySide_units;
robustMinimum_units = minimumCorrection_units+roundoff_units;
robustMaximum_units = maximumCorrection_units-roundoff_units;
correction_units = zeros(size(minimumCorrection_units));
possible = minimumCorrection_units<=maximumCorrection_units;
robust = possible & robustMinimum_units<=robustMaximum_units;
correction_units(robust) = min(max(0,robustMinimum_units(robust)),robustMaximum_units(robust));
% A correction interval narrower than roundoff still exists; take its midpoint.
correction_units(possible & ~robust) = 0.5*(minimumCorrection_units(possible & ~robust)+ ...
    maximumCorrection_units(possible & ~robust));
offset_units = offset_units+correction_units;
minimumObstacleSide_units = minimumObstacleSide_units+correction_units;
maximumTrajectorySide_units = maximumTrajectorySide_units+correction_units;

%% Section 2: Require Every Acceptance Inequality Together
signedGap_units = minimumObstacleSide_units-maximumTrajectorySide_units;
normalNormLimit = 1+2^20*eps;
clearanceTarget_units = (target_units-reserve_units)/normalNormLimit;
certifiedClearance_units = (signedGap_units-2*reserve_units)./max(maximumNormalNorm,realmin);
verified = minimumObstacleSide_units>=target_units & maximumTrajectorySide_units<=-reserve_units & ...
    signedGap_units>=target_units+reserve_units & certifiedClearance_units>=clearanceTarget_units & ...
    maximumNormalNorm<=normalNormLimit;
end
