function corridor = applyAzElHs3CorridorRepair( ...
        corridor, meshTau, repairTau, repairBuffer_deg, ...
        baseClearance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   corridor = azElInternal.applyAzElHs3CorridorRepair( ...
%       corridor, meshTau, repairTau, repairBuffer_deg, ...
%       baseClearance_deg)
%**************************************************************************
% PURPOSE
%   - Add measured collision-repair tube clearance to affected segments.
%**************************************************************************
% INPUTS
%   - corridor (scalar struct)
%       HS-3 corridor with PointTau and PointClearance_deg fields.
%   - meshTau (N-by-1 numeric vector)
%       Strictly increasing normalized knot times.
%   - repairTau (M-by-1 numeric vector)
%       Normalized collision times.
%   - repairBuffer_deg (M-by-1 numeric vector)
%       Required extra curve-tube clearance at each collision time.
%   - baseClearance_deg (nonnegative numeric scalar)
%       Required clearance before the repair buffer is added.
%**************************************************************************
% OUTPUTS
%   - corridor (scalar struct)
%       Input corridor with updated point clearances.
%**************************************************************************
% UNITS
%   - Clearance is degrees. meshTau and repairTau are dimensionless.
%**************************************************************************

%% Section 1: Apply Segment Repair Clearances

pointClearance_deg = baseClearance_deg * ...
    ones(numel(corridor.PointTau), 1);
tauTolerance = 64 * eps(max(1, max(abs(meshTau))));
for repairIndex = 1:numel(repairTau)
    segmentIndex = find(meshTau <= ...
        repairTau(repairIndex) + tauTolerance, 1, "last");
    segmentIndex = min(numel(meshTau) - 1, max(1, segmentIndex));
    pointIsInSegment = corridor.PointTau >= ...
        meshTau(segmentIndex) - tauTolerance & ...
        corridor.PointTau <= meshTau(segmentIndex + 1) + tauTolerance;
    requiredClearance_deg = baseClearance_deg + ...
        repairBuffer_deg(repairIndex);
    pointClearance_deg(pointIsInSegment) = max( ...
        pointClearance_deg(pointIsInSegment), requiredClearance_deg);
end
corridor.PointClearance_deg = pointClearance_deg;
end
