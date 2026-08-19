function inequality = seedCorridorInequality(polynomial, corridor)
%% Section 0: Header & Readme
% SYNTAX
%   inequality = azElInternal.seedCorridorInequality( ...
%       polynomial, corridor)
%**************************************************************************
% PURPOSE
%   - Enforce each complete trajectory segment outside its seed corridor.
%**************************************************************************
% INPUTS
%   - polynomial (scalar planner polynomial struct)
%       positionPower_deg contains segment-local ascending coefficients.
%   - corridor (structure array)
%       Records from azElInternal.buildSeedCorridor.
%**************************************************************************
% OUTPUTS
%   - inequality (numeric column vector)
%       Values are feasible when every value is less than or equal to zero.
%**************************************************************************
% UNITS
%   - Inequality values are degrees.
%**************************************************************************
%% Section 1: Convert Continuous Projection Bounds
if isempty(corridor)
    inequality = zeros(0, 1);
    return;
end
coefficientCount = size(polynomial.positionPower_deg, 3);
inequality = zeros(numel(corridor) * coefficientCount, 1);
writeIndex = 0;
for corridorIndex = 1:numel(corridor)
    record = corridor(corridorIndex);
    segmentIndex = record.SegmentIndex;
    azimuthPower_deg = reshape( ...
        polynomial.positionPower_deg(segmentIndex, 1, :), [], 1);
    elevationPower_deg = reshape( ...
        polynomial.positionPower_deg(segmentIndex, 2, :), [], 1);
    projectionPower_deg = ...
        record.Normal(1) * azimuthPower_deg + ...
        record.Normal(2) * elevationPower_deg;
    projectionBernstein_deg = azElInternal.powerToBernstein( ...
        projectionPower_deg);
    rows = writeIndex + (1:coefficientCount);
    inequality(rows) = record.BoundaryOffset_deg + ...
        record.Clearance_deg - projectionBernstein_deg;
    writeIndex = writeIndex + coefficientCount;
end
end
