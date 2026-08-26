function [segmentIndex, hullMap] = subintervalHullMap( ...
        tauStart, tauEnd, segmentCount, coefficientCount)
%% Section 0: Header & Readme
% SYNTAX
%   [segmentIndex, hullMap] = azElInternal.subintervalHullMap( ...
%       tauStart, tauEnd, segmentCount, coefficientCount)
%**************************************************************************
% PURPOSE
%   - Provide a deprecated compatibility alias to the neutral HS3 interval
%     restriction and Bernstein-hull map.
%**************************************************************************
% INPUTS
%   - tauStart, tauEnd (numeric vectors), normalized interval endpoints.
%   - segmentCount, coefficientCount (positive integer scalars).
%**************************************************************************
% OUTPUTS
%   - segmentIndex (numeric column), owning polynomial segments.
%   - hullMap (P-by-P-by-N numeric), exact restricted Bernstein maps.
%**************************************************************************
% UNITS
%   - Inputs and maps are dimensionless.
%**************************************************************************
[segmentIndex, hullMap] = hs3.subintervalHullMap( ...
    tauStart, tauEnd, segmentCount, coefficientCount);
end
