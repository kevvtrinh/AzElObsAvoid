function [pointTau, associationTau] = azElHs3CorridorTau(meshTau)
%% Section 0: Header & Readme
% SYNTAX
%   [pointTau, associationTau] = ...
%       azElInternal.azElHs3CorridorTau(meshTau)
%**************************************************************************
% PURPOSE
%   - Return the standard HS-3 corridor constraint and association times.
%**************************************************************************
% INPUTS
%   - meshTau (N-by-1 numeric vector)
%       Strictly increasing normalized knot times.
%**************************************************************************
% OUTPUTS
%   - pointTau (5*(N-1)-by-1 numeric vector)
%       Segment endpoints, quarter points, and midpoints.
%   - associationTau (5*(N-1)-by-1 numeric vector)
%       Interior times used to select stable endpoint separators.
%**************************************************************************
% UNITS
%   - All values are dimensionless normalized times.
%**************************************************************************

%% Section 1: Expand Each Mesh Segment

localTau = [0, 0.25, 0.5, 0.75, 1];
associationLocalTau = [0.125, 0.25, 0.5, 0.75, 0.875];
segmentStartTau = meshTau(1:end - 1);
segmentWidth = diff(meshTau);
pointMatrix = segmentStartTau + segmentWidth .* localTau;
associationMatrix = segmentStartTau + ...
    segmentWidth .* associationLocalTau;
pointTau = reshape(pointMatrix.', [], 1);
associationTau = reshape(associationMatrix.', [], 1);
end
