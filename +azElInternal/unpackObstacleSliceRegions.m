function regions = unpackObstacleSliceRegions(obstacle, sampleIndex)
%% Section 0: Header & Readme
% SYNTAX
%   regions = azElInternal.unpackObstacleSliceRegions( ...
%       obstacle, sampleIndex)
%**************************************************************************
% PURPOSE
%   - Recover independent polygon rings from one packed obstacle slice.
%   - Keep search and visualization consumers on the same packed-geometry
%     interpretation so displayed and planned boundaries cannot diverge.
%**************************************************************************
% INPUTS
%   - obstacle (scalar packed-obstacle struct)
%       SliceOffsets, AzimuthDeg, and ElevationDeg follow the packed field
%       schema returned by buildAzElTimeObstacleField.
%   - sampleIndex (positive integer scalar)
%       One-based source slice to decode.
%**************************************************************************
% OUTPUTS
%   - regions (N-by-1 cell array)
%       Each cell contains one M-by-2 [azimuth elevation] polygon ring.
%**************************************************************************
% UNITS
%   - Polygon coordinates are degrees.
%**************************************************************************

firstVertexIndex = double(obstacle.SliceOffsets(sampleIndex));
finalVertexIndex = double(obstacle.SliceOffsets(sampleIndex + 1) - 1);
if finalVertexIndex < firstVertexIndex
    regions = cell(0, 1);
    return;
end

azimuth_deg = double( ...
    obstacle.AzimuthDeg(firstVertexIndex:finalVertexIndex));
elevation_deg = double( ...
    obstacle.ElevationDeg(firstVertexIndex:finalVertexIndex));
isFiniteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);

% Loop equivalent: scan vertices in order, start a ring when a finite row
% follows a separator, and close it when the next separator is reached.
% Padding with false makes the vectorized transitions handle both ends.
regionTransitions = diff([false; isFiniteVertex; false]);
regionStartIndex = find(regionTransitions == 1);
regionStopIndex = find(regionTransitions == -1) - 1;

regions = cell(numel(regionStartIndex), 1);
for regionIndex = 1:numel(regionStartIndex)
    vertexIndex = ...
        regionStartIndex(regionIndex):regionStopIndex(regionIndex);
    regions{regionIndex} = [ ...
        azimuth_deg(vertexIndex), elevation_deg(vertexIndex)];
end
end
