function regions_deg = splitAzElRegions(azimuthBoundary_deg, ...
        elevationBoundary_deg)
%% Section 0: Header & Readme
% SYNTAX
%   regions_deg = splitAzElRegions( ...
%       azimuthBoundary_deg, elevationBoundary_deg)
%**************************************************************************
% PURPOSE
%   - Split paired nonfinite-separated canonical boundaries into polygons.
%**************************************************************************
% INPUTS
%   - azimuthBoundary_deg (numeric vector)
%   - elevationBoundary_deg (numeric vector)
%**************************************************************************
% OUTPUTS
%   - regions_deg (cell column)
%       Each cell is an N-by-2 polygon without a duplicated closing row.
%**************************************************************************
% UNITS
%   - Polygon coordinates are degrees.

azimuthBoundary_deg = double(azimuthBoundary_deg(:));
elevationBoundary_deg = double(elevationBoundary_deg(:));
if numel(azimuthBoundary_deg) ~= numel(elevationBoundary_deg)
    error("splitAzElRegions:BoundaryLengthMismatch", ...
        "Azimuth and elevation boundary lengths must match.");
end
if isempty(azimuthBoundary_deg)
    regions_deg = cell(0, 1);
    return;
end

separatorMask = ~isfinite(azimuthBoundary_deg) | ...
    ~isfinite(elevationBoundary_deg);
separatorIndices = [0; find(separatorMask); numel(azimuthBoundary_deg) + 1];
regions_deg = cell(numel(separatorIndices) - 1, 1);
writeIndex = 0;
for regionIndex = 1:(numel(separatorIndices) - 1)
    firstIndex = separatorIndices(regionIndex) + 1;
    lastIndex = separatorIndices(regionIndex + 1) - 1;
    if lastIndex < firstIndex
        continue;
    end
    vertices_deg = [azimuthBoundary_deg(firstIndex:lastIndex), ...
        elevationBoundary_deg(firstIndex:lastIndex)];
    if size(vertices_deg, 1) >= 2 && ...
            norm(vertices_deg(1, :) - vertices_deg(end, :)) <= ...
            64 * eps(max(1, max(abs(vertices_deg), [], "all")))
        vertices_deg(end, :) = [];
    end
    if size(vertices_deg, 1) >= 3
        writeIndex = writeIndex + 1;
        regions_deg{writeIndex} = vertices_deg;
    end
end
regions_deg = regions_deg(1:writeIndex);
end
