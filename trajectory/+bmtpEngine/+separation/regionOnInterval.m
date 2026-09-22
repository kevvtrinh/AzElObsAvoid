function intervalVertices_units = regionOnInterval( ...
    startVertices_units, regionMotionData, regionIndex, requestedInterval_s)
%% Section 0: Header & Readme
% SYNTAX
%   intervalVertices_units = bmtpEngine.separation.regionOnInterval( ...
%       startVertices_units, regionMotionData, regionIndex, requestedInterval_s)
%**************************************************************************
% PURPOSE
%   - Return a convex obstacle region at the requested start/end times,
%     using the stored linear motion of its vertices.
%**************************************************************************
% INPUTS
%   - startVertices_units (N-by-2 numeric array)
%       Region vertices at the start of its stored motion interval.
%   - regionMotionData (scalar struct)
%       EndRegions_units and ActiveTimeInterval_s when motion is present.
%   - regionIndex (positive integer scalar)
%       Region to select from the stored motion arrays.
%   - requestedInterval_s (empty or 1-by-2 numeric row)
%       Two absolute times. Empty requests one polygon enclosing everywhere
%       the region travels during its complete stored interval.
%**************************************************************************
% OUTPUTS
%   - intervalVertices_units (M-by-2 or N-by-2-by-2 numeric array)
%       Static/enclosing polygon vertices, or moving-region vertices at the
%       two requested times. An out-of-range moving-region request throws.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Return Unchanged Vertices For A Static Region

intervalVertices_units = startVertices_units;
if ~isfield(regionMotionData, 'EndRegions_units')
    return
end
endVertices_units = regionMotionData.EndRegions_units{regionIndex};
if isequal(startVertices_units, endVertices_units)
    return
end

%% Section 2: Enclose The Whole Motion When No Times Are Requested

% Linear vertex paths stay inside the convex hull of their start/end points.
% This enclosure can include extra area; it is a whole-motion obstacle bound.

if isempty(requestedInterval_s)
    endpointVertices_units = [startVertices_units; endVertices_units];
    hullVertexIndices      = convhull(endpointVertices_units(:, 1), endpointVertices_units(:, 2));
    intervalVertices_units = endpointVertices_units(hullVertexIndices(1:end - 1), :);
    return
end

%% Section 3: Interpolate Vertices At The Requested Times

% fraction = (requested time - start time) / stored duration. Allow only
% a small rounding error beyond [0 1], then clamp that error to the endpoint.

activeInterval_s  = regionMotionData.ActiveTimeInterval_s(regionIndex, :);
intervalFractions = (requestedInterval_s - activeInterval_s(1)) / diff(activeInterval_s);
intervalLeavesActiveWindow = any(intervalFractions < -64 * eps | intervalFractions > 1 + 64 * eps);
if intervalLeavesActiveWindow
    error('bmtpEngine:RegionTimeOutsideCell', 'Region restriction must remain within its active time interval.');
end
intervalFractions           = min(1, max(0, intervalFractions));
vertexDisplacement_units    = endVertices_units - startVertices_units;
intervalStartVertices_units = startVertices_units + intervalFractions(1) * vertexDisplacement_units;
intervalEndVertices_units   = startVertices_units + intervalFractions(2) * vertexDisplacement_units;

% At the stored start/end times, return the original vertices exactly.
% Recalculating start + displacement could otherwise add rounding error
% and prevent a later geometry comparison from recognizing the same region.
if intervalFractions(1) == 0
    intervalStartVertices_units = startVertices_units;
end
if intervalFractions(2) == 1
    intervalEndVertices_units = endVertices_units;
end
intervalVertices_units = cat(3, intervalStartVertices_units, intervalEndVertices_units);
end
