function intervalVertices_units = regionOnInterval( ...
    startVertices_units, regionMotionData, regionIndex, requestedInterval_s)
%% Section 0: Header & Readme
% SYNTAX
%   intervalVertices_units = bmtpEngine.separation.regionOnInterval( ...
%       startVertices_units, regionMotionData, regionIndex, requestedInterval_s)
%**************************************************************************
% PURPOSE
%   - Return a prepared obstacle at two requested times. Moving vertices
%     travel linearly from their stored start to end positions. With no
%     requested times, return one polygon enclosing that whole movement.
%     A static obstacle keeps its original vertices.
%**************************************************************************
% INPUTS
%   - startVertices_units (N-by-2 numeric array)
%       Polygon vertices at the start of the stored motion interval.
%   - regionMotionData (scalar struct)
%       EndRegions_units holds matching end vertices and
%       ActiveTimeInterval_s holds each moving region's [start, end] time.
%       Without EndRegions_units, the region is static.
%   - regionIndex (positive integer scalar)
%       One-based region number in those stored arrays.
%   - requestedInterval_s (empty or 1-by-2 numeric row)
%       Two absolute times inside the region's active interval. Empty asks
%       for one polygon enclosing all positions over the stored interval.
%**************************************************************************
% OUTPUTS
%   - intervalVertices_units (M-by-2 or N-by-2-by-2 numeric array)
%       For a static region or empty request, one polygon's [x, y] rows.
%       For a moving region at two times, N vertices at each time in the
%       third dimension. A moving request outside its active times throws.
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

% Each vertex travels on a straight line between its start and end.
% The convex hull of all endpoints therefore encloses every intermediate
% polygon. It may include extra area that the region never occupies.

if isempty(requestedInterval_s)
    endpointVertices_units = [startVertices_units; endVertices_units];
    hullVertexIndices      = convhull(endpointVertices_units(:, 1), endpointVertices_units(:, 2));
    intervalVertices_units = endpointVertices_units(hullVertexIndices(1:end - 1), :);
    return
end

%% Section 3: Interpolate Vertices At The Requested Times

% fraction = (requested time - active start) / active duration. For motion
% from 2 to 6 s, a request at 3 s is one-quarter of the way to the end
% vertices. Allow only a tiny rounding error outside [0, 1], then use
% the nearest endpoint for that error.

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
