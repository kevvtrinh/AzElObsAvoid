function vertices_units = regionOnInterval(region_units, coverage, regionIndex, interval_s)
%% Section 0: Header & Readme
% SYNTAX
%   vertices_units = bmtpEngine.regionOnInterval(region_units, coverage, regionIndex, interval_s)
%**************************************************************************
% PURPOSE
%   - Restrict a source-derived affine convex cell to physical time.
%**************************************************************************
% INPUTS
%   - region_units (N-by-2 numeric array)
%       Convex cell vertices at the start of the cell's active interval.
%   - coverage (scalar struct)
%       Cell coverage carrying end vertices and active time intervals.
%   - regionIndex (positive integer scalar)
%       Index of the cell within the coverage.
%   - interval_s (empty or 1-by-2 numeric row)
%       Two absolute times. An empty interval requests the conservative
%       spatial projection over the whole active interval.
%**************************************************************************
% OUTPUTS
%   - vertices_units (N-by-2 or N-by-2-by-2 numeric array)
%       Static vertices, or affine endpoint vertices for a moving cell.
%       A request outside the cell's active time interval throws.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Return The Static Cell When No Motion Is Stored

vertices_units = region_units;
if ~isfield(coverage, 'EndRegions_units')
    return
end
endVertices_units = coverage.EndRegions_units{regionIndex};
if isequal(region_units, endVertices_units)
    return
end

%% Section 2: Project The Whole Sweep When No Interval Is Requested

if isempty(interval_s)
    points_units   = [region_units; endVertices_units];
    hullIndex      = convhull(points_units(:, 1), points_units(:, 2));
    vertices_units = points_units(hullIndex(1:end - 1), :);
    return
end

%% Section 3: Evaluate The Source Cell At The Requested Endpoints

activeInterval_s = coverage.ActiveTimeInterval_s(regionIndex, :);
fraction         = (interval_s - activeInterval_s(1)) / diff(activeInterval_s);
intervalLeavesActiveWindow = any(fraction < -64 * eps | fraction > 1 + 64 * eps);
if intervalLeavesActiveWindow
    error('bmtpEngine:RegionTimeOutsideCell', 'Region restriction must remain within its active time interval.');
end
fraction    = min(1, max(0, fraction));
delta_units = endVertices_units - region_units;
first_units = region_units + fraction(1) * delta_units;
last_units  = region_units + fraction(2) * delta_units;
% Preserve the authoritative stored endpoints exactly. Besides avoiding an
% unnecessary roundoff step, this makes full-cell geometry safe to cache.
if fraction(1) == 0
    first_units = region_units;
end
if fraction(2) == 1
    last_units = endVertices_units;
end
vertices_units = cat(3, first_units, last_units);
end
