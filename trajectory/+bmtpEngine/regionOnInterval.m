function vertices_units = regionOnInterval(region_units, coverage, regionIndex, interval_s)
%% Section 0: Header & Readme
% SYNTAX: vertices = bmtpEngine.regionOnInterval(region, coverage, index, interval)
% PURPOSE: Restrict a source-derived affine convex cell to physical time.
% INPUTS: Start vertices, cell coverage, index, and two absolute times. An empty interval requests
%   the conservative spatial projection.
% OUTPUTS: N-by-2 static vertices or N-by-2-by-2 affine endpoint vertices.
% UNITS: Coordinate units and seconds.

%% Section 1: Evaluate The Source Cell At The Requested Endpoints
vertices_units = region_units;
if ~isfield(coverage,'EndRegions_units'), return; end
end_units = coverage.EndRegions_units{regionIndex};
if isequal(region_units,end_units), return; end
if isempty(interval_s)
    points_units = [region_units;end_units];
    hull = convhull(points_units(:,1),points_units(:,2));
    vertices_units = points_units(hull(1:end-1),:);
    return;
end
active_s = coverage.ActiveTimeInterval_s(regionIndex,:);
fraction = (interval_s-active_s(1))/diff(active_s);
if any(fraction < -64*eps | fraction > 1+64*eps)
    error('bmtpEngine:RegionTimeOutsideCell','Region restriction must remain within its active time interval.');
end
fraction = min(1,max(0,fraction));
delta_units = end_units-region_units;
first_units = region_units+fraction(1)*delta_units;
last_units = region_units+fraction(2)*delta_units;
% Preserve the authoritative stored endpoints exactly. Besides avoiding an
% unnecessary roundoff step, this makes full-cell geometry safe to cache.
if fraction(1)==0, first_units=region_units; end
if fraction(2)==1, last_units=end_units; end
vertices_units = cat(3,first_units,last_units);
end
