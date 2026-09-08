function corridor = createStaticCorridor(request,warmStart)
%% Section 0: Header & Readme
% SYNTAX: corridor = bmtpEngine.createStaticCorridor(request,warmStart)
% PURPOSE: Sweep exact convex source facets into the corridor facing a guide.
% INPUTS: Static request and connected monotone visibility guide.
% OUTPUTS: Affine lower/upper boundaries with closed coordinate intervals.
%   This chooses a motion topology; final source-pair validation is required.
% UNITS: Coordinates relative to the initial state, in coordinate units.

%% Section 1: Collect Facets In The Monotone Coordinate
[~,axisIndex] = max(warmStart.AxisMinimumTime_s);
axisOrder = [axisIndex,3-axisIndex];
origin_units = request.InitialState.position_units(axisOrder);
regionCount = numel(request.Regions_units);
first_units = cell(regionCount,1); last_units = first_units; owners = first_units;
for region = 1:regionCount
    vertices_units = request.Regions_units{region}(:,axisOrder)-origin_units;
    first_units{region} = vertices_units;
    last_units{region} = circshift(vertices_units,-1);
    owners{region} = repmat(region,size(vertices_units,1),1);
end
first_units = vertcat(first_units{:}); last_units = vertcat(last_units{:});
owners = vertcat(owners{:});
nonvertical = first_units(:,1)~=last_units(:,1);
first_units = first_units(nonvertical,:); last_units = last_units(nonvertical,:);
owners = owners(nonvertical);
slopes = (last_units(:,2)-first_units(:,2))./(last_units(:,1)-first_units(:,1));
intercepts_units = first_units(:,2)-slopes.*first_units(:,1);
minimum_units = min(first_units(:,1),last_units(:,1));
maximum_units = max(first_units(:,1),last_units(:,1));
guide_units = sortrows(warmStart.ClockGuide.Route_units(:,axisOrder)-origin_units,1);
% A vertical guide edge cannot be traversed by this prescribed clock.
available = all(diff(guide_units(:,1))>0);
corridor = struct('Available',available,'Intervals_units',zeros(0,4), ...
    'Slopes',slopes,'Intercepts_units',intercepts_units,'AxisIndex',axisIndex);
if ~available, return; end
events_units = unique([first_units(:,1);last_units(:,1);guide_units(:,1)]);
events_units = events_units(events_units>=guide_units(1,1) & events_units<=guide_units(end,1));

%% Section 2: Retain The Exact Envelopes Of All Facing Source Facets
pieces = cell(2*(numel(events_units)-1),1);
for slab = 1:numel(events_units)-1
    interval_units = events_units(slab:slab+1).'; midpoint_units = mean(interval_units);
    guideCoordinate_units = interp1(guide_units(:,1),guide_units(:,2),midpoint_units);
    active = find(minimum_units<midpoint_units & maximum_units>midpoint_units);
    values_units = slopes(active)*midpoint_units+intercepts_units(active);
    lower_units = accumarray(owners(active),values_units,[regionCount,1],@min,Inf);
    upper_units = accumarray(owners(active),values_units,[regionCount,1],@max,-Inf);
    present = isfinite(lower_units) & isfinite(upper_units);
    % A guide on a boundary chooses the facing side. Even if floating-point
    % visibility puts it slightly inside, retain that source region: the
    % optimizer must satisfy its original facet and full clearance.
    below = present & guideCoordinate_units>=(lower_units+upper_units)/2;
    above = present & ~below;
    belowEdges = active(below(owners(active)) & values_units==upper_units(owners(active)));
    aboveEdges = active(above(owners(active)) & values_units==lower_units(owners(active)));
    pieces{2*slab-1} = facetEnvelope(slopes,intercepts_units,belowEdges,interval_units,1);
    pieces{2*slab} = facetEnvelope(slopes,intercepts_units,aboveEdges,interval_units,-1);
end
pieces = sortrows(vertcat(pieces{:}),[4,3,1]);
merged = zeros(size(pieces)); count = 0;
for k = 1:size(pieces,1)
    if count>0 && all(pieces(k,3:4)==merged(count,3:4)) && pieces(k,1)==merged(count,2)
        merged(count,2) = pieces(k,2);
    else
        count = count+1; merged(count,:) = pieces(k,:);
    end
end
corridor.Intervals_units = merged(1:count,:);
end

%% Section 3: Compute A Piecewise Affine Envelope Without Sampling
function pieces = facetEnvelope(slopes,intercepts_units,indices,interval_units,side)
    pieces = zeros(0,4);
    if isempty(indices), return; end
    lines = sortrows([side*slopes(indices),side*intercepts_units(indices),indices],[1,2]);
    lines = lines([diff(lines(:,1))~=0;true],:);
    stack = zeros(size(lines)); starts_units = zeros(size(lines,1),1); count = 0;
    for k = 1:size(lines,1)
        crossing_units = -Inf;
        while count>0
            crossing_units = (stack(count,2)-lines(k,2))/(lines(k,1)-stack(count,1));
            if crossing_units>starts_units(count), break; end
            count = count-1;
        end
        if count==0, crossing_units = -Inf; end
        count = count+1; stack(count,:) = lines(k,:); starts_units(count) = crossing_units;
    end
    ends_units = [starts_units(2:count);Inf];
    for k = 1:count
        active_units = [max(interval_units(1),starts_units(k)),min(interval_units(2),ends_units(k))];
        if active_units(1)<active_units(2)
            pieces(end+1,:) = [active_units,stack(k,3),side]; %#ok<AGROW>
        end
    end
end
