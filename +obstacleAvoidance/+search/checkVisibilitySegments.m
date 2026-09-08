function isVisible = checkVisibilitySegments(first_units, second_units, shape, edgeStart_units, edgeEnd_units)
%% Section 0: Header & Readme
% SYNTAX
%   isVisible = obstacleAvoidance.search.checkVisibilitySegments( ...
%       first_units, second_units, shape, edgeStart_units, edgeEnd_units)
% PURPOSE
%   - Check straight spatial segments against one proposal obstacle shape.
%   - Provide one shared visibility rule for graph and route cleanup stages.
% INPUTS
%   - first_units, second_units (N-by-2 finite numeric matrices)
%       Paired segment endpoints in [x y] order.
%   - shape (scalar polyshape)
%       Spatial proposal obstacle used for route guidance.
%   - edgeStart_units, edgeEnd_units (M-by-2 numeric matrices)
%       Ordered proposal-boundary edge endpoints.
% OUTPUTS
%   - isVisible (N-by-1 logical vector)
%       True where the segment avoids the proposal shape and its boundary.
% UNITS
%   - All geometry is coordinate units.

%% Section 1: Check Exact Visibility In Bounded Pair Blocks
isVisible = true(size(first_units,1),1);
if isempty(shape.Vertices), return; end
middle_units = (first_units+second_units)/2;
isVisible = ~isinterior(shape,middle_units(:,1),middle_units(:,2));
if isempty(edgeStart_units), return; end
scale_units = bmtpEngine.createCoordinateTolerances(first_units,second_units,edgeStart_units,edgeEnd_units);
% Preserve one request-wide roundoff bound. Recomputing it on smaller batches
% would change decisions near contact, especially at large coordinate offsets.
tolerance_units2 = 512*eps(scale_units^2);
% Every temporary matrix has at most 65,536 segment-edge pairs. Once rejected,
% a segment needs no remaining edge tests; surviving segments check every edge.
edgeBatch = min(512,size(edgeStart_units,1));
segmentBatch = max(1,floor(65536/edgeBatch));
for firstIndex = 1:segmentBatch:size(first_units,1)
    pending = (firstIndex:min(size(first_units,1),firstIndex+segmentBatch-1)).';
    pending = pending(isVisible(pending));
    for edgeIndex = 1:edgeBatch:size(edgeStart_units,1)
        if isempty(pending), break; end
        edges = edgeIndex:min(size(edgeStart_units,1),edgeIndex+edgeBatch-1);
        firstEdge_units = edgeStart_units(edges,:); secondEdge_units = edgeEnd_units(edges,:);
        segmentStart_units = first_units(pending,:); segment_units = second_units(pending,:)-segmentStart_units;
        boundary_units = secondEdge_units-firstEdge_units;
        offsetX_units = firstEdge_units(:,1).'-segmentStart_units(:,1);
        offsetY_units = firstEdge_units(:,2).'-segmentStart_units(:,2);
        denominator = segment_units(:,1).*boundary_units(:,2).'-segment_units(:,2).*boundary_units(:,1).';
        isNonparallel = abs(denominator)>tolerance_units2;
        safeDenominator = denominator; safeDenominator(~isNonparallel)=1;
        firstFraction = (offsetX_units.*boundary_units(:,2).'-offsetY_units.*boundary_units(:,1).')./safeDenominator;
        secondFraction = (offsetX_units.*segment_units(:,2)-offsetY_units.*segment_units(:,1))./safeDenominator;
        crosses = isNonparallel & firstFraction>=-1e-12 & firstFraction<=1+1e-12 & secondFraction>=-1e-12 & secondFraction<=1+1e-12;
        isCollinear = ~isNonparallel & abs(offsetX_units.*segment_units(:,2)-offsetY_units.*segment_units(:,1))<=tolerance_units2;
        segmentScale_units2 = max(sum(segment_units.^2,2),eps);
        firstProjection = (offsetX_units.*segment_units(:,1)+offsetY_units.*segment_units(:,2))./segmentScale_units2;
        nextOffsetX_units = secondEdge_units(:,1).'-segmentStart_units(:,1); nextOffsetY_units = secondEdge_units(:,2).'-segmentStart_units(:,2);
        secondProjection = (nextOffsetX_units.*segment_units(:,1)+nextOffsetY_units.*segment_units(:,2))./segmentScale_units2;
        overlaps = isCollinear & max(min(firstProjection,secondProjection),0)<=min(max(firstProjection,secondProjection),1)+1e-12;
        rejected = any(crosses|overlaps,2);
        isVisible(pending(rejected)) = false;
        pending = pending(~rejected);
    end
end
end
