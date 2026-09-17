function isVisible = checkVisibilitySegments(first_units, second_units, shape, edgeStart_units, edgeEnd_units)
%% Section 0: Header & Readme
% SYNTAX
%   isVisible = obstacleAvoidance.search.checkVisibilitySegments( ...
%       first_units, second_units, shape, edgeStart_units, edgeEnd_units)
%**************************************************************************
% PURPOSE
%   - Check straight segments against one proposal obstacle shape.
%**************************************************************************
% INPUTS
%   - first_units (N-by-2 numeric array)
%       First endpoint of each segment.
%   - second_units (N-by-2 numeric array)
%       Second endpoint of each segment.
%   - shape (scalar polyshape)
%       Spatial proposal obstacle.
%   - edgeStart_units (M-by-2 numeric array)
%       Start point of each ordered proposal-boundary edge.
%   - edgeEnd_units (M-by-2 numeric array)
%       End point of each ordered proposal-boundary edge.
%**************************************************************************
% OUTPUTS
%   - isVisible (N-by-1 logical vector)
%       True where a segment avoids the proposal shape and boundary. An empty
%       shape leaves every segment visible; malformed input throws an error.
%**************************************************************************
% UNITS
%   - All geometry is coordinate units.
%**************************************************************************

%% Section 1: Resolve Zero-Length Segments

isVisible = true(size(first_units, 1), 1);
if isempty(shape.Vertices)
    return
end
segment_units       = second_units - first_units;
segmentIsDegenerate = all(segment_units == 0, 2);

% A zero-length segment is a single point, so containment is the whole test.
if any(segmentIsDegenerate)
    isVisible(segmentIsDegenerate) = ~isinterior( ...
        shape, first_units(segmentIsDegenerate, 1), first_units(segmentIsDegenerate, 2));
end

%% Section 2: Reject Transverse Crossings And Collinear Overlaps

boundary_units   = edgeEnd_units - edgeStart_units;
offsetX_units    = edgeStart_units(:, 1).' - first_units(:, 1);
offsetY_units    = edgeStart_units(:, 2).' - first_units(:, 2);
denominator      = segment_units(:, 1) .* boundary_units(:, 2).' - segment_units(:, 2) .* boundary_units(:, 1).';
scale_units      = bmtpEngine.validation.createCoordinateTolerances(first_units, second_units, edgeStart_units, edgeEnd_units);
tolerance_units2 = 512 * eps(scale_units^2);
pairIsNonparallel = abs(denominator) > tolerance_units2;

% Parallel pairs keep a unit denominator so the shared division stays finite.
safeDenominator  = denominator;
safeDenominator(~pairIsNonparallel) = 1;

firstFraction            = (offsetX_units .* boundary_units(:, 2).' - ...
    offsetY_units .* boundary_units(:, 1).') ./ safeDenominator;
secondFraction           = (offsetX_units .* segment_units(:, 2) - ...
    offsetY_units .* segment_units(:, 1)) ./ safeDenominator;
firstFractionIntersects  = firstFraction >= -1e-12 & firstFraction <= 1 + 1e-12;
secondFractionIntersects = secondFraction >= -1e-12 & secondFraction <= 1 + 1e-12;
pairCrosses              = pairIsNonparallel & firstFractionIntersects & secondFractionIntersects;

crossProduct_units2 = offsetX_units .* segment_units(:, 2) - offsetY_units .* segment_units(:, 1);
pairIsCollinear     = ~pairIsNonparallel & abs(crossProduct_units2) <= tolerance_units2;
segmentScale_units2 = max(sum(segment_units.^2, 2), eps);
firstProjection     = (offsetX_units .* segment_units(:, 1) + ...
    offsetY_units .* segment_units(:, 2)) ./ segmentScale_units2;
nextOffsetX_units   = edgeEnd_units(:, 1).' - first_units(:, 1);
nextOffsetY_units   = edgeEnd_units(:, 2).' - first_units(:, 2);
secondProjection    = (nextOffsetX_units .* segment_units(:, 1) + ...
    nextOffsetY_units .* segment_units(:, 2)) ./ segmentScale_units2;
overlapStart        = max(min(firstProjection, secondProjection), 0);
overlapEnd          = min(max(firstProjection, secondProjection), 1) + 1e-12;
pairOverlaps        = pairIsCollinear & overlapStart <= overlapEnd;
isVisible(~segmentIsDegenerate) = ~any( ...
    pairCrosses(~segmentIsDegenerate, :) | pairOverlaps(~segmentIsDegenerate, :), 2);

%% Section 3: Test The Midpoints Of The Surviving Segments

% A segment that touches no boundary edge either stays outside the shape or
% lies wholly inside it, so its midpoint decides the remaining cases.
candidateIndices = find(isVisible & ~segmentIsDegenerate);
if isempty(candidateIndices)
    return
end
midpoint_units = (first_units(candidateIndices, :) + second_units(candidateIndices, :)) / 2;
isVisible(candidateIndices) = ~isinterior(shape, midpoint_units(:, 1), midpoint_units(:, 2));
end
