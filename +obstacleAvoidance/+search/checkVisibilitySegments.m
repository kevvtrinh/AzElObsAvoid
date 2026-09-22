function isVisible = checkVisibilitySegments( ...
    segmentStart_units, segmentEnd_units, obstacleShape, boundaryStart_units, boundaryEnd_units)
%% Section 0: Header & Readme
% SYNTAX
%   isVisible = obstacleAvoidance.search.checkVisibilitySegments( ...
%       segmentStart_units, segmentEnd_units, obstacleShape, boundaryStart_units, boundaryEnd_units)
%**************************************************************************
% PURPOSE
%   - Check whether each straight segment stays outside one obstacle
%     polygon. Touching or following its boundary blocks visibility.
%**************************************************************************
% INPUTS
%   - segmentStart_units (N-by-2 numeric array)
%       First endpoint of each segment.
%   - segmentEnd_units (N-by-2 numeric array)
%       Second endpoint of each segment.
%   - obstacleShape (scalar polyshape)
%       Obstacle polygon at the time being checked.
%   - boundaryStart_units (M-by-2 numeric array)
%       Start point of each edge along the obstacle boundary.
%   - boundaryEnd_units (M-by-2 numeric array)
%       End point of each edge along the obstacle boundary.
%**************************************************************************
% OUTPUTS
%   - isVisible (N-by-1 logical vector)
%       True where a segment avoids the obstacle and its boundary. An empty
%       shape leaves every segment visible; malformed input throws an error.
%**************************************************************************
% UNITS
%   - All geometry is coordinate units.
%**************************************************************************

%% Section 1: Check Segments That Are Single Points

isVisible = true(size(segmentStart_units, 1), 1);
if isempty(obstacleShape.Vertices)
    return
end
segmentVector_units = segmentEnd_units - segmentStart_units;
segmentIsPoint      = all(segmentVector_units == 0, 2);

% Identical endpoints represent one point. Its location inside or outside
% the polygon decides the result; there is no line to intersect.
if any(segmentIsPoint)
    isVisible(segmentIsPoint) = ~isinterior( ...
        obstacleShape, segmentStart_units(segmentIsPoint, 1), segmentStart_units(segmentIsPoint, 2));
end

%% Section 2: Check Where Each Segment Meets The Obstacle Boundary

% Compare every segment with every boundary edge: one segment per row and
% one boundary edge per column. A cross product near zero means parallel.
boundaryVector_units         = boundaryEnd_units - boundaryStart_units;
boundaryStartOffsetX_units   = boundaryStart_units(:, 1).' - segmentStart_units(:, 1);
boundaryStartOffsetY_units   = boundaryStart_units(:, 2).' - segmentStart_units(:, 2);
directionCrossProduct_units2 = segmentVector_units(:, 1) .* boundaryVector_units(:, 2).' - ...
    segmentVector_units(:, 2) .* boundaryVector_units(:, 1).';
coordinateScale_units = bmtpEngine.validation.createCoordinateTolerances( ...
    segmentStart_units, segmentEnd_units, boundaryStart_units, boundaryEnd_units);
crossProductTolerance_units2 = 512 * eps(coordinateScale_units^2);
pairIsNonparallel            = abs(directionCrossProduct_units2) > crossProductTolerance_units2;

% Use 1 for parallel pairs during division to avoid division by zero.
% The nonparallel flag excludes these placeholder values from this test.
safeCrossProduct_units2 = directionCrossProduct_units2;
safeCrossProduct_units2(~pairIsNonparallel) = 1;

% Both intersection fractions must be in [0 1] for the finite lines to meet.
% For a segment from x = 0 to x = 10, fraction 0.5 means x = 5. The small
% allowance at 0 and 1 keeps rounding from missing endpoint contact.
segmentIntersectionFraction = (boundaryStartOffsetX_units .* boundaryVector_units(:, 2).' - ...
    boundaryStartOffsetY_units .* boundaryVector_units(:, 1).') ./ safeCrossProduct_units2;
boundaryIntersectionFraction = (boundaryStartOffsetX_units .* segmentVector_units(:, 2) - ...
    boundaryStartOffsetY_units .* segmentVector_units(:, 1)) ./ safeCrossProduct_units2;
intersectionIsOnSegment  = segmentIntersectionFraction >= -1e-12 & segmentIntersectionFraction <= 1 + 1e-12;
intersectionIsOnBoundary = boundaryIntersectionFraction >= -1e-12 & boundaryIntersectionFraction <= 1 + 1e-12;
pairIntersects           = pairIsNonparallel & intersectionIsOnSegment & intersectionIsOnBoundary;

% Parallel lines may still overlap if they lie on the same straight line.
% Project the boundary endpoints onto the segment, then check whether their
% fraction range overlaps [0 1].
lineOffsetCrossProduct_units2 = boundaryStartOffsetX_units .* segmentVector_units(:, 2) - ...
    boundaryStartOffsetY_units .* segmentVector_units(:, 1);
pairSharesLine = ~pairIsNonparallel & abs(lineOffsetCrossProduct_units2) <= crossProductTolerance_units2;
safeSegmentLengthSquared_units2 = max(sum(segmentVector_units.^2, 2), eps);
boundaryStartFraction = (boundaryStartOffsetX_units .* segmentVector_units(:, 1) + ...
    boundaryStartOffsetY_units .* segmentVector_units(:, 2)) ./ safeSegmentLengthSquared_units2;
boundaryEndOffsetX_units = boundaryEnd_units(:, 1).' - segmentStart_units(:, 1);
boundaryEndOffsetY_units = boundaryEnd_units(:, 2).' - segmentStart_units(:, 2);
boundaryEndFraction      = (boundaryEndOffsetX_units .* segmentVector_units(:, 1) + ...
    boundaryEndOffsetY_units .* segmentVector_units(:, 2)) ./ safeSegmentLengthSquared_units2;
overlapStartFraction = max(min(boundaryStartFraction, boundaryEndFraction), 0);
overlapEndFraction   = min(max(boundaryStartFraction, boundaryEndFraction), 1) + 1e-12;
pairOverlaps         = pairSharesLine & overlapStartFraction <= overlapEndFraction;
isVisible(~segmentIsPoint) = ~any( ...
    pairIntersects(~segmentIsPoint, :) | pairOverlaps(~segmentIsPoint, :), 2);

%% Section 3: Test The Midpoints Of The Surviving Segments

% With no boundary contact, a segment stays entirely inside or entirely
% outside the polygon. Its midpoint decides which of those cases applies.
segmentsToCheck = find(isVisible & ~segmentIsPoint);
if isempty(segmentsToCheck)
    return
end
midpoint_units = ...
    (segmentStart_units(segmentsToCheck, :) + segmentEnd_units(segmentsToCheck, :)) / 2;
isVisible(segmentsToCheck) = ~isinterior(obstacleShape, midpoint_units(:, 1), midpoint_units(:, 2));
end
