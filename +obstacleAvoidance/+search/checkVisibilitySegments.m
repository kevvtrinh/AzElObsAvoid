function isVisible = checkVisibilitySegments(first_units, second_units, shape, edgeStart_units, edgeEnd_units)
%% Section 0: Header & Readme
% SYNTAX
%   isVisible = obstacleAvoidance.search.checkVisibilitySegments( ...
%       first_units, second_units, shape, edgeStart_units, edgeEnd_units)
%
% PURPOSE
%   - Check straight spatial segments against one proposal obstacle shape.
%   - Provide one shared visibility rule for graph and route cleanup stages.
%
% INPUTS
%   - first_units, second_units (N-by-2 finite numeric matrices)
%       Paired segment endpoints in [x y] order.
%   - shape (scalar polyshape)
%       Spatial proposal obstacle used for route guidance.
%   - edgeStart_units, edgeEnd_units (M-by-2 numeric matrices)
%       Ordered proposal-boundary edge endpoints.
%
% OUTPUTS
%   - isVisible (N-by-1 logical vector)
%       True where the segment avoids the proposal shape and its boundary.
%
% UNITS
%   - All geometry is coordinate units.
%

%% Section 1: Reject Interior And Boundary Intersections

% Reject interior crossings, boundary crossings, and collinear overlaps.

isVisible = true(size(first_units, 1), 1);
if isempty(shape.Vertices)
    return;
end
middle_units          = (first_units + second_units) / 2;
isVisible           = ~isinterior(shape, middle_units(:, 1), middle_units(:, 2));
segment_units         = second_units - first_units;
boundary_units        = edgeEnd_units - edgeStart_units;
offsetX_units   = edgeStart_units(:, 1).' - first_units(:, 1);
offsetY_units = edgeStart_units(:, 2).' - first_units(:, 2);
denominator         = segment_units(:, 1) .* boundary_units(:, 2).' - segment_units(:, 2) .* boundary_units(:, 1).';
scale_units           = bmtpEngine.createCoordinateTolerances(first_units, second_units, edgeStart_units, edgeEnd_units);
tolerance_units2      = 512 * eps(scale_units^2);
isNonparallel       = abs(denominator) > tolerance_units2;
safeDenominator     = denominator;
safeDenominator(~isNonparallel) = 1;
firstFraction           = (offsetX_units .* boundary_units(:, 2).' - offsetY_units .* boundary_units(:, 1).') ./ safeDenominator;
secondFraction          = (offsetX_units .* segment_units(:, 2) - offsetY_units .* segment_units(:, 1)) ./ safeDenominator;
crosses                 = isNonparallel & firstFraction >= -1e-12 & firstFraction <= 1 + 1e-12 & secondFraction >= -1e-12 & secondFraction <= 1 + 1e-12;
isCollinear             = ~isNonparallel & abs(offsetX_units .* segment_units(:, 2) - offsetY_units .* segment_units(:, 1)) <= tolerance_units2;
segmentScale_units2       = max(sum(segment_units.^2, 2), eps);
firstProjection         = (offsetX_units .* segment_units(:, 1) + offsetY_units .* segment_units(:, 2)) ./ segmentScale_units2;
nextOffsetX_units   = edgeEnd_units(:, 1).' - first_units(:, 1);
nextOffsetY_units = edgeEnd_units(:, 2).' - first_units(:, 2);
secondProjection        = (nextOffsetX_units .* segment_units(:, 1) + nextOffsetY_units .* segment_units(:, 2)) ./ segmentScale_units2;
overlaps                = isCollinear & max(min(firstProjection, secondProjection), 0) <= min(max(firstProjection, secondProjection), 1) + 1e-12;
isVisible               = isVisible & ~any(crosses | overlaps, 2);
end
