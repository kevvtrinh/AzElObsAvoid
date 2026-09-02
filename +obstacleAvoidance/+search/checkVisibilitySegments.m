function isVisible = checkVisibilitySegments( ...
        first_deg, second_deg, shape, edgeStart_deg, edgeEnd_deg)
%% Section 0: Header & Readme
% SYNTAX
%   isVisible = obstacleAvoidance.search.checkVisibilitySegments( ...
%       first_deg, second_deg, shape, edgeStart_deg, edgeEnd_deg)
%**************************************************************************
% PURPOSE
%   - Check straight spatial segments against one proposal obstacle shape.
%   - Provide one shared visibility rule for graph and route cleanup stages.
%**************************************************************************
% INPUTS
%   - first_deg, second_deg (N-by-2 finite numeric matrices)
%       Paired segment endpoints in [azimuth elevation] order.
%   - shape (scalar polyshape)
%       Spatial proposal obstacle used for route guidance.
%   - edgeStart_deg, edgeEnd_deg (M-by-2 numeric matrices)
%       Ordered proposal-boundary edge endpoints.
%**************************************************************************
% OUTPUTS
%   - isVisible (N-by-1 logical vector)
%       True where the segment avoids the proposal shape and its boundary.
%**************************************************************************
% UNITS
%   - All geometry is degrees.
%**************************************************************************

%% Section 1: Reject Interior And Boundary Intersections

% Midpoint containment catches segments through occupied interiors. The
% vectorized boundary test also rejects crossings and collinear overlap so
% graph construction and route shortening cannot disagree about visibility.

isVisible = true(size(first_deg, 1), 1);
if isempty(shape.Vertices)
    return;
end
middle_deg = (first_deg + second_deg) / 2;
isVisible = ~isinterior(shape, middle_deg(:, 1), middle_deg(:, 2));
segment_deg = second_deg - first_deg;
boundary_deg = edgeEnd_deg - edgeStart_deg;
offsetAzimuth_deg = edgeStart_deg(:, 1).' - first_deg(:, 1);
offsetElevation_deg = edgeStart_deg(:, 2).' - first_deg(:, 2);
denominator = segment_deg(:, 1) .* boundary_deg(:, 2).' - ...
    segment_deg(:, 2) .* boundary_deg(:, 1).';
scale_deg = bmtpEngine.createCoordinateTolerances( ...
    first_deg, second_deg, edgeStart_deg, edgeEnd_deg);
tolerance_deg2 = 512 * eps(scale_deg^2);
isNonparallel = abs(denominator) > tolerance_deg2;
safeDenominator = denominator;
safeDenominator(~isNonparallel) = 1;
firstFraction = (offsetAzimuth_deg .* boundary_deg(:, 2).' - ...
    offsetElevation_deg .* boundary_deg(:, 1).') ./ safeDenominator;
secondFraction = (offsetAzimuth_deg .* segment_deg(:, 2) - ...
    offsetElevation_deg .* segment_deg(:, 1)) ./ safeDenominator;
crosses = isNonparallel & firstFraction >= -1e-12 & ...
    firstFraction <= 1 + 1e-12 & secondFraction >= -1e-12 & ...
    secondFraction <= 1 + 1e-12;
isCollinear = ~isNonparallel & ...
    abs(offsetAzimuth_deg .* segment_deg(:, 2) - ...
    offsetElevation_deg .* segment_deg(:, 1)) <= tolerance_deg2;
segmentScale_deg2 = max(sum(segment_deg.^2, 2), eps);
firstProjection = (offsetAzimuth_deg .* segment_deg(:, 1) + ...
    offsetElevation_deg .* segment_deg(:, 2)) ./ segmentScale_deg2;
nextOffsetAzimuth_deg = edgeEnd_deg(:, 1).' - first_deg(:, 1);
nextOffsetElevation_deg = edgeEnd_deg(:, 2).' - first_deg(:, 2);
secondProjection = (nextOffsetAzimuth_deg .* segment_deg(:, 1) + ...
    nextOffsetElevation_deg .* segment_deg(:, 2)) ./ segmentScale_deg2;
overlaps = isCollinear & ...
    max(min(firstProjection, secondProjection), 0) <= ...
    min(max(firstProjection, secondProjection), 1) + 1e-12;
isVisible = isVisible & ~any(crosses | overlaps, 2);
end
