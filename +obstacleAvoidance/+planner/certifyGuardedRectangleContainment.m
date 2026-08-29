function [contained, ringPassed, ring_uv_deg] = certifyGuardedRectangleContainment( ...
        bounds_uv_deg, occupiedGeometry, axisOrder, axisSign, ...
        localOrigin_uv_deg, guard_deg, requireOrthogonal)
%% Section 0: Header & Readme
% SYNTAX
%   [contained, ringPassed, ring_uv_deg] = ...
%       obstacleAvoidance.planner.certifyGuardedRectangleContainment( ...
%       bounds_uv_deg, occupiedGeometry, axisOrder, axisSign, ...
%       localOrigin_uv_deg, guard_deg, requireOrthogonal)
%**************************************************************************
% PURPOSE
%   - Prove guarded rectangles lie inside one simple protected polygon ring.
%**************************************************************************
% INPUTS
%   - bounds_uv_deg (N-by-4 numeric)
%       Rows are [minimumU maximumU minimumV maximumV].
%   - occupiedGeometry (scalar polyshape or N-by-2 numeric)
%       Exact prepared shape used by collision queries, or an authoritative
%       NaN-separated source boundary when Boolean repair must be excluded.
%   - axisOrder, axisSign (1-by-2 numeric)
%       Coordinate permutation and signed local-frame orientation.
%   - localOrigin_uv_deg (1-by-2 numeric)
%       Local origin subtracted before guarded predicates.
%   - guard_deg (positive finite scalar)
%       Ambiguity and minimum-separation guard.
%   - requireOrthogonal (logical scalar)
%       True additionally requires every source-ring edge to be axis aligned.
%**************************************************************************
% OUTPUTS
%   - contained (N-by-1 logical)
%       True only for rectangles proven wholly in the occupied interior.
%   - ringPassed (logical scalar)
%       True only for one finite, guarded-simple, non-self-touching ring.
%   - ring_uv_deg (M-by-2 numeric)
%       The checked finite ring in the requested local [u v] frame.
%**************************************************************************
% UNITS
%   - Geometry and guard values are degrees.
%**************************************************************************

%% Section 1: Create And Validate The Local Ring

rectangleCount = size(bounds_uv_deg, 1);
contained = false(rectangleCount, 1);
ringPassed = false;
ring_uv_deg = zeros(0, 2);
isShape = isa(occupiedGeometry, "polyshape") && isscalar(occupiedGeometry);
isBoundary = isnumeric(occupiedGeometry) && size(occupiedGeometry, 2) == 2;
if ~(isShape || isBoundary) || size(bounds_uv_deg, 2) ~= 4 || ...
        any(~isfinite(bounds_uv_deg), "all") || ~isequal(sort(axisOrder), [1 2]) || ...
        ~isequal(abs(axisSign), [1 1]) || ...
        ~isscalar(guard_deg) || ~isfinite(guard_deg) || guard_deg <= 0
    return;
end
if isShape
    [firstCoordinate_deg, secondCoordinate_deg] = boundary(occupiedGeometry);
    world_deg = [firstCoordinate_deg(:), secondCoordinate_deg(:)];
else
    world_deg = double(occupiedGeometry);
end
finiteRow = all(isfinite(world_deg), 2);
if nnz(diff([false; finiteRow; false])) ~= 2
    return;
end
ring_deg = world_deg(finiteRow, :);
if size(ring_deg, 1) > 1 && isequal(ring_deg(1, :), ring_deg(end, :))
    ring_deg(end, :) = [];
end
ring_uv_deg = ring_deg(:, axisOrder) .* axisSign - localOrigin_uv_deg;
edgeStart_deg = ring_uv_deg;
edgeEnd_deg = ring_uv_deg([2:end 1], :);
edgeDelta_deg = edgeEnd_deg - edgeStart_deg;
edgeLength_deg = vecnorm(edgeDelta_deg, 2, 2);
if size(ring_uv_deg, 1) < 4 || any(~isfinite(ring_uv_deg), "all") || ...
        any(edgeLength_deg <= 8 * guard_deg)
    return;
end
if requireOrthogonal && ~all((edgeDelta_deg(:, 1) == 0) ~= ...
        (edgeDelta_deg(:, 2) == 0))
    return;
end
previousDelta_deg = edgeDelta_deg([end 1:end - 1], :);
turn_deg2 = crossRows(previousDelta_deg, edgeDelta_deg);
reversal = abs(turn_deg2) <= 4 * guard_deg * edgeLength_deg & ...
    sum(previousDelta_deg .* edgeDelta_deg, 2) < 0;
if any(reversal)
    return;
end
edgeCount = size(edgeStart_deg, 1);
pairs = nchoosek(1:edgeCount, 2);
adjacent = pairs(:, 2) == pairs(:, 1) + 1 | ...
    pairs(:, 1) == 1 & pairs(:, 2) == edgeCount;
pairs = pairs(~adjacent, :);
if any(~segmentsSeparated(edgeStart_deg(pairs(:, 1), :), ...
        edgeEnd_deg(pairs(:, 1), :), edgeStart_deg(pairs(:, 2), :), ...
        edgeEnd_deg(pairs(:, 2), :), guard_deg))
    return;
end
ringPassed = true;

%% Section 2: Prove Every Rectangle Is Contained

for boundsIndex = 1:rectangleCount
    corners_deg = [bounds_uv_deg(boundsIndex, [1 2 2 1]).', ...
        bounds_uv_deg(boundsIndex, [3 3 4 4]).'];
    sideIndex = repelem((1:4).', edgeCount);
    edgeIndex = repmat((1:edgeCount).', 4, 1);
    if any(~segmentsSeparated(corners_deg(sideIndex, :), ...
            corners_deg(mod(sideIndex, 4) + 1, :), edgeStart_deg(edgeIndex, :), ...
            edgeEnd_deg(edgeIndex, :), guard_deg))
        continue;
    end
    height_deg = edgeStart_deg(:, 2) - corners_deg(:, 2).';
    endHeight_deg = edgeEnd_deg(:, 2) - corners_deg(:, 2).';
    straddles = (height_deg > 0) ~= (endHeight_deg > 0);
    orientation_deg2 = edgeDelta_deg(:, 1) .* ...
        (corners_deg(:, 2).' - edgeStart_deg(:, 2)) - edgeDelta_deg(:, 2) .* ...
        (corners_deg(:, 1).' - edgeStart_deg(:, 1));
    orientationGuard_deg2 = 2 * guard_deg * edgeLength_deg;
    rayPassed = all(abs(height_deg) > 2 * guard_deg & ...
        abs(endHeight_deg) > 2 * guard_deg & ...
        (~straddles | abs(orientation_deg2) > orientationGuard_deg2), 1);
    rayCrosses = straddles & ((orientation_deg2 > orientationGuard_deg2 & ...
        edgeDelta_deg(:, 2) > 0) | (orientation_deg2 < -orientationGuard_deg2 & ...
        edgeDelta_deg(:, 2) < 0));
    contained(boundsIndex) = all(rayPassed & mod(sum(rayCrosses, 1), 2) == 1);
end
end

%% Section 3: Local Functions

function separated = segmentsSeparated(a, b, c, d, guard_deg)
% Reject any crossing, touch, or guard-scale approach of nonadjacent edges.
ab = b - a;
cd = d - c;
orientation = [crossRows(ab, c - a), crossRows(ab, d - a), ...
    crossRows(cd, a - c), crossRows(cd, b - c)];
orientationGuard = 4 * guard_deg * max(vecnorm(ab, 2, 2), vecnorm(cd, 2, 2));
boxesSeparated = any(min(a, b) > max(c, d) + 4 * guard_deg, 2) | ...
    any(min(c, d) > max(a, b) + 4 * guard_deg, 2);
crossing = orientation(:, 1) .* orientation(:, 2) < 0 & ...
    orientation(:, 3) .* orientation(:, 4) < 0;
distanceSquared = [pointSegmentSquared(a, c, d), ...
    pointSegmentSquared(b, c, d), pointSegmentSquared(c, a, b), ...
    pointSegmentSquared(d, a, b)];
separated = boxesSeparated | (all(abs(orientation) > orientationGuard, 2) & ...
    ~crossing & min(distanceSquared, [], 2) > (4 * guard_deg) ^ 2);
end

function value = crossRows(first, second)
% Return rowwise planar cross products.
value = first(:, 1) .* second(:, 2) - first(:, 2) .* second(:, 1);
end

function distanceSquared = pointSegmentSquared(point, startPoint, endPoint)
% Return rowwise squared point-to-segment distances.
delta = endPoint - startPoint;
fraction = sum((point - startPoint) .* delta, 2) ./ sum(delta .^ 2, 2);
fraction = min(1, max(0, fraction));
offset = point - startPoint - fraction .* delta;
distanceSquared = sum(offset .^ 2, 2);
end
