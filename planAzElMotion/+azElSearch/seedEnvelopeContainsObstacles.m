function containsAllObstacles = seedEnvelopeContainsObstacles(boundary_deg, obstacles, tolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   containsAllObstacles = azElSearch.seedEnvelopeContainsObstacles( ...
%       boundary_deg, obstacles, tolerance_deg)
%**************************************************************************
% PURPOSE
%   - Verify that one connected certificate region contains a conservative
%     continuous sweep of every protected obstacle history.
%**************************************************************************
% INPUTS
%   - boundary_deg (N-by-2 numeric array)
%       Paired nonfinite rows can separate envelope regions.
%   - obstacles (canonical protected obstacle struct array)
%       Each continuous history must lie in one connected envelope region.
%   - tolerance_deg (nonnegative finite scalar)
%       Outward numerical containment tolerance.
%**************************************************************************
% OUTPUTS
%   - containsAllObstacles (logical scalar)
%       True only when every complete continuous history is covered.
%**************************************************************************
% UNITS
%   - Boundary coordinates and tolerance are degrees.
%**************************************************************************

%% Section 1: Validate Optional Envelope Geometry

% Invalid optional certificate data returns false rather than throwing because
% the planner can report an ordinary candidate-validation failure.
validateattributes(tolerance_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
containsAllObstacles = false;
if isempty(boundary_deg) || ~isnumeric(boundary_deg) || size(boundary_deg, 2) ~= 2
    return;
end
boundary_deg = double(boundary_deg);
if any(xor(isfinite(boundary_deg(:, 1)), isfinite(boundary_deg(:, 2))))
    return;
end
envelopeShape = polyshape( boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
envelopeRegions = regions(envelopeShape);
regionCount = numel(envelopeRegions);
if regionCount < 1
    return;
end

%% Section 2: Buffer Each Connected Certificate Region

% The small outward buffer absorbs polygon Boolean roundoff only; protected
% obstacle margins were already applied by public obstacle construction.
containmentTolerance_deg = max(1e-9, tolerance_deg);
bufferedEnvelopeRegions = cell(regionCount, 1);
for regionIndex = 1:regionCount
    bufferedEnvelopeRegions{regionIndex} = polybuffer( ...
        envelopeRegions(regionIndex), containmentTolerance_deg);
end

%% Section 3: Verify Every Complete Continuous History

% Matching vertices interpolate linearly, so the convex hull of adjacent
% slices contains every polygon occupied during that interval. Topology-change
% intervals use the same prepared conservative union as runtime queries.
for obstacleIndex = 1:numel(obstacles)
    obstacle = azElObstacles.prepareDynamic(obstacles(obstacleIndex));
    preparation = obstacle.InternalPreparation;
    sampleCount = numel(preparation.SampleShapes);
    intervalCount = max(0, sampleCount - 1);
    sweepParts = cell(sampleCount + intervalCount, 1);
    partCount = 0;

    % Source shapes protect exact history times and stationary requests.
    for sampleIndex = 1:sampleCount
        sampleShape = preparation.SampleShapes{sampleIndex};
        if isempty(sampleShape.Vertices)
            return;
        end
        partCount = partCount + 1;
        sweepParts{partCount} = sampleShape;
    end

    % Add a conservative occupied shape for every interval between samples.
    for intervalIndex = 1:intervalCount
        if preparation.MatchingTopology(intervalIndex)
            position_deg = [ ...
                obstacle.az_deg{intervalIndex}(:), ...
                obstacle.el_deg{intervalIndex}(:); ...
                obstacle.az_deg{intervalIndex + 1}(:), ...
                obstacle.el_deg{intervalIndex + 1}(:)];
            position_deg = position_deg(all(isfinite(position_deg), 2), :);
            position_deg = unique(position_deg, "rows", "stable");
            if size(position_deg, 1) < 3
                return;
            end
            hullIndex = convhull(position_deg(:, 1), position_deg(:, 2));
            intervalShape = polyshape( ...
                position_deg(hullIndex(1:end - 1), :), ...
                "Simplify", false, "KeepCollinearPoints", true);
        else
            intervalShape = preparation.IntervalUnionShapes{intervalIndex};
        end
        if isempty(intervalShape.Vertices)
            return;
        end
        partCount = partCount + 1;
        sweepParts{partCount} = intervalShape;
    end
    sweptHistoryShape = union([sweepParts{1:partCount}]);
    areaTolerance_deg2 = 256 * eps(max(1, area(sweptHistoryShape)));
    containingRegionFound = false;

    % A disconnected union cannot certify motion through the gap between its
    % components; one region must contain the obstacle's entire swept history.
    for regionIndex = 1:regionCount
        uncoveredShape = subtract( ...
            sweptHistoryShape, bufferedEnvelopeRegions{regionIndex});
        if area(uncoveredShape) <= areaTolerance_deg2
            containingRegionFound = true;
            break;
        end
    end
    if ~containingRegionFound
        return;
    end
end
containsAllObstacles = true;
end
