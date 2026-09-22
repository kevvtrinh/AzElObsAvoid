function [boxMinimum_units, boxMaximum_units, regionIsCounterclockwise] = ...
    createCellBoxes(timedRegions, nodeCoordinateScale_units)
%% Section 0: Header & Readme
% SYNTAX
%   [boxMinimum_units, boxMaximum_units, regionIsCounterclockwise] = ...
%       obstacleAvoidance.search.createCellBoxes(timedRegions, nodeCoordinateScale_units)
%**************************************************************************
% PURPOSE
%   - Build an x/y-aligned box around each region throughout its active time
%     interval. Include every point the collision tolerance can treat as
%     blocked, so a quick box check cannot miss a possible collision.
%   - Record whether each boundary runs clockwise or counterclockwise.
%**************************************************************************
% INPUTS
%   - timedRegions (scalar struct)
%       Regions from createTimeCells, with vertices moving linearly between
%       their stored start and end positions.
%   - nodeCoordinateScale_units (numeric scalar)
%       Largest absolute node coordinate used to size the rounding allowance.
%**************************************************************************
% OUTPUTS
%   - boxMinimum_units, boxMaximum_units (C-by-2 numeric)
%       Lower and upper [x y] box corners per region. Infinite bounds keep a
%       region in the full collision check when a finite box is uncertain.
%   - regionIsCounterclockwise (C-by-1 logical)
%       Boundary direction halfway through each region's active interval.
%**************************************************************************
% UNITS
%   - Positions are coordinate units.
%**************************************************************************

%% Section 1: Build A Safe Box For Each Moving Region

regionCount              = numel(timedRegions.Regions_units);
boxMinimum_units         = zeros(regionCount, 2);
boxMaximum_units         = zeros(regionCount, 2);
regionIsCounterclockwise = false(regionCount, 1);
for regionIndex = 1:regionCount
    regionStart_units         = timedRegions.Regions_units{regionIndex};
    regionEnd_units           = timedRegions.EndRegions_units{regionIndex};
    endpointVertices_units    = [regionStart_units; regionEnd_units];
    vertexCount               = size(regionStart_units, 1);
    nextVertexIndices         = [2:vertexCount, 1];
    previousVertexIndices     = [vertexCount, 1:vertexCount - 1];
    midpointVertices_units    = (regionStart_units + regionEnd_units) / 2;
    midpointSignedArea_units2 = sum( ...
        midpointVertices_units(:, 1) .* midpointVertices_units(nextVertexIndices, 2) - ...
        midpointVertices_units(:, 2) .* midpointVertices_units(nextVertexIndices, 1)) / 2;
    % Positive signed area means the boundary runs counterclockwise.
    regionIsCounterclockwise(regionIndex) = midpointSignedArea_units2 >= 0;

    % Find the shortest edge over the whole interval, including between
    % the recorded endpoints. A shrinking edge increases the padding needed
    % to cover points accepted by the collision tolerance.
    startEdgeVectors_units         = regionStart_units(nextVertexIndices, :) - regionStart_units;
    endEdgeVectors_units           = regionEnd_units(nextVertexIndices, :) - regionEnd_units;
    edgeVectorChanges_units        = endEdgeVectors_units - startEdgeVectors_units;
    edgeChangeLengthSquared_units2 = sum(edgeVectorChanges_units.^2, 2);
    shortestEdgeFractions          = zeros(vertexCount, 1);
    edgeVectorChangesWithTime      = edgeChangeLengthSquared_units2 > 0;
    shortestEdgeFractions(edgeVectorChangesWithTime) = min(1, max(0, ...
        -sum(startEdgeVectors_units(edgeVectorChangesWithTime, :) .* ...
        edgeVectorChanges_units(edgeVectorChangesWithTime, :), 2) ./ ...
        edgeChangeLengthSquared_units2(edgeVectorChangesWithTime)));
    shortestEdgeLength_units = min(vecnorm( ...
        startEdgeVectors_units + shortestEdgeFractions .* edgeVectorChanges_units, 2, 2));
    maximumEdgeLengths_units = max(vecnorm(startEdgeVectors_units, 2, 2), vecnorm(endEdgeVectors_units, 2, 2));

    % A flat corner gives zero cross product for its two adjacent edges.
    % With linear vertex motion it changes as C + B x u + A x u^2. Find its
    % smallest magnitude for 0 <= u <= 1, including any flat corner.
    cornerConstant_units2 = obstacleAvoidance.geometry.cross2d( ...
        startEdgeVectors_units(previousVertexIndices, :), startEdgeVectors_units);
    cornerLinear_units2 = obstacleAvoidance.geometry.cross2d( ...
        startEdgeVectors_units(previousVertexIndices, :), edgeVectorChanges_units) + ...
        obstacleAvoidance.geometry.cross2d( ...
        edgeVectorChanges_units(previousVertexIndices, :), startEdgeVectors_units);
    cornerQuadratic_units2 = obstacleAvoidance.geometry.cross2d( ...
        edgeVectorChanges_units(previousVertexIndices, :), edgeVectorChanges_units);
    minimumCornerCrossProduct_units2 = minimumAbsoluteQuadratic( ...
        cornerQuadratic_units2, cornerLinear_units2, cornerConstant_units2);

    % Signed area is also a quadratic in the interval fraction. A near-zero
    % area means the region can flatten and its finite padding is uncertain.
    vertexDisplacement_units = regionEnd_units - regionStart_units;
    areaConstant_units2      = sum( ...
        obstacleAvoidance.geometry.cross2d( ...
        regionStart_units, regionStart_units(nextVertexIndices, :))) / 2;
    areaLinear_units2 = sum( ...
        obstacleAvoidance.geometry.cross2d( ...
        regionStart_units, vertexDisplacement_units(nextVertexIndices, :)) + ...
        obstacleAvoidance.geometry.cross2d( ...
        vertexDisplacement_units, regionStart_units(nextVertexIndices, :))) / 2;
    areaQuadratic_units2 = sum( ...
        obstacleAvoidance.geometry.cross2d( ...
        vertexDisplacement_units, vertexDisplacement_units(nextVertexIndices, :))) / 2;
    minimumAreaMagnitude_units2 = minimumAbsoluteQuadratic( ...
        areaQuadratic_units2, areaLinear_units2, areaConstant_units2);

    % This padding covers numerical tolerance; it does not add another
    % obstacle safety margin. Use the same side tolerance as the full test.
    coordinateScale_units    = max([1; abs(endpointVertices_units(:)); nodeCoordinateScale_units]);
    edgeSideTolerance_units2 = obstacleAvoidance.search.createResidualBound(coordinateScale_units);
    regionMayFlatten = shortestEdgeLength_units <= 0 || ...
        minimumAreaMagnitude_units2 <= edgeSideTolerance_units2 || ...
        any(minimumCornerCrossProduct_units2 <= edgeSideTolerance_units2);
    boxPadding_units = Inf;
    if ~regionMayFlatten
        % Nearly parallel edges give a small sine and need more padding.
        % Minimum corner cross products and maximum edge lengths cover the full
        % interval, even when their extreme values occur at different times.
        cornerSineLowerBound = min(minimumCornerCrossProduct_units2 ./ ...
            (maximumEdgeLengths_units(previousVertexIndices) .* maximumEdgeLengths_units));
        boxPadding_units = 2 * edgeSideTolerance_units2 / (shortestEdgeLength_units * cornerSineLowerBound);
    end

    % Linear vertex motion stays within the endpoint coordinate ranges.
    % Infinite padding prevents the box filter from skipping uncertain regions.
    boxMinimum_units(regionIndex, :) = min(endpointVertices_units, [], 1) - boxPadding_units;
    boxMaximum_units(regionIndex, :) = max(endpointVertices_units, [], 1) + boxPadding_units;
end
end

%% Section 2: Local Functions

function minimumMagnitude_units2 = minimumAbsoluteQuadratic( ...
        quadraticCoefficient_units2, linearCoefficient_units2, constantCoefficient_units2)
    % Find the smallest |A x u^2 + B x u + C| for 0 <= u <= 1.
    % Check both ends, any turning point, and any zero crossing inside.
    minimumMagnitude_units2 = min(abs(constantCoefficient_units2), ...
        abs(quadraticCoefficient_units2 + linearCoefficient_units2 + constantCoefficient_units2));
    hasQuadraticTerm     = quadraticCoefficient_units2 ~= 0;
    turningPointFraction = -linearCoefficient_units2 ./ (2 * quadraticCoefficient_units2);
    turningPointIsInside = hasQuadraticTerm & turningPointFraction > 0 & turningPointFraction < 1;
    if any(turningPointIsInside)
        turningPointMagnitude_units2 = abs( ...
            quadraticCoefficient_units2 .* turningPointFraction.^2 + ...
            linearCoefficient_units2 .* turningPointFraction + constantCoefficient_units2);
        minimumMagnitude_units2(turningPointIsInside) = min( ...
            minimumMagnitude_units2(turningPointIsInside), turningPointMagnitude_units2(turningPointIsInside));
    end

    % A root inside the interval makes the smallest magnitude zero.
    discriminant_units4 = linearCoefficient_units2.^2 - ...
        4 * quadraticCoefficient_units2 .* constantCoefficient_units2;
    discriminantRoot_units2 = sqrt(max(0, discriminant_units4));
    firstRootFraction = (-linearCoefficient_units2 - discriminantRoot_units2) ./ ...
        (2 * quadraticCoefficient_units2);
    secondRootFraction = (-linearCoefficient_units2 + discriminantRoot_units2) ./ ...
        (2 * quadraticCoefficient_units2);
    hasZeroInInterval       = hasQuadraticTerm & discriminant_units4 >= 0 & ...
        ((firstRootFraction >= 0 & firstRootFraction <= 1) | (secondRootFraction >= 0 & secondRootFraction <= 1));

    % If A = 0, solve the linear case; A = B = C = 0 is already zero.
    hasOnlyLinearTerm  = ~hasQuadraticTerm & linearCoefficient_units2 ~= 0;
    linearRootFraction = -constantCoefficient_units2 ./ linearCoefficient_units2;
    hasZeroInInterval  = hasZeroInInterval | (hasOnlyLinearTerm & linearRootFraction >= 0 & linearRootFraction <= 1);
    hasZeroInInterval = hasZeroInInterval | ...
        (~hasQuadraticTerm & linearCoefficient_units2 == 0 & constantCoefficient_units2 == 0);
    minimumMagnitude_units2(hasZeroInInterval) = 0;
end
