function [lower_units, upper_units, isCounterclockwise] = createCellBoxes(cells, nodeScale_units)
%% Section 0: Header & Readme
% SYNTAX
%   [lower_units, upper_units, isCounterclockwise] = ...
%       obstacleAvoidance.search.createCellBoxes(cells, nodeScale_units)
%**************************************************************************
% PURPOSE
%   - Bound every affine time cell over its whole clock by an axis-aligned
%     box widened by the algebraic slack of the exact predicate, and record
%     the cell's orientation.
%**************************************************************************
% INPUTS
%   - cells (scalar struct)
%       Affine time cells from createTimeCells.
%   - nodeScale_units (numeric scalar)
%       Largest node coordinate magnitude, part of the residual scale.
%**************************************************************************
% OUTPUTS
%   - lower_units, upper_units (C-by-2 numeric)
%       Box corners per cell; infinite when the cell flattens.
%   - isCounterclockwise (C-by-1 logical)
%       Orientation of the cell at its middle clock.
%**************************************************************************
% UNITS
%   - Positions are coordinate units.
%**************************************************************************

%% Section 1: Bound Each Cell Over Its Clock

% Outer boxes of the tolerance-expanded affine cells over their clocks.
cellCount   = numel(cells.Regions_units);
lower_units = zeros(cellCount, 2);
upper_units = zeros(cellCount, 2);
isCounterclockwise = false(cellCount, 1);
for cellIndex = 1:cellCount
    regionStart_units = cells.Regions_units{cellIndex};
    regionEnd_units   = cells.EndRegions_units{cellIndex};
    cellBounds_units  = [regionStart_units; regionEnd_units];
    vertexCount       = size(regionStart_units, 1);
    followingIndices  = [2:vertexCount, 1];
    precedingIndices  = [vertexCount, 1:vertexCount - 1];
    middleRegion_units = (regionStart_units + regionEnd_units) / 2;
    middleSignedArea_units2 = sum( ...
        middleRegion_units(:, 1) .* middleRegion_units(followingIndices, 2) - ...
        middleRegion_units(:, 2) .* middleRegion_units(followingIndices, 1)) / 2;
    isCounterclockwise(cellIndex) = middleSignedArea_units2 >= 0;

    edgeStart_units = regionStart_units(followingIndices, :) - regionStart_units;
    edgeEnd_units   = regionEnd_units(followingIndices, :) - regionEnd_units;
    edgeDelta_units = edgeEnd_units - edgeStart_units;
    edgeDeltaScale_units2 = sum(edgeDelta_units.^2, 2);
    edgeFraction = zeros(vertexCount, 1);
    edgeIsMoving = edgeDeltaScale_units2 > 0;
    edgeFraction(edgeIsMoving) = min(1, max(0, ...
        -sum(edgeStart_units(edgeIsMoving, :) .* edgeDelta_units(edgeIsMoving, :), 2) ./ ...
        edgeDeltaScale_units2(edgeIsMoving)));
    shortestEdge_units = min(vecnorm(edgeStart_units + edgeFraction .* edgeDelta_units, 2, 2));
    longestEdge_units  = max(vecnorm(edgeStart_units, 2, 2), vecnorm(edgeEnd_units, 2, 2));

    cornerConstant_units2 = obstacleAvoidance.geometry.cross2d( ...
        edgeStart_units(precedingIndices, :), edgeStart_units);
    cornerLinear_units2 = obstacleAvoidance.geometry.cross2d( ...
        edgeStart_units(precedingIndices, :), edgeDelta_units) + ...
        obstacleAvoidance.geometry.cross2d( ...
        edgeDelta_units(precedingIndices, :), edgeStart_units);
    cornerQuadratic_units2 = obstacleAvoidance.geometry.cross2d( ...
        edgeDelta_units(precedingIndices, :), edgeDelta_units);
    smallestCorner_units2 = minimumAbsoluteQuadratic( ...
        cornerQuadratic_units2, cornerLinear_units2, cornerConstant_units2);

    regionDelta_units = regionEnd_units - regionStart_units;
    areaConstant_units2 = sum( ...
        obstacleAvoidance.geometry.cross2d( ...
        regionStart_units, regionStart_units(followingIndices, :))) / 2;
    areaLinear_units2 = sum( ...
        obstacleAvoidance.geometry.cross2d( ...
        regionStart_units, regionDelta_units(followingIndices, :)) + ...
        obstacleAvoidance.geometry.cross2d( ...
        regionDelta_units, regionStart_units(followingIndices, :))) / 2;
    areaQuadratic_units2 = sum( ...
        obstacleAvoidance.geometry.cross2d( ...
        regionDelta_units, regionDelta_units(followingIndices, :))) / 2;
    smallestArea_units2 = minimumAbsoluteQuadratic( ...
        areaQuadratic_units2, areaLinear_units2, areaConstant_units2);

    coordinateScale_units = max([1; abs(cellBounds_units(:)); nodeScale_units]);
    residualBound_units2  = obstacleAvoidance.search.createResidualBound(coordinateScale_units);
    cellFlattens = shortestEdge_units <= 0 || smallestArea_units2 <= residualBound_units2 || ...
        any(smallestCorner_units2 <= residualBound_units2);
    boxMargin_units = Inf;
    if ~cellFlattens
        smallestSine = min(smallestCorner_units2 ./ ...
            (longestEdge_units(precedingIndices) .* longestEdge_units));
        boxMargin_units = 2 * residualBound_units2 / (shortestEdge_units * smallestSine);
    end
    lower_units(cellIndex, :) = min(cellBounds_units, [], 1) - boxMargin_units;
    upper_units(cellIndex, :) = max(cellBounds_units, [], 1) + boxMargin_units;
end
end

%% Section 2: Local Functions

function minimumAbsolute = minimumAbsoluteQuadratic(quadratic, linear, constant)
    % Smallest magnitude of a quadratic on the closed unit interval.
    minimumAbsolute = min(abs(constant), abs(quadratic + linear + constant));
    isCurved = quadratic ~= 0;
    stationaryClock = -linear ./ (2 * quadratic);
    stationaryIsInside = isCurved & stationaryClock > 0 & stationaryClock < 1;
    if any(stationaryIsInside)
        stationaryValue = abs( ...
            quadratic .* stationaryClock.^2 + linear .* stationaryClock + constant);
        minimumAbsolute(stationaryIsInside) = min( ...
            minimumAbsolute(stationaryIsInside), stationaryValue(stationaryIsInside));
    end
    discriminant = linear.^2 - 4 * quadratic .* constant;
    rootRadius   = sqrt(max(0, discriminant));
    firstRoot    = (-linear - rootRadius) ./ (2 * quadratic);
    secondRoot   = (-linear + rootRadius) ./ (2 * quadratic);
    hasRoot = isCurved & discriminant >= 0 & ...
        ((firstRoot >= 0 & firstRoot <= 1) | (secondRoot >= 0 & secondRoot <= 1));
    isLinear   = ~isCurved & linear ~= 0;
    linearRoot = -constant ./ linear;
    hasRoot = hasRoot | (isLinear & linearRoot >= 0 & linearRoot <= 1);
    hasRoot = hasRoot | (~isCurved & linear == 0 & constant == 0);
    minimumAbsolute(hasRoot) = 0;
end
