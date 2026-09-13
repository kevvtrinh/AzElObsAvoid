function obstacle = prepareOneObstacle(obstacle, preparationVersion, sourceSnapshot, timeRange_s, previous)
%% Section 0: Header & Readme
% SYNTAX: obstacle = obstacleAvoidance.obstacles.prepareOneObstacle( obstacle, preparationVersion,
%   sourceSnapshot, timeRange_s, previous)
% PURPOSE: Prepare requested entries of one obstacle history for repeated geometry queries.
%   Retain the interval method, bounds, edges, motion, and static status.
% INPUTS: obstacle (scalar canonical obstacle struct) Protected and original source histories remain
%   unchanged.
%   preparationVersion (positive integer scalar) Version written into the internal preparation
%   record.
%   sourceSnapshot (scalar struct) Source fields assembled by prepareObstacles for cache validation.
%   timeRange_s: requested closed interval; omitted means the full history.
%   previous: source-checked preparation to extend; omitted means empty.
% OUTPUTS: obstacle (scalar canonical obstacle struct) InternalPreparation contains reusable
%   source-derived geometry data.
% UNITS: Geometry is coordinate units, time is seconds, and speed is coordinate units per second.

%% Section 1: Select Source Samples Without Changing The History
validateattributes(preparationVersion, {'numeric'}, {'real','finite','scalar','integer','positive'});
if nargin<4, timeRange_s=[-Inf,Inf]; end
if nargin<5, previous=[]; end
time_s=obstacle.time_s; sampleCount=numel(time_s); intervalCount=sampleCount-1;
neededSamples=time_s>=timeRange_s(1) & time_s<=timeRange_s(2);
neededIntervals=time_s(1:end-1)<timeRange_s(2) & time_s(2:end)>timeRange_s(1);
intervalIndices=find(neededIntervals);
neededSamples([intervalIndices;intervalIndices+1])=true;
if sampleCount==1, neededSamples(1)=true; end

%% Section 2: Extend The Single Source-Checked Preparation Record
if isempty(previous)
    preparation=struct('PreparationVersion',preparationVersion,'SourceSnapshot',sourceSnapshot, ...
        'SamplePrepared',false(sampleCount,1),'IntervalPrepared',false(intervalCount,1), ...
        'SampleShapes',{cell(sampleCount,1)}, ...
        'SampleEdgeStart_units',{cell(sampleCount,1)},'SampleEdgeEnd_units',{cell(sampleCount,1)}, ...
        'IntervalUnionShapes',{cell(intervalCount,1)}, ...
        'IntervalUnionEdgeStart_units',{cell(intervalCount,1)}, ...
        'IntervalUnionEdgeEnd_units',{cell(intervalCount,1)}, ...
        'IntervalStartRegions_units',{cell(intervalCount,1)}, ...
        'IntervalEndRegions_units',{cell(intervalCount,1)}, ...
        'DeltaX_units',{cell(intervalCount,1)},'DeltaY_units',{cell(intervalCount,1)}, ...
        'MatchingTopology',false(intervalCount,1),'IntervalGeometryModel',strings(intervalCount,1), ...
        'IntervalSpeedBound_units_s',Inf(intervalCount,1), ...
        'SampleSpeedBound_units_s',Inf(sampleCount,1),'IsTimeInvariant',false);
    % Exact numeric equality can establish a globally static shape without
    % constructing polygons outside the requested window. Activity still uses time_s.
    preparation.SamplesExactlyEqual=all(cellfun(@(x,y)isequaln(x,obstacle.x_units{1}) && ...
        isequaln(y,obstacle.y_units{1}),obstacle.x_units,obstacle.y_units));
    preparation.IsTimeInvariant=preparation.SamplesExactlyEqual;
else
    preparation=previous;
end
for sampleIndex=reshape(find(neededSamples & ~preparation.SamplePrepared),1,[])
    shape=obstacleAvoidance.geometry.boundaryToShape(obstacle.x_units{sampleIndex},obstacle.y_units{sampleIndex});
    preparation.SampleShapes{sampleIndex}=shape;
    [preparation.SampleEdgeStart_units{sampleIndex},preparation.SampleEdgeEnd_units{sampleIndex}]= ...
        obstacleAvoidance.geometry.boundaryToEdges(shape,0);
    preparation.SamplePrepared(sampleIndex)=true;
end

%% Section 3: Prepare Each Newly Requested Source Interval Once
for intervalIndex=reshape(find(neededIntervals & ~preparation.IntervalPrepared),1,[])
    lowerX_units=obstacle.x_units{intervalIndex}; lowerY_units=obstacle.y_units{intervalIndex};
    upperX_units=obstacle.x_units{intervalIndex+1}; upperY_units=obstacle.y_units{intervalIndex+1};
    [matched,alignedUpper_units,startRegions_units,endRegions_units,geometryModel]= ...
        alignVerifiedSingleRing(lowerX_units,lowerY_units,upperX_units,upperY_units, ...
        preparation.SampleShapes{intervalIndex},preparation.SampleShapes{intervalIndex+1});
    preparation.MatchingTopology(intervalIndex)=matched;
    if matched
        preparation.DeltaX_units{intervalIndex}=alignedUpper_units(:,1)-lowerX_units;
        preparation.DeltaY_units{intervalIndex}=alignedUpper_units(:,2)-lowerY_units;
        preparation.IntervalStartRegions_units{intervalIndex}=startRegions_units;
        preparation.IntervalEndRegions_units{intervalIndex}=endRegions_units;
        speed_units_s=hypot(preparation.DeltaX_units{intervalIndex},preparation.DeltaY_units{intervalIndex})/diff(time_s(intervalIndex:intervalIndex+1));
        preparation.IntervalSpeedBound_units_s(intervalIndex)=max([0;speed_units_s(isfinite(speed_units_s))]);
        preparation.IntervalGeometryModel(intervalIndex)=geometryModel;
    else
        firstShape=preparation.SampleShapes{intervalIndex}; lastShape=preparation.SampleShapes{intervalIndex+1};
        [equivalent,~]=compareShapes(firstShape,lastShape);
        if equivalent
            shape=firstShape; method="staticEquivalentSamples";
        else
            shape=polyshape(); method="unsupportedContinuousDeformation";
        end
        preparation.IntervalUnionShapes{intervalIndex}=shape;
        preparation.IntervalGeometryModel(intervalIndex)=method;
        preparation.IntervalSpeedBound_units_s(intervalIndex)=0;
        [preparation.IntervalUnionEdgeStart_units{intervalIndex},preparation.IntervalUnionEdgeEnd_units{intervalIndex}]= ...
            obstacleAvoidance.geometry.boundaryToEdges(shape,0);
    end
    preparation.IntervalPrepared(intervalIndex)=true;
end

%% Section 4: Retain Conservative Bounds At Unprepared Neighbor Intervals
preparation.SampleSpeedBound_units_s=max([0;preparation.IntervalSpeedBound_units_s], ...
    [preparation.IntervalSpeedBound_units_s;0]);
staticIntervals=preparation.IntervalGeometryModel=="staticEquivalentSamples" | ...
    (preparation.MatchingTopology & preparation.IntervalSpeedBound_units_s==0);
preparation.IsTimeInvariant=preparation.IsTimeInvariant || ...
    (all(preparation.IntervalPrepared) && all(staticIntervals));
if preparation.IsTimeInvariant, preparation.SampleSpeedBound_units_s(:)=0; end
obstacle.InternalPreparation=preparation;
end

%% Section 5: Local Functions
function [verified, alignedUpper_units, startRegions_units, endRegions_units, geometryModel] = ...
        alignVerifiedSingleRing(lowerX_units, lowerY_units, upperX_units, upperY_units, lowerShape, upperShape)
    % Align rings, then certify either one moving convex region or an exact
    % moving convex partition of the complete interpolated polygon.
    lower_units        = [lowerX_units(:), lowerY_units(:)];
    upper_units        = [upperX_units(:), upperY_units(:)];
    verified         = false;
    alignedUpper_units = zeros(0, 2);
    startRegions_units = cell(0,1);
    endRegions_units = cell(0,1);
    geometryModel = "";
    lowerFinite = all(isfinite(lower_units),2);
    upperFinite = all(isfinite(upper_units),2);
    if isequal(lowerFinite,upperFinite) && nnz(lowerFinite)>=3
        finiteDelta_units = upper_units(lowerFinite,:)-lower_units(lowerFinite,:);
        coordinateScale_units = max([1;abs(lower_units(lowerFinite,1)); ...
            abs(lower_units(lowerFinite,2));abs(upper_units(upperFinite,1)); ...
            abs(upper_units(upperFinite,2))]);
        translationTolerance_units = 512*eps(coordinateScale_units);
        if max(abs(finiteDelta_units-finiteDelta_units(1,:)),[],'all')<=translationTolerance_units
            alignedUpper_units = upper_units;
            startRegions_units = obstacleAvoidance.geometry.convexRegions(lowerShape,true);
            endRegions_units = cellfun(@(region)region+finiteDelta_units(1,:), ...
                startRegions_units,'UniformOutput',false);
            verified = true;
            geometryModel = "linearCorrespondingConvexPartition";
            return;
        end
    end
    isSingleRing     = size(lower_units, 1) >= 3 && isequal(size(lower_units), size(upper_units)) && all(isfinite(lower_units), "all") && all(isfinite(upper_units), "all");
    if ~isSingleRing
        return;
    end
    % Identical rings already attain the first possible zero-distance match.
    if isequal(lower_units,upper_units)
        alignedUpper_units = upper_units;
        verified = true;
        geometryModel = "linearCorrespondingVertices";
        return;
    end
    % Center and scale before FFT correlation: translation does not affect
    % the least-squares correspondence, and normalization avoids cancellation
    % when a small polygon is far from the coordinate origin.
    centeredLower = lower_units-mean(lower_units,1);
    centeredUpper = upper_units-mean(upper_units,1);
    scale_units = max(abs([centeredLower;centeredUpper]),[],'all');
    if scale_units==0, return; end
    centeredLower = centeredLower/scale_units;
    centeredUpper = centeredUpper/scale_units;
    lowerSpectrum = fft(centeredLower);
    anchorChoices = find(lower_units(:,1)==min(lower_units(:,1)));
    [~,anchorChoice] = min(lower_units(anchorChoices,2));
    anchorIndex = anchorChoices(anchorChoice);
    bestSquaredCost = Inf;
    % Circular correlation evaluates every cyclic alignment in O(N log N).
    % Only the two selected shifts are materialized; there is no shift-loop fallback.
    for orientationIndex = 1:2
        orientedUpper_units = upper_units;
        orientedUpper = centeredUpper;
        if orientationIndex == 2
            orientedUpper_units = flipud(orientedUpper_units);
            orientedUpper = flipud(orientedUpper);
        end
        correlation = sum(real(ifft(lowerSpectrum.*conj(fft(orientedUpper)))),2);
        % Resolve numerically tied alignments at a physical anchor vertex,
        % independent of either incoming ring's starting index.
        tieTolerance = 64*ceil(log2(size(lower_units,1)))*eps(max(abs(correlation)));
        shifts = find(correlation>=max(correlation)-tieTolerance);
        anchorVertices_units = orientedUpper_units(mod(anchorIndex-shifts,size(lower_units,1))+1,:);
        choices = find(anchorVertices_units(:,1)==min(anchorVertices_units(:,1)));
        [~,choice] = min(anchorVertices_units(choices,2));
        shiftIndex = shifts(choices(choice));
        shiftCount = shiftIndex-1;
        squaredCost = sum((circshift(orientedUpper,shiftCount,1)-centeredLower).^2,'all');
        if squaredCost < bestSquaredCost
            bestSquaredCost = squaredCost;
            alignedUpper_units = circshift(orientedUpper_units,shiftCount,1);
        end
    end
    delta_units                = alignedUpper_units - lower_units;
    coordinateScale_units      = max([ 1; abs(lower_units(:)); abs(alignedUpper_units(:))]);
    translationTolerance_units = 512 * eps(coordinateScale_units);
    isTranslation            = max(abs(delta_units - delta_units(1, :)), [], "all") <= translationTolerance_units;
    if isTranslation
        % A translation preserves every face of one exact partition. Build
        % the terminal faces by translating those same faces; this avoids
        % relying on polyshape's vertex ordering after cyclic/reversed input.
        startRegions_units = obstacleAvoidance.geometry.convexRegions(lowerShape,true);
        endRegions_units = cellfun(@(region)region+delta_units(1,:), ...
            startRegions_units,'UniformOutput',false);
        verified = ~isempty(startRegions_units);
        if verified
            geometryModel = "linearCorrespondingConvexPartition";
            return;
        end
    end
    isConvexMotion = remainsStrictlyConvex(lower_units,alignedUpper_units,coordinateScale_units);
    verified = isConvexMotion;
    if verified
        geometryModel = "linearCorrespondingVertices";
        return;
    end
    [globalAffineVerified,~] = verifiedGlobalAffineMap( ...
        lower_units,alignedUpper_units,coordinateScale_units);
    [partitionVerified,startRegions_units,endRegions_units] = ...
        createVerifiedMovingPartition(lower_units,alignedUpper_units,lowerShape,upperShape,coordinateScale_units);
    verified = partitionVerified && (globalAffineVerified || movingBoundaryRemainsSimple( ...
        lower_units,alignedUpper_units,coordinateScale_units));
    if verified
        geometryModel = "linearCorrespondingConvexPartition";
        return;
    end
    if ~verified
        alignedUpper_units = zeros(0, 2);
        startRegions_units = cell(0,1);
        endRegions_units = cell(0,1);
    end
end

function [verified,transformation] = verifiedGlobalAffineMap(lower_units,upper_units,coordinateScale_units)
    % A single affine map preserves every edge and face. Its linear blend
    % with identity is valid when the determinant stays strictly positive.
    source = [lower_units,ones(size(lower_units,1),1)];
    transformation = source\upper_units;
    residual_units = source*transformation-upper_units;
    residualTolerance_units = 4096*eps(coordinateScale_units);
    if max(abs(residual_units),[],'all')>residualTolerance_units
        verified = false;
        return;
    end
    linearMap = transformation(1:2,:);
    deltaMap = linearMap-eye(2);
    determinantCoefficients = [det(deltaMap), ...
        deltaMap(1,1)+deltaMap(2,2),1];
    candidateTau = [0;1];
    if determinantCoefficients(1)~=0
        stationaryTau = -determinantCoefficients(2)/(2*determinantCoefficients(1));
        if stationaryTau>0 && stationaryTau<1
            candidateTau(end+1,1)=stationaryTau;
        end
    end
    determinants = polyval(determinantCoefficients,candidateTau);
    determinantTolerance = 4096*eps(max(1,norm(linearMap,'fro')^2));
    verified = all(determinants>determinantTolerance);
end

function [verified,startRegions_units,endRegions_units] = createVerifiedMovingPartition( ...
        lower_units,upper_units,lowerShape,upperShape,coordinateScale_units)
    % Carry one exact lower-sample partition through the supplied vertex
    % correspondence. Every face must remain convex for the whole interval.
    verified = false;
    startRegions_units = obstacleAvoidance.geometry.convexRegions(lowerShape,true);
    endRegions_units = cell(size(startRegions_units));
    for regionIndex = 1:numel(startRegions_units)
        [isSourceVertex,sourceIndex] = ismember(startRegions_units{regionIndex},lower_units,'rows');
        if ~all(isSourceVertex) || numel(unique(sourceIndex))~=numel(sourceIndex)
            startRegions_units = cell(0,1);
            endRegions_units = cell(0,1);
            return;
        end
        endRegions_units{regionIndex} = upper_units(sourceIndex,:);
        if ~remainsStrictlyConvex(startRegions_units{regionIndex}, ...
                endRegions_units{regionIndex},coordinateScale_units)
            [verified,startRegions_units,endRegions_units] = createVerifiedMovingTriangles( ...
                lower_units,upper_units,lowerShape,upperShape,coordinateScale_units);
            return;
        end
    end
    verified = partitionMatchesEndpointShapes(startRegions_units,endRegions_units,lowerShape,upperShape);
end

function [verified,startRegions_units,endRegions_units] = createVerifiedMovingTriangles( ...
        lower_units,upper_units,lowerShape,upperShape,coordinateScale_units)
    % A triangle mesh is the non-heuristic fallback partition. It is
    % accepted only when every mesh point is an original corresponding vertex.
    verified = false;
    startRegions_units = cell(0,1);
    endRegions_units = cell(0,1);
    mesh = triangulation(lowerShape);
    [isSourceVertex,sourceIndex] = ismember(mesh.Points,lower_units,'rows');
    if ~all(isSourceVertex) || numel(unique(sourceIndex))~=numel(sourceIndex)
        return;
    end
    faceCount = size(mesh.ConnectivityList,1);
    startRegions_units = cell(faceCount,1);
    endRegions_units = cell(faceCount,1);
    for faceIndex = 1:faceCount
        pointIndex = mesh.ConnectivityList(faceIndex,:);
        startRegions_units{faceIndex} = mesh.Points(pointIndex,:);
        endRegions_units{faceIndex} = upper_units(sourceIndex(pointIndex),:);
        if ~remainsStrictlyConvex(startRegions_units{faceIndex}, ...
                endRegions_units{faceIndex},coordinateScale_units)
            startRegions_units = cell(0,1);
            endRegions_units = cell(0,1);
            return;
        end
    end
    verified = partitionMatchesEndpointShapes(startRegions_units,endRegions_units,lowerShape,upperShape);
end

function verified = partitionMatchesEndpointShapes(startRegions_units,endRegions_units,lowerShape,upperShape)
    % Endpoint Boolean equality catches mapping or triangulation changes
    % before the continuous face certificates are trusted.
    startUnion = unionRegions(startRegions_units);
    endUnion = unionRegions(endRegions_units);
    areaScale_units2 = max([1,area(lowerShape),area(upperShape)]);
    areaTolerance_units2 = 4096*eps(areaScale_units2);
    verified = area(xor(startUnion,lowerShape))<=areaTolerance_units2 && ...
        area(xor(endUnion,upperShape))<=areaTolerance_units2;
end

function shape = unionRegions(regions_units)
    shape = polyshape();
    for regionIndex = 1:numel(regions_units)
        region_units = regions_units{regionIndex};
        shape = union(shape,polyshape(region_units,'Simplify',false,'KeepCollinearPoints',true));
    end
end

function verified = movingBoundaryRemainsSimple(lower_units,upper_units,coordinateScale_units)
    % A crossing can begin or end only when one endpoint becomes collinear
    % with the other moving edge. Split time at every such quadratic root,
    % then test the roots and the open intervals between them.
    verified = true;
    count = size(lower_units,1);
    delta_units = upper_units-lower_units;
    orientationTolerance_units2 = 4096*eps(coordinateScale_units^2);
    positionTolerance_units = 4096*eps(coordinateScale_units);
    for firstIndex = 1:count
        firstNext = mod(firstIndex,count)+1;
        for secondIndex = firstIndex+1:count
            secondNext = mod(secondIndex,count)+1;
            if secondIndex==firstNext || secondNext==firstIndex
                continue;
            end
            triples = [firstIndex,firstNext,secondIndex; ...
                firstIndex,firstNext,secondNext;secondIndex,secondNext,firstIndex; ...
                secondIndex,secondNext,firstNext];
            criticalTau = [0;1];
            for tripleIndex = 1:4
                indices = triples(tripleIndex,:);
                coefficients = orientationCoefficients(lower_units(indices,:),delta_units(indices,:));
                if all(abs(coefficients)<=orientationTolerance_units2)
                    verified = false;
                    return;
                end
                rootsTau = realPolynomialRoots(coefficients,orientationTolerance_units2);
                criticalTau = [criticalTau;rootsTau]; %#ok<AGROW>
            end
            criticalTau = unique(min(1,max(0,criticalTau)));
            probeTau = [criticalTau;(criticalTau(1:end-1)+criticalTau(2:end))/2];
            for tau = reshape(probeTau,1,[])
                points_units = lower_units+tau*delta_units;
                if segmentsIntersect(points_units(firstIndex,:),points_units(firstNext,:), ...
                        points_units(secondIndex,:),points_units(secondNext,:), ...
                        orientationTolerance_units2,positionTolerance_units)
                    verified = false;
                    return;
                end
            end
        end
    end
end

function coefficients = orientationCoefficients(points_units,delta_units)
    firstEdge_units = points_units(2,:)-points_units(1,:);
    secondEdge_units = points_units(3,:)-points_units(1,:);
    firstDelta_units = delta_units(2,:)-delta_units(1,:);
    secondDelta_units = delta_units(3,:)-delta_units(1,:);
    coefficients = [cross2d(firstDelta_units,secondDelta_units), ...
        cross2d(firstDelta_units,secondEdge_units)+cross2d(firstEdge_units,secondDelta_units), ...
        cross2d(firstEdge_units,secondEdge_units)];
end

function rootsTau = realPolynomialRoots(coefficients,tolerance)
    if abs(coefficients(1))<=tolerance
        if abs(coefficients(2))<=tolerance
            rootsTau = zeros(0,1);
        else
            rootsTau = -coefficients(3)/coefficients(2);
        end
    else
        candidate = roots(coefficients);
        imaginaryTolerance = 1024*eps(max(1,max(abs(candidate))));
        rootsTau = real(candidate(abs(imag(candidate))<=imaginaryTolerance));
    end
    timeTolerance = 1024*eps;
    rootsTau = rootsTau(rootsTau>=-timeTolerance & rootsTau<=1+timeTolerance);
end

function intersects = segmentsIntersect(firstStart_units,firstEnd_units,secondStart_units,secondEnd_units, ...
        orientationTolerance_units2,positionTolerance_units)
    firstDirection_units = firstEnd_units-firstStart_units;
    secondDirection_units = secondEnd_units-secondStart_units;
    orientations = [cross2d(firstDirection_units,secondStart_units-firstStart_units), ...
        cross2d(firstDirection_units,secondEnd_units-firstStart_units), ...
        cross2d(secondDirection_units,firstStart_units-secondStart_units), ...
        cross2d(secondDirection_units,firstEnd_units-secondStart_units)];
    signs = sign(orientations);
    signs(abs(orientations)<=orientationTolerance_units2) = 0;
    boundingBoxesOverlap = all(max(min([firstStart_units;firstEnd_units]), ...
        min([secondStart_units;secondEnd_units])) <= ...
        min(max([firstStart_units;firstEnd_units]),max([secondStart_units;secondEnd_units]))+positionTolerance_units);
    intersects = boundingBoxesOverlap && signs(1)*signs(2)<=0 && signs(3)*signs(4)<=0;
end

function verified = remainsStrictlyConvex(lower_units, upper_units, coordinateScale_units)
    % Check that interpolated turns keep the same nonzero sign on [0, 1].
    lowerEdge_units      = circshift(lower_units, -1, 1) - lower_units;
    upperEdge_units      = circshift(upper_units, -1, 1) - upper_units;
    lowerTurn_units2     = cross2d(lowerEdge_units, circshift(lowerEdge_units, -1, 1));
    orientation        = sign(sum(lowerTurn_units2));
    turnTolerance_units2 = 4096 * eps(coordinateScale_units ^ 2);
    if orientation == 0 || any(orientation * lowerTurn_units2 <= turnTolerance_units2)
        verified = false;
        return;
    end
    edgeDelta_units     = upperEdge_units - lowerEdge_units;
    nextLowerEdge_units = circshift(lowerEdge_units, -1, 1);
    nextEdgeDelta_units = circshift(edgeDelta_units, -1, 1);
    constant_units2     = cross2d(lowerEdge_units, nextLowerEdge_units);
    linear_units2       = cross2d(edgeDelta_units, nextLowerEdge_units) + cross2d(lowerEdge_units, nextEdgeDelta_units);
    quadratic_units2    = cross2d(edgeDelta_units, nextEdgeDelta_units);
    verified          = true;
    for vertexIndex = 1:size(lower_units, 1)
        candidateTau = [0; 1];
        if quadratic_units2(vertexIndex) ~= 0
            stationaryTau = -linear_units2(vertexIndex) / (2 * quadratic_units2(vertexIndex));
            if stationaryTau > 0 && stationaryTau < 1
                candidateTau(end + 1, 1) = stationaryTau; %#ok<AGROW>
            end
        end
        turn_units2 = constant_units2(vertexIndex) + linear_units2(vertexIndex) * candidateTau + quadratic_units2(vertexIndex) * candidateTau .^ 2;
        if any(orientation * turn_units2 <= turnTolerance_units2)
            verified = false;
            return;
        end
    end
end

function value = cross2d(first_units, second_units)
    % Return row-wise signed two-dimensional cross products.
    value = first_units(:, 1) .* second_units(:, 2) - first_units(:, 2) .* second_units(:, 1);
end

function [equivalent, nested] = compareShapes(firstShape, secondShape)
    % Check equality and containment using shape differences.
    firstArea_units2 = area(firstShape);
    secondArea_units2 = area(secondShape);
    areaScale_units2     = max([1, firstArea_units2, secondArea_units2]);
    areaTolerance_units2 = 512 * eps(areaScale_units2);
    % A larger polygon cannot fit inside a smaller one: the area difference
    % bounds the subtraction from below. Keep a full extra tolerance of
    % roundoff reserve and perform the original Boolean check near equality.
    firstIsContained = firstArea_units2 <= secondArea_units2+2*areaTolerance_units2 && ...
        area(subtract(firstShape, secondShape)) <= areaTolerance_units2;
    secondIsContained = secondArea_units2 <= firstArea_units2+2*areaTolerance_units2 && ...
        area(subtract(secondShape, firstShape)) <= areaTolerance_units2;
    equivalent         = firstIsContained && secondIsContained;
    nested             = firstIsContained || secondIsContained;
end
