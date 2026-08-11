function schedule = buildAdaptiveSchedule(request)
%% Section 0: Header & Readme
% SYNTAX
%   schedule = buildAdaptiveSchedule(request)
%**************************************************************************
% PURPOSE
%   - Derive deterministic coarse-to-fine spatial and temporal work from
%     geometry, dynamics, obstacle cadence, seam policy, and mission horizon.
%**************************************************************************
% INPUTS
%   - request (normalized scalar planning request)
%**************************************************************************
% OUTPUTS
%   - schedule (scalar struct)
%       Private refinement sequence and scene facts recorded in diagnostics.
%**************************************************************************
% UNITS
%   - Spatial resolution is degrees and temporal resolution is seconds.

%% Section 1: Measure Scene Geometry & Timing
startTime_s = request.initialState.time_s;
referenceTime_s = min(request.options.deadline_s, ...
    startTime_s + 0.5 .* (request.options.deadline_s - startTime_s));
if request.goal.type == "moving"
    referenceTime_s = max(referenceTime_s, request.goal.time_s(1));
end
referenceGoal = evaluateAzElGoal(request.goal, referenceTime_s);
startPosition_deg = request.initialState.position_deg;
goalSeparation_deg = norm(referenceGoal.position_deg - startPosition_deg);

azimuthExtent_deg = diff(request.limits.azimuth_deg);
domainExtent_deg = norm([azimuthExtent_deg, ...
    diff(request.limits.elevation_deg)]);

edgeLengths_deg = zeros(0, 1);
sampleCadence_s = zeros(0, 1);
hasTimeVariation = request.goal.type == "moving";
for obstacleIndex = 1:numel(request.obstacles)
    obstacle = request.obstacles{obstacleIndex};
    if numel(obstacle.time_s) > 1
        sampleCadence_s = [sampleCadence_s; diff(obstacle.time_s)]; ...
            %#ok<AGROW>
    end
    firstAzimuth_deg = obstacle.az_deg{1};
    firstElevation_deg = obstacle.el_deg{1};
    for sampleIndex = 1:numel(obstacle.time_s)
        regions_deg = splitAzElRegions(obstacle.az_deg{sampleIndex}, ...
            obstacle.el_deg{sampleIndex});
        for regionIndex = 1:numel(regions_deg)
            vertices_deg = regions_deg{regionIndex};
            closedVertices_deg = [vertices_deg; vertices_deg(1, :)];
            edgeLengths_deg = [edgeLengths_deg; ...
                vecnorm(diff(closedVertices_deg, 1, 1), 2, 2)]; ...
                %#ok<AGROW>
        end
        if sampleIndex > 1 && ...
                (~isequaln(firstAzimuth_deg, obstacle.az_deg{sampleIndex}) || ...
                ~isequaln(firstElevation_deg, obstacle.el_deg{sampleIndex}))
            hasTimeVariation = true;
        end
    end
end
edgeLengths_deg = edgeLengths_deg(edgeLengths_deg > ...
    256 .* eps(max(1, domainExtent_deg)));
if isempty(edgeLengths_deg)
    minimumFeature_deg = max(domainExtent_deg ./ 12, 1);
else
    minimumFeature_deg = min(edgeLengths_deg);
end

%% Section 2: Derive Coarse-To-Fine Levels
safetyScale_deg = max(request.options.safetyMargin_deg, ...
    domainExtent_deg ./ 1000);
coarseSpatial_deg = median([ ...
    max(goalSeparation_deg ./ 8, safetyScale_deg), ...
    max(minimumFeature_deg ./ 2, safetyScale_deg), ...
    max(domainExtent_deg ./ 40, safetyScale_deg)]);
coarseSpatial_deg = min(max(coarseSpatial_deg, ...
    domainExtent_deg ./ 160), domainExtent_deg ./ 6);
minimumSpatial_deg = max([ ...
    safetyScale_deg ./ 3, minimumFeature_deg ./ 12, ...
    domainExtent_deg ./ 640, 0.02]);

horizon_s = request.options.deadline_s - startTime_s;
if isempty(sampleCadence_s)
    cadence_s = horizon_s ./ 8;
else
    cadence_s = median(sampleCadence_s);
end
dynamicCrossing_s = safetyScale_deg ./ ...
    max(norm(request.limits.maxVelocity_deg_s), eps);
coarseTemporal_s = min([horizon_s ./ 6, cadence_s, ...
    max(dynamicCrossing_s .* 2, horizon_s ./ 80)]);
coarseTemporal_s = min(max(coarseTemporal_s, 0.05), ...
    max(horizon_s ./ 3, 0.05));
minimumTemporal_s = max(min([cadence_s ./ 8, ...
    dynamicCrossing_s ./ 2, horizon_s ./ 256]), 0.01);

if isempty(request.obstacles) && request.goal.type == "fixed"
    maximumLevelCount = 1;
elseif hasTimeVariation
    maximumLevelCount = 5;
else
    maximumLevelCount = 4;
end
spatialResolution_deg = zeros(maximumLevelCount, 1);
temporalResolution_s = zeros(maximumLevelCount, 1);
for levelIndex = 1:maximumLevelCount
    spatialResolution_deg(levelIndex) = max( ...
        coarseSpatial_deg ./ 2.^(levelIndex - 1), minimumSpatial_deg);
    temporalResolution_s(levelIndex) = max( ...
        coarseTemporal_s ./ 2.^(levelIndex - 1), minimumTemporal_s);
end
[spatialResolution_deg, uniqueIndices] = unique( ...
    spatialResolution_deg, "stable");
temporalResolution_s = temporalResolution_s(uniqueIndices);

schedule = struct( ...
    "spatialResolution_deg", spatialResolution_deg, ...
    "temporalResolution_s", temporalResolution_s, ...
    "hasTimeVariation", hasTimeVariation, ...
    "goalSeparation_deg", goalSeparation_deg, ...
    "minimumFeature_deg", minimumFeature_deg, ...
    "obstacleCadence_s", cadence_s, ...
    "referenceTime_s", referenceTime_s);
end
