function azElData = calculateAreaTargetAzEl( ...
        targetName, boundaryLatLon_deg, time_s, sensorFixed_km, ...
        CfixedToSensor, maximumBoundaryStep_deg)
%% Section 0: Header & Readme
% SYNTAX
%   azElData = calculateAreaTargetAzEl( ...
%       targetName, boundaryLatLon_deg, time_s, sensorFixed_km, ...
%       CfixedToSensor, maximumBoundaryStep_deg)
%**************************************************************************
% PURPOSE
%   - Project one stationary latitude/longitude area boundary into the
%     sensor azimuth/elevation frame at every requested time.
%   - Clip against the WGS-84 horizon and return one obstacle record accepted
%     by the build-core planner. Requires Aerospace Toolbox (lla2ecef).
%**************************************************************************
% INPUTS
%   - targetName (scalar text)
%       Area name retained in the canonical obstacle record.
%   - boundaryLatLon_deg (N-by-2 numeric matrix, N >= 3)
%       Boundary vertices ordered as [latitude longitude]. The first vertex
%       may be repeated as the final vertex.
%   - time_s (nonempty strictly increasing numeric vector)
%       Projection sample times matching the position and attitude history.
%   - sensorFixed_km (numel(time_s)-by-3 numeric matrix)
%       Sensor position history in the Earth-fixed frame.
%   - CfixedToSensor (3-by-3-by-numel(time_s) numeric array)
%       Direction-cosine matrices rotating Earth-fixed directions into the
%       sensor frame.
%   - maximumBoundaryStep_deg (positive numeric scalar)
%       Largest latitude/longitude step used when densifying each edge.
%**************************************************************************
% OUTPUTS
%   - azElData (scalar struct)
%       Canonical obstacle record containing targetName, time_s, x_units,
%       y_units, originalX_units, originalY_units, safetyMargin_units, status.
%       x is azimuth and y is elevation, both in degrees. Retained original
%       fields refer to the projected boundary before buffering;
%       they do not retain the geographic boundary.
%       The returned safety margin is zero and may be applied later through
%       the repository's public obstacle-protection workflow.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Latitude, longitude, azimuth, elevation, and boundary step are degrees.
%     Earth-fixed positions are kilometers and time is seconds.
%**************************************************************************

%% Section 1: Validate & Normalize Inputs

narginchk(6, 6);
validateattributes(boundaryLatLon_deg, {'numeric'}, ...
    {'2d', 'ncols', 2, 'finite', 'real'});
if size(boundaryLatLon_deg, 1) < 3
    error("calculateAreaTargetAzEl:BoundaryTooShort", ...
        "boundaryLatLon_deg requires at least three vertices.");
end
validateattributes(time_s, {'numeric'}, ...
    {'vector', 'nonempty', 'finite', 'real', 'increasing'});
validateattributes(sensorFixed_km, {'numeric'}, ...
    {'2d', 'ncols', 3, 'finite', 'real'});
validateattributes(CfixedToSensor, {'numeric'}, {'finite', 'real'});
validateattributes(maximumBoundaryStep_deg, {'numeric'}, ...
    {'scalar', 'finite', 'real', 'positive'});

time_s = double(time_s(:));
numberOfTimes = numel(time_s);
attitudeSizeIsValid = size(CfixedToSensor, 1) == 3 && ...
    size(CfixedToSensor, 2) == 3 && ...
    size(CfixedToSensor, 3) == numberOfTimes;
if size(sensorFixed_km, 1) ~= numberOfTimes || ~attitudeSizeIsValid
    error("calculateAreaTargetAzEl:HistorySizeMismatch", ...
        "time_s, sensorFixed_km, and CfixedToSensor histories must align.");
end

if isequal(boundaryLatLon_deg(1, :), boundaryLatLon_deg(end, :))
    boundaryLatLon_deg(end, :) = [];
end

originalPointCount = size(boundaryLatLon_deg, 1);
if originalPointCount < 3 || size(unique(boundaryLatLon_deg, 'rows'), 1) < 3
    error("calculateAreaTargetAzEl:BoundaryTooShort", ...
        "The boundary must contain at least three distinct vertices.");
end

%% Section 2: Densify & Convert The Stationary Boundary

numberOfVertices = size(boundaryLatLon_deg, 1);
boundarySegments = cell(numberOfVertices, 1);

for vertexIndex = 1:numberOfVertices
    nextIndex = mod(vertexIndex, numberOfVertices) + 1;
    delta_deg = boundaryLatLon_deg(nextIndex, :) - ...
        boundaryLatLon_deg(vertexIndex, :);
    delta_deg(2) = mod(delta_deg(2) + 180, 360) - 180;

    numberOfSegments = max(1, ceil( ...
        hypot(delta_deg(1), delta_deg(2)) / maximumBoundaryStep_deg));
    fraction = (0:numberOfSegments - 1).' / numberOfSegments;
    boundarySegments{vertexIndex} = ...
        boundaryLatLon_deg(vertexIndex, :) + fraction .* delta_deg;
    boundarySegments{vertexIndex}(:, 2) = mod( ...
        boundarySegments{vertexIndex}(:, 2) + 180, 360) - 180;
end

denseLatLon_deg = vertcat(boundarySegments{:});
denseLatLon_deg(end + 1, :) = denseLatLon_deg(1, :);
denseFixed_km = lla2ecef( ...
    [denseLatLon_deg, zeros(size(denseLatLon_deg, 1), 1)]) / 1000;

earthA_km = 6378.137;
earthB_km = 6356.752314245;
% WGS-84 surface normals are reused at every time step.
surfaceNormal = [ ...
    denseFixed_km(:, 1) / earthA_km^2, ...
    denseFixed_km(:, 2) / earthA_km^2, ...
    denseFixed_km(:, 3) / earthB_km^2];

az_deg = cell(numberOfTimes, 1);
el_deg = cell(numberOfTimes, 1);
status = strings(numberOfTimes, 1);

%% Section 3: Clip & Project Each Boundary Slice

for timeIndex = 1:numberOfTimes
    sensorPosition_km = sensorFixed_km(timeIndex, :);
    CfixedToSensorNow = CfixedToSensor(:, :, timeIndex);

    visibility = sum(surfaceNormal .* ...
        (sensorPosition_km - denseFixed_km), 2);
    % A nonnegative tangent-plane value places the sensor above the point's
    % local horizon and therefore makes that surface point visible.
    isVisible = visibility >= 0;
    visibleFixed_km = zeros(0, 3);

    for edgeIndex = 1:size(denseFixed_km, 1) - 1
        point1Fixed_km = denseFixed_km(edgeIndex, :);
        visible1 = isVisible(edgeIndex);
        visible2 = isVisible(edgeIndex + 1);

        if visible1
            pointIsNew = isempty(visibleFixed_km) || ...
                any(isnan(visibleFixed_km(end, :))) || ...
                norm(visibleFixed_km(end, :) - point1Fixed_km) > 1e-9;
            if pointIsNew
                visibleFixed_km(end + 1, :) = point1Fixed_km; %#ok<AGROW>
            end
        end

        if visible1 ~= visible2
            point1_deg = denseLatLon_deg(edgeIndex, :);
            delta_deg = denseLatLon_deg(edgeIndex + 1, :) - point1_deg;
            delta_deg(2) = mod(delta_deg(2) + 180, 360) - 180;
            low = 0;
            high = 1;
            lowValue = visibility(edgeIndex);

            % Twenty bisections locate the ellipsoidal horizon crossing well
            % below the angular resolution introduced by boundary densifying.
            for iteration = 1:20
                middle = (low + high) / 2;
                trial_deg = point1_deg + middle * delta_deg;
                trial_deg(2) = mod(trial_deg(2) + 180, 360) - 180;
                trialFixed_km = lla2ecef([trial_deg, 0]) / 1000;
                trialNormal = [ ...
                    trialFixed_km(1) / earthA_km^2, ...
                    trialFixed_km(2) / earthA_km^2, ...
                    trialFixed_km(3) / earthB_km^2];
                middleValue = sum(trialNormal .* ...
                    (sensorPosition_km - trialFixed_km));

                if sign(middleValue) == sign(lowValue)
                    low = middle;
                    lowValue = middleValue;
                else
                    high = middle;
                end
            end

            crossing_deg = point1_deg + ((low + high) / 2) * delta_deg;
            crossing_deg(2) = mod(crossing_deg(2) + 180, 360) - 180;
            crossingFixed_km = lla2ecef([crossing_deg, 0]) / 1000;
            visibleFixed_km(end + 1, :) = crossingFixed_km; %#ok<AGROW>

            if visible1
                visibleFixed_km(end + 1, :) = NaN(1, 3); %#ok<AGROW>
            end
        end
    end

    if isVisible(end)
        visibleFixed_km(end + 1, :) = denseFixed_km(end, :); %#ok<AGROW>
    end
    while ~isempty(visibleFixed_km) && ...
            any(isnan(visibleFixed_km(end, :)))
        visibleFixed_km(end, :) = [];
    end

    currentAz_deg = zeros(0, 1);
    currentEl_deg = zeros(0, 1);
    currentStatus = "below_horizon";

    if ~isempty(visibleFixed_km)
        validPoint = all(isfinite(visibleFixed_km), 2);
        sensorDirectionFixed = ...
            visibleFixed_km(validPoint, :) - sensorPosition_km;
        sensorDirectionFixed = sensorDirectionFixed ./ ...
            vecnorm(sensorDirectionFixed, 2, 2);
        sensorDirection = ...
            (CfixedToSensorNow * sensorDirectionFixed.').';
        outputCount = size(visibleFixed_km, 1);

        localAz_deg = mod( ...
            atan2d(sensorDirection(:, 2), sensorDirection(:, 1)) + 180, ...
            360) - 180;
        localEl_deg = atan2d( ...
            sensorDirection(:, 3), ...
            hypot(sensorDirection(:, 1), sensorDirection(:, 2)));

        % The repository supports the full [-90, 90] degree elevation range.
        % A sign test on elevation would incorrectly discard valid geometry.
        currentAz_deg = NaN(outputCount, 1);
        currentEl_deg = NaN(outputCount, 1);
        validIndex = find(validPoint);
        currentAz_deg(validIndex) = localAz_deg;
        currentEl_deg(validIndex) = localEl_deg;

        % Break the line rather than connecting +180 to -180 degrees.
        adjacent = isfinite(currentAz_deg(1:end - 1)) & ...
            isfinite(currentAz_deg(2:end));
        wrapBreak = find( ...
            adjacent & abs(diff(currentAz_deg)) > 180) + 1;
        currentAz_deg(wrapBreak) = NaN;
        currentEl_deg(wrapBreak) = NaN;

        if any(isfinite(currentAz_deg))
            currentStatus = "visible";
        else
            currentStatus = "projection_invalid";
        end
    end

    az_deg{timeIndex} = currentAz_deg;
    el_deg{timeIndex} = currentEl_deg;
    status(timeIndex) = currentStatus;
end

%% Section 4: Assemble The Canonical Obstacle Record

azElData = obstacleAvoidance.obstacles.createObstacle(struct( ...
    "targetName", targetName, ...
    "time_s", time_s, ...
    "x_units", {az_deg}, ...
    "y_units", {el_deg}, ...
    "status", status));
end
