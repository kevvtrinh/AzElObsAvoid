function [figureHandle, axesHandle] = createSphereView(result, obstacleShapes, snapshotTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [figureHandle, axesHandle] = obstacleAvoidance.plotting.createSphereView( ...
%       result, obstacleShapes, snapshotTime_s, options)
%**************************************************************************
% PURPOSE
%   - Draw an azimuth/elevation trajectory and protected obstacle snapshots
%     on the same unit sphere. Unwrapped positions map through the pole
%     without splitting the returned motion or duplicating obstacle copies.
%**************************************************************************
% INPUTS
%   - result (scalar planner result)
%       Returned motion and endpoint positions in [azimuth elevation] degrees.
%   - obstacleShapes (cell array of polyshape)
%       One protected obstacle snapshot per source obstacle; empty for an
%       unavailable or absent obstacle at snapshotTime_s.
%   - snapshotTime_s (scalar)
%       Time of the displayed obstacle shapes.
%   - options (scalar struct)
%       Plot options resolved by plotTrajectory.
%**************************************************************************
% OUTPUTS
%   - figureHandle, axesHandle (MATLAB graphics handles)
%       The sphere figure and its axes.
%**************************************************************************
% UNITS
%   - Input coordinates use degrees; plotted Cartesian coordinates are on
%     a unit sphere. Small radial offsets keep marks visible above its skin.
%**************************************************************************

%% Section 1: Draw The Unit Sphere

figureHandle = figure("Name", options.Title + " sphere", "Visible", options.FigureVisible);
axesHandle   = axes(figureHandle);
hold(axesHandle, "on");

[sphereX, sphereY, sphereZ] = sphere(48);
surf(axesHandle, sphereX, sphereY, sphereZ, ...
    "FaceColor", [0.82 0.88 0.94], "FaceAlpha", 0.32, ...
    "EdgeColor", "none", "HandleVisibility", "off");
drawAngularGrid(axesHandle);

%% Section 2: Draw Protected Obstacles At One Time

obstacleColors = lines(max(1, numel(obstacleShapes)));
firstObstacleShown = false;
for obstacleIndex = 1:numel(obstacleShapes)
    shape = obstacleShapes{obstacleIndex};
    if isempty(shape) || isempty(shape.Vertices)
        continue
    end

    % Triangulation preserves disconnected regions and holes. Subdivide its
    % faces in azimuth/elevation before projection so a wide triangle follows
    % the curved sphere instead of cutting through it as a flat chord.
    [faceVertices, faces] = createSphericalMesh(shape);
    if isempty(faces)
        continue
    end
    legendVisibility = "off";
    if ~firstObstacleShown
        legendVisibility = "on";
        firstObstacleShown = true;
    end
    patch(axesHandle, "Vertices", faceVertices, "Faces", faces, ...
        "FaceColor", obstacleColors(obstacleIndex, :), "FaceAlpha", 0.82, ...
        "EdgeColor", "none", "DisplayName", "Protected obstacle", ...
        "HandleVisibility", legendVisibility);

    % A denser boundary makes narrow obstacles and holes visible from above.
    [boundaryX_deg, boundaryY_deg] = boundary(shape);
    finitePoint = isfinite(boundaryX_deg) & isfinite(boundaryY_deg);
    runStarts   = find(diff([false; finitePoint; false]) == 1);
    runEnds     = find(diff([false; finitePoint; false]) == -1) - 1;
    for runIndex = 1:numel(runStarts)
        ring_deg = [boundaryX_deg(runStarts(runIndex):runEnds(runIndex)), ...
            boundaryY_deg(runStarts(runIndex):runEnds(runIndex))];
        if size(ring_deg, 1) < 2
            continue
        end
        if any(ring_deg(end, :) ~= ring_deg(1, :))
            ring_deg(end + 1, :) = ring_deg(1, :);
        end
        ring_deg = sampleBoundary(ring_deg, 2);
        ring_xyz = azElToCartesian(ring_deg, 1.018);
        plot3(axesHandle, ring_xyz(:, 1), ring_xyz(:, 2), ring_xyz(:, 3), ...
            "-", "Color", obstacleColors(obstacleIndex, :), ...
            "LineWidth", 1.2, "HandleVisibility", "off");
    end
end

%% Section 3: Draw Returned Motion And Endpoints

if ~isempty(result.position_units)
    motion_xyz = azElToCartesian(result.position_units, 1.026);
    plot3(axesHandle, motion_xyz(:, 1), motion_xyz(:, 2), motion_xyz(:, 3), ...
        "k-", "LineWidth", 2.6, "DisplayName", "Returned motion");
end

start_xyz = azElToCartesian(result.Inputs.initialState.position_units, 1.04);
goal_deg = result.Inputs.goalState.position_units;
if result.Success && ~isempty(result.position_units)
    goal_deg = result.position_units(end, :);
end
goal_xyz = azElToCartesian(goal_deg, 1.04);
plot3(axesHandle, start_xyz(1), start_xyz(2), start_xyz(3), ...
    "go", "MarkerSize", 9, "MarkerFaceColor", "g", "DisplayName", "Start");
plot3(axesHandle, goal_xyz(1), goal_xyz(2), goal_xyz(3), ...
    "ro", "MarkerSize", 9, "MarkerFaceColor", "r", "DisplayName", "Goal");

axis(axesHandle, "equal");
axis(axesHandle, "vis3d");
xlim(axesHandle, [-1.25 1.25]);
ylim(axesHandle, [-1.25 1.25]);
zlim(axesHandle, [-1.25 1.25]);
% Cartesian coordinates place the graphics, but degree labels describe the
% sphere. There are no meaningful azimuth/elevation ticks on X, Y, or Z.
axis(axesHandle, "off");
title(axesHandle, sprintf("%s | azimuth -180 to 180 deg, elevation -90 to 90 deg\nObstacles at t = %.3f s", ...
    options.Title, snapshotTime_s));
view(axesHandle, [35 35]);
legend(axesHandle, "Location", "bestoutside");
end

%% Section 4: Local Functions

function drawAngularGrid(axesHandle)
    % Meridians and latitude circles show angles directly on the sphere.
    % Use signed azimuth labels even for a planner interval of [0 360]:
    % azimuth 190 is the same direction as -170. -180 and +180 share one
    % physical line, so give their common point one label.
    guideColor = [0.57 0.61 0.66];
    azimuthSamples_deg   = linspace(-180, 180, 181).';
    elevationSamples_deg = linspace(-90, 90, 91).';
    for elevation_deg = -60:30:60
        circle_deg = [azimuthSamples_deg, ...
            repmat(elevation_deg, numel(azimuthSamples_deg), 1)];
        circle_xyz = azElToCartesian(circle_deg, 1.005);
        plot3(axesHandle, circle_xyz(:, 1), circle_xyz(:, 2), circle_xyz(:, 3), ...
            ":", "Color", guideColor, "LineWidth", 0.7, "HandleVisibility", "off");
    end
    for azimuth_deg = -180:30:150
        meridian_deg = [repmat(azimuth_deg, numel(elevationSamples_deg), 1), ...
            elevationSamples_deg];
        meridian_xyz = azElToCartesian(meridian_deg, 1.005);
        plot3(axesHandle, meridian_xyz(:, 1), meridian_xyz(:, 2), meridian_xyz(:, 3), ...
            ":", "Color", guideColor, "LineWidth", 0.7, "HandleVisibility", "off");
    end

    labelColor = [0.23 0.28 0.34];
    for azimuth_deg = -180:60:120
        label = sprintf("%d", azimuth_deg);
        if azimuth_deg == -180
            label = "-180 / +180";
        end
        label_xyz = azElToCartesian([azimuth_deg, 0], 1.15);
        text(axesHandle, label_xyz(1), label_xyz(2), label_xyz(3), label, ...
            "HorizontalAlignment", "center", "FontSize", 8, ...
            "Color", labelColor, "HandleVisibility", "off");
    end
    for elevation_deg = -90:30:90
        label_xyz = azElToCartesian([35, elevation_deg], 1.15);
        text(axesHandle, label_xyz(1), label_xyz(2), label_xyz(3), ...
            sprintf("el %+d", elevation_deg), "HorizontalAlignment", "center", ...
            "FontSize", 8, "Color", labelColor, "HandleVisibility", "off");
    end
end

function xyz = azElToCartesian(azEl_deg, radius)
    % Spherical coordinates also accept elevation beyond a pole. For
    % example, (10, 150) and (190, 30) reach the same point on the sphere.
    azimuth_rad   = deg2rad(azEl_deg(:, 1));
    elevation_rad = deg2rad(azEl_deg(:, 2));
    [x, y, z] = sph2cart(azimuth_rad, elevation_rad, ...
        radius * ones(size(azimuth_rad)));
    xyz = [x, y, z];
end

function [vertices_xyz, faces] = createSphericalMesh(shape)
    planarMesh = triangulation(shape);
    triangleCount = size(planarMesh.ConnectivityList, 1);
    subdivisions = zeros(triangleCount, 1);
    for triangleIndex = 1:triangleCount
        corners_deg = planarMesh.Points(planarMesh.ConnectivityList(triangleIndex, :), :);
        edgeLengths_deg = vecnorm([corners_deg(2, :) - corners_deg(1, :); ...
            corners_deg(3, :) - corners_deg(2, :); corners_deg(1, :) - corners_deg(3, :)], 2, 2);
        subdivisions(triangleIndex) = max(1, ceil(max(edgeLengths_deg) / 4));
    end

    vertexCount = sum((subdivisions + 1) .* (subdivisions + 2) / 2);
    faceCount   = sum(subdivisions .^ 2);
    vertices_deg = zeros(vertexCount, 2);
    faces        = zeros(faceCount, 3);
    vertexEnd    = 0;
    faceEnd      = 0;
    for triangleIndex = 1:triangleCount
        corners_deg = planarMesh.Points(planarMesh.ConnectivityList(triangleIndex, :), :);
        steps       = subdivisions(triangleIndex);
        nodeIndices = zeros(steps + 1);
        for firstIndex = 0:steps
            secondIndices = (0:steps - firstIndex).';
            count = numel(secondIndices);
            indices = vertexEnd + (1:count);
            nodeIndices(firstIndex + 1, secondIndices + 1) = indices;
            vertices_deg(indices, :) = corners_deg(1, :) + ...
                (firstIndex / steps) * (corners_deg(2, :) - corners_deg(1, :)) + ...
                (secondIndices / steps) * (corners_deg(3, :) - corners_deg(1, :));
            vertexEnd = vertexEnd + count;
        end
        for firstIndex = 0:steps - 1
            for secondIndex = 0:steps - firstIndex - 1
                faceEnd = faceEnd + 1;
                faces(faceEnd, :) = [nodeIndices(firstIndex + 1, secondIndex + 1), ...
                    nodeIndices(firstIndex + 2, secondIndex + 1), ...
                    nodeIndices(firstIndex + 1, secondIndex + 2)];
                if secondIndex < steps - firstIndex - 1
                    faceEnd = faceEnd + 1;
                    faces(faceEnd, :) = [nodeIndices(firstIndex + 2, secondIndex + 1), ...
                        nodeIndices(firstIndex + 2, secondIndex + 2), ...
                        nodeIndices(firstIndex + 1, secondIndex + 2)];
                end
            end
        end
    end
    vertices_xyz = azElToCartesian(vertices_deg, 1.012);
end

function sampledRing_deg = sampleBoundary(ring_deg, spacing_deg)
    % Interpolate each straight edge in the planner's azimuth/elevation
    % coordinates before bending it onto the sphere.
    sampledRing_deg = zeros(0, 2);
    for edgeIndex = 1:size(ring_deg, 1) - 1
        start_deg = ring_deg(edgeIndex, :);
        finish_deg = ring_deg(edgeIndex + 1, :);
        steps = max(1, ceil(max(abs(finish_deg - start_deg)) / spacing_deg));
        fractions = (0:steps - 1).' / steps;
        sampledRing_deg = [sampledRing_deg; ...
            start_deg + fractions * (finish_deg - start_deg)]; %#ok<AGROW>
    end
    sampledRing_deg(end + 1, :) = ring_deg(end, :);
end
