%% Section 0: Header & Readme
% SYNTAX
%   run("scratch/visualizeFortyCircleObstacleField3D.m")
%**************************************************************************
% PURPOSE
%   - Build the unchanged 40-circle moving obstacle field without running
%     the planner.
%   - Open interactive azimuth/elevation/time and snapshot views.
%**************************************************************************
% INPUTS
%   - None. Adjust the display controls in Section 1 before running.
%**************************************************************************
% OUTPUTS
%   - obstacleField (scalar packed obstacle-field struct)
%   - visualization (scalar graphics-handle struct)
%       Variables remain in the MATLAB workspace for inspection.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************

%% Section 1: Set Display Controls

displayEndTime_s = 60;     % Use 60 for motion detail or 200 for all time.
showSweptSurfaces = true;
viewAzimuth_deg = 42;
viewElevation_deg = 24;

validateattributes(displayEndTime_s, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive', '<=', 200});

%% Section 2: Build The Forty Moving Circles

repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);

missionEndTime_s = 200;
obstacleTime_s = [0; 15; 30; 45; 60; missionEndTime_s];
verticalTranslation_deg = [0; 1.5; 0; -1.5; 0; 0];

gridSpacing_deg = 8;
columnAzimuth_deg = (-3.5:3.5) * gridSpacing_deg;
rowElevation_deg = (-2:2).' * gridSpacing_deg;

circleRadius_deg = 3;
safetyMargin_deg = 0.10;
circleVertexCount = 16;
circleAngle_rad = (0:circleVertexCount - 1).' * ...
    (2 * pi / circleVertexCount);
unitCircle = [cos(circleAngle_rad), sin(circleAngle_rad)];

obstacleCount = numel(columnAzimuth_deg) * numel(rowElevation_deg);
obstacleByIndex = cell(obstacleCount, 1);
obstacleIndex = 0;

for rowIndex = 1:numel(rowElevation_deg)
    for columnIndex = 1:numel(columnAzimuth_deg)
        obstacleIndex = obstacleIndex + 1;
        obstacleAzimuthByTime_deg = cell(numel(obstacleTime_s), 1);
        obstacleElevationByTime_deg = cell(numel(obstacleTime_s), 1);

        for sampleIndex = 1:numel(obstacleTime_s)
            center_deg = [ ...
                columnAzimuth_deg(columnIndex), ...
                rowElevation_deg(rowIndex) + ...
                    verticalTranslation_deg(sampleIndex)];
            circlePosition_deg = ...
                center_deg + circleRadius_deg * unitCircle;
            obstacleAzimuthByTime_deg{sampleIndex} = ...
                circlePosition_deg(:, 1);
            obstacleElevationByTime_deg{sampleIndex} = ...
                circlePosition_deg(:, 2);
        end

        obstacleByIndex{obstacleIndex} = makeAzElObstacleData( ...
            "Moving grid circle " + obstacleIndex, ...
            obstacleTime_s, ...
            obstacleAzimuthByTime_deg, ...
            obstacleElevationByTime_deg, ...
            safetyMargin_deg);
    end
end

obstacles = combineAzElObstacles(obstacleByIndex{:});
obstacleField = buildAzElTimeObstacleField(obstacles);

%% Section 3: Draw The Interactive Three-Dimensional Field

% The animation function expects a timed path. This dashed diagonal is a
% visual reference only; the planner is not called and no route is implied.
referencePath = struct( ...
    "Success", true, ...
    "time_s", [0; displayEndTime_s], ...
    "position_deg", [-36 -24; 36 24], ...
    "velocity_deg_s", zeros(2, 2));

visualization = animateAzElTimedSlopePath( ...
    referencePath, obstacleField, struct( ...
    "FigureVisible", "on", ...
    "FrameStride", 2, ...
    "Pause_s", 0, ...
    "ShowSweptSurfaces", showSweptSurfaces, ...
    "MaximumDisplayedSlicesPerObstacle", 6, ...
    "ObstacleFaceAlpha", 0.06, ...
    "SweptSurfaceAlpha", 0.10, ...
    "Title", "40-circle moving obstacle field"));

delete(visualization.Trail3D);
delete(visualization.Current3D);
delete(visualization.SlopeArrow3D);

axes3D = visualization.Axes3D;
referenceHandle = findobj( ...
    axes3D, "DisplayName", "Complete timed path");
set(referenceHandle, ...
    "Color", [0.05 0.05 0.05], ...
    "LineStyle", "--", ...
    "LineWidth", 2, ...
    "DisplayName", "Straight reference (not planned)");

xlim(axes3D, [-40 40]);
ylim(axes3D, [-26 26]);
zlim(axes3D, [0 displayEndTime_s]);
set(axes3D, ...
    "DataAspectRatioMode", "auto", ...
    "PlotBoxAspectRatio", [1.5 1 0.9]);
view(axes3D, viewAzimuth_deg, viewElevation_deg);
title(axes3D, sprintf( ...
    "40 moving circles, 0-%.0f s", displayEndTime_s));
legend(axes3D, referenceHandle, "Location", "northeast");
rotate3d(visualization.Figure, "on");
drawnow;

fprintf("Created the 3-D obstacle field for 0-%.0f s.\n", ...
    displayEndTime_s);
fprintf("Drag the 3-D axes to rotate. Set displayEndTime_s to 200 " + ...
    "to show the full static tail.\n");
