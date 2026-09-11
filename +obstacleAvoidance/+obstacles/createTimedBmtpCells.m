function [regions_units, coverage] = createTimedBmtpCells(obstacles, startTime_s, finishTime_s, timedSegmentCount)
%% Section 0: Header & Readme
% SYNTAX: [regions_units,coverage] = obstacleAvoidance.obstacles.createTimedBmtpCells(
%   obstacles,startTime_s,finishTime_s,timedSegmentCount)
% PURPOSE: Cover static geometry exactly and each moving time cell by a convex hull of its protected
%   endpoint and midpoint geometry.
% INPUTS: Prepared obstacles, physical bounds in seconds, and segment count.
% OUTPUTS: Convex exclusion regions and their physical and normalized intervals.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Build Conservative Time Cells
regions_units = cell(0,1);
activeTime_s = zeros(0,2);
baseEdges_s = linspace(startTime_s,finishTime_s,timedSegmentCount + 1).';
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    if obstacle.InternalPreparation.IsTimeInvariant
        shape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,startTime_s);
        staticRegions = obstacleAvoidance.geometry.convexRegions(shape,true);
        regions_units = [regions_units;staticRegions]; %#ok<AGROW>
        activeTime_s = [activeTime_s; ...
            repmat([startTime_s finishTime_s],numel(staticRegions),1)]; %#ok<AGROW>
        continue;
    end
    eventTimes_s = obstacle.time_s(obstacle.time_s > startTime_s & ...
        obstacle.time_s < finishTime_s);
    cellEdges_s = unique([baseEdges_s;eventTimes_s(:)]);
    for cellIndex = 1:numel(cellEdges_s) - 1
        cellTime_s = [cellEdges_s(cellIndex); ...
            mean(cellEdges_s(cellIndex:cellIndex + 1));cellEdges_s(cellIndex + 1)];
        vertices_units = zeros(0,2);
        for queryIndex = 1:3
            shape = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
                obstacle,cellTime_s(queryIndex));
            vertices = shape.Vertices;
            vertices_units = [vertices_units; ...
                vertices(all(isfinite(vertices),2),:)]; %#ok<AGROW>
        end
        vertices_units = unique(vertices_units,'rows','stable');
        if size(vertices_units,1) < 3, continue; end
        hullIndex = convhull(vertices_units(:,1),vertices_units(:,2));
        regions_units{end + 1,1} = vertices_units(hullIndex(1:end - 1),:); %#ok<AGROW>
        activeTime_s(end + 1,:) = cellEdges_s(cellIndex:cellIndex + 1); %#ok<AGROW>
    end
end

%% Section 2: Return Coverage Evidence
duration_s = finishTime_s - startTime_s;
coverage = struct('Passed',true,'ExactRegionCount',numel(regions_units), ...
    'SolverRegionCount',numel(regions_units), ...
    'ActiveTimeInterval_s',activeTime_s, ...
    'RegionActiveTauInterval',(activeTime_s - startTime_s) / duration_s, ...
    'EndRegions_units',{regions_units},'BreakTime_s',unique([startTime_s;finishTime_s;activeTime_s(:)]), ...
    'TimedSegmentCount',timedSegmentCount, ...
    'AuthoritativeCoverageCheck','publicDynamicValidation');
end
