function cells = createTimeCells(obstacles, initialTime_s, finalTime_s)
%% Section 0: Header & Readme
% SYNTAX: cells = obstacleAvoidance.obstacles.createTimeCells(obstacles,t0,t1)
% PURPOSE: Prepare one convex space-time exclusion representation. Static
%          regions and dynamic histories differ only in their activity times.
% INPUTS: Prepared authoritative obstacle histories and physical horizon.
% OUTPUTS: Convex regions, absolute active intervals, source IDs, event knots.
% UNITS: Coordinate units and seconds.

%% Section 1: Cover Every Active Source Interval
regions_units = cell(0,1); intervals_s = zeros(0,2); sources = zeros(0,1);
breaks_s = [initialTime_s;finalTime_s];
for k = 1:numel(obstacles)
    obstacle = obstacles(k); preparation = obstacle.InternalPreparation;
    if isscalar(obstacle.time_s)
        sourceIntervals_s = [initialTime_s,finalTime_s];
    else
        sourceIntervals_s = [obstacle.time_s(1:end-1),obstacle.time_s(2:end)];
    end
    for j = 1:size(sourceIntervals_s,1)
        active_s = [max(initialTime_s,sourceIntervals_s(j,1)),min(finalTime_s,sourceIntervals_s(j,2))];
        if active_s(1) >= active_s(2), continue; end
        if isscalar(obstacle.time_s) || preparation.IsTimeInvariant
            shape = preparation.SampleShapes{1};
        elseif preparation.MatchingTopology(j)
            % Endpoint hull encloses every admitted corresponding-vertex
            % interpolation. It is an explicit conservative solver enclosure.
            vertices_units = [obstacle.x_units{j},obstacle.y_units{j}; ...
                obstacle.x_units{j+1},obstacle.y_units{j+1}];
            vertices_units = vertices_units(all(isfinite(vertices_units),2),:);
            hull = convhull(vertices_units(:,1),vertices_units(:,2));
            shape = polyshape(vertices_units(hull(1:end-1),:),'Simplify',false);
        else
            shape = preparation.IntervalUnionShapes{j};
        end
        regions = obstacleAvoidance.geometry.convexRegions(shape);
        regions_units = [regions_units;regions]; %#ok<AGROW>
        intervals_s = [intervals_s;repmat(active_s,numel(regions),1)]; %#ok<AGROW>
        sources = [sources;repmat(k,numel(regions),1)]; %#ok<AGROW>
        breaks_s = [breaks_s;active_s(:)]; %#ok<AGROW>
    end
end
cells = struct('Regions_units',{regions_units},'ActiveTimeInterval_s',intervals_s, ...
    'SourceObstacleIndex',sources,'BreakTime_s',unique(breaks_s));
end
