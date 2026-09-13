function cells = createTimeCells(obstacles, initialTime_s, finalTime_s, longestSharedEdgeFirst)
%% Section 0: Header & Readme
% SYNTAX: cells = obstacleAvoidance.obstacles.createTimeCells(obstacles,t0,t1)
%         cells = obstacleAvoidance.obstacles.createTimeCells(obstacles,t0,t1,true)
% PURPOSE: Prepare convex exclusion cells with affine vertex motion in
%          absolute time; static cells have equal endpoint vertices.
% INPUTS: Prepared authoritative obstacle histories, physical horizon, and
%         optional deterministic merge ordering.
% OUTPUTS: Convex regions, absolute active intervals, source IDs, event knots.
% UNITS: Coordinate units and seconds.

%% Section 1: Cover Every Active Source Interval
if nargin < 4, longestSharedEdgeFirst = false; end
obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles,[initialTime_s,finalTime_s]);
regions_units = cell(0,1); endRegions_units = cell(0,1);
intervals_s = zeros(0,2); sources = zeros(0,1);
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
            shape = preparation.SampleShapes{j};
            regions = obstacleAvoidance.geometry.convexRegions(shape,longestSharedEdgeFirst);
            endRegions = regions;
        elseif preparation.MatchingTopology(j) && preparation.IntervalSpeedBound_units_s(j)==0
            % A history can change elsewhere while this interval remains
            % stationary. Preserve its cavities and disconnected components.
            regions = obstacleAvoidance.geometry.convexRegions( ...
                preparation.SampleShapes{j},longestSharedEdgeFirst);
            endRegions = regions;
        elseif preparation.MatchingTopology(j) && ...
                preparation.IntervalGeometryModel(j)=="linearCorrespondingConvexPartition"
            % Each stored face is convex for the complete linear morph, and
            % the moving union equals the authoritative concave polygon.
            fraction = (active_s-sourceIntervals_s(j,1))/diff(sourceIntervals_s(j,:));
            startRegions = preparation.IntervalStartRegions_units{j};
            finishRegions = preparation.IntervalEndRegions_units{j};
            regions = cell(size(startRegions));
            endRegions = cell(size(startRegions));
            for regionIndex = 1:numel(startRegions)
                delta_units = finishRegions{regionIndex}-startRegions{regionIndex};
                regions{regionIndex} = startRegions{regionIndex}+fraction(1)*delta_units;
                endRegions{regionIndex} = startRegions{regionIndex}+fraction(2)*delta_units;
            end
        elseif preparation.MatchingTopology(j)
            % A verified convex boundary stays convex throughout the linear
            % vertex interpolation.
            lower_units = [obstacle.x_units{j},obstacle.y_units{j}];
            delta_units = [preparation.DeltaX_units{j},preparation.DeltaY_units{j}];
            fraction = (active_s-sourceIntervals_s(j,1))/diff(sourceIntervals_s(j,:));
            regions = {lower_units+fraction(1)*delta_units};
            endRegions = {lower_units+fraction(2)*delta_units};
        elseif preparation.IntervalGeometryModel(j)=="unsupportedContinuousDeformation"
            error('createTimeCells:UnsupportedContinuousDeformation', ...
                'The obstacle interval has no verified exact continuous geometry model.');
        else
            shape = preparation.IntervalUnionShapes{j};
            regions = obstacleAvoidance.geometry.convexRegions(shape,longestSharedEdgeFirst);
            endRegions = regions;
        end
        regions_units = [regions_units;regions]; %#ok<AGROW>
        endRegions_units = [endRegions_units;endRegions]; %#ok<AGROW>
        intervals_s = [intervals_s;repmat(active_s,numel(regions),1)]; %#ok<AGROW>
        sources = [sources;repmat(k,numel(regions),1)]; %#ok<AGROW>
        breaks_s = [breaks_s;active_s(:)]; %#ok<AGROW>
    end
end
cells = struct('Regions_units',{regions_units},'ActiveTimeInterval_s',intervals_s, ...
    'EndRegions_units',{endRegions_units},'SourceObstacleIndex',sources,'BreakTime_s',unique(breaks_s));
end
