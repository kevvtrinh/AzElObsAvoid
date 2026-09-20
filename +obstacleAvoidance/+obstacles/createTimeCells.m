function cells = createTimeCells(obstacles, initialTime_s, finalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   cells = obstacleAvoidance.obstacles.createTimeCells( ...
%       obstacles, initialTime_s, finalTime_s)
%**************************************************************************
% PURPOSE
%   - Prepare convex exclusion cells with affine vertex motion.
%**************************************************************************
% INPUTS
%   - obstacles (canonical or prepared obstacle array)
%       Static or time-varying obstacle histories. Raw histories are
%       prepared here; already-prepared histories are reused.
%   - initialTime_s (numeric scalar)
%       Planning-horizon start time.
%   - finalTime_s (numeric scalar)
%       Planning-horizon end time, not earlier than initialTime_s.
%**************************************************************************
% OUTPUTS
%   - cells (scalar struct)
%       Regions, active intervals, source indices, and event times. Invalid
%       input or unsupported continuous geometry throws an error.
%**************************************************************************
% UNITS
%   - Geometry uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Cover Every Active Source Interval

obstacles        = obstacleAvoidance.obstacles.prepareObstacles(obstacles, [initialTime_s, finalTime_s]);
regions_units    = cell(0, 1);
endRegions_units = cell(0, 1);
intervals_s      = zeros(0, 2);
sourceIndices    = zeros(0, 1);
breakTimes_s     = [initialTime_s; finalTime_s];
for obstacleIndex = 1:numel(obstacles)
    obstacle    = obstacles(obstacleIndex);
    preparation = obstacle.InternalPreparation;
    if isscalar(obstacle.time_s)
        sourceIntervals_s = [initialTime_s, finalTime_s];
    else
        sourceIntervals_s = [obstacle.time_s(1:end - 1), obstacle.time_s(2:end)];
    end
    spanStarts = 1:size(sourceIntervals_s, 1);
    if ~isscalar(obstacle.time_s)
        spanStarts = unique(preparation.SpanStartSampleIndex).';
    end
    for firstIntervalIndex = spanStarts
        lastIntervalIndex = firstIntervalIndex;
        if ~isscalar(obstacle.time_s)
            lastIntervalIndex = preparation.SpanEndSampleIndex(firstIntervalIndex) - 1;
            sourceIntervals_s(firstIntervalIndex, 2) = obstacle.time_s(lastIntervalIndex + 1);
        end
        activeInterval_s = [max(initialTime_s, sourceIntervals_s(firstIntervalIndex, 1)), ...
            min(finalTime_s, sourceIntervals_s(firstIntervalIndex, 2))];
        if activeInterval_s(1) >= activeInterval_s(2)
            continue;
        end
        if isscalar(obstacle.time_s) || preparation.IsTimeInvariant
            shape                 = preparation.SampleShapes{firstIntervalIndex};
            intervalRegions_units = obstacleAvoidance.geometry.convexRegions(shape);
            intervalEndRegions_units = intervalRegions_units;
        elseif preparation.IntervalUsesMovingCells(firstIntervalIndex)
            intervalRegions_units    = preparation.IntervalStartRegions_units{firstIntervalIndex};
            intervalEndRegions_units = intervalRegions_units;
        elseif preparation.IntervalUsesEndpointHull(firstIntervalIndex)
            intervalRegions_units    = preparation.IntervalStartRegions_units{firstIntervalIndex};
            intervalEndRegions_units = intervalRegions_units;
            addedArea_units2         = preparation.IntervalEndpointHullAddedArea_units2(firstIntervalIndex);
            if ~isfinite(addedArea_units2) || addedArea_units2 < 0
                error('createTimeCells:InvalidEndpointHullDiagnostic', ...
                    'The endpoint hull added-area diagnostic must be finite and nonnegative.');
            end
        elseif preparation.MatchingTopology(firstIntervalIndex) && ...
                preparation.IntervalSpeedBound_units_s(firstIntervalIndex) == 0
            % A history can change elsewhere while this interval remains
            % stationary. Preserve its cavities and disconnected components.
            intervalRegions_units = obstacleAvoidance.geometry.convexRegions( ...
                preparation.SampleShapes{firstIntervalIndex});
            intervalEndRegions_units = intervalRegions_units;
        elseif preparation.MatchingTopology(firstIntervalIndex) && ...
                preparation.IntervalHasExactPartition(firstIntervalIndex)
            % Each stored face is convex for the complete linear morph, and
            % the moving union equals the authoritative concave polygon.
            fraction = (activeInterval_s - sourceIntervals_s(firstIntervalIndex, 1)) / ...
                diff(sourceIntervals_s(firstIntervalIndex, :));
            startRegions_units  = preparation.IntervalStartRegions_units{firstIntervalIndex};
            finishRegions_units = preparation.IntervalEndRegions_units{lastIntervalIndex};
            intervalRegions_units    = cell(size(startRegions_units));
            intervalEndRegions_units = cell(size(startRegions_units));
            for regionIndex = 1:numel(startRegions_units)
                delta_units = finishRegions_units{regionIndex} - startRegions_units{regionIndex};
                intervalRegions_units{regionIndex} = startRegions_units{regionIndex} + fraction(1) * delta_units;
                intervalEndRegions_units{regionIndex} = startRegions_units{regionIndex} + fraction(2) * delta_units;
            end
        elseif preparation.MatchingTopology(firstIntervalIndex)
            % A verified convex boundary stays convex throughout the linear
            % vertex interpolation.
            lower_units = [obstacle.x_units{firstIntervalIndex}, obstacle.y_units{firstIntervalIndex}];
            delta_units = [obstacle.x_units{lastIntervalIndex + 1}, ...
                obstacle.y_units{lastIntervalIndex + 1}] - lower_units;
            if lastIntervalIndex == firstIntervalIndex
                delta_units = [preparation.DeltaX_units{firstIntervalIndex}, ...
                    preparation.DeltaY_units{firstIntervalIndex}];
            end
            fraction = (activeInterval_s - sourceIntervals_s(firstIntervalIndex, 1)) / ...
                diff(sourceIntervals_s(firstIntervalIndex, :));
            intervalRegions_units    = {lower_units + fraction(1) * delta_units};
            intervalEndRegions_units = {lower_units + fraction(2) * delta_units};
        elseif preparation.IntervalIsUnsupported(firstIntervalIndex)
            error('createTimeCells:UnsupportedContinuousDeformation', ...
                'The obstacle interval has no verified exact continuous geometry model.');
        elseif preparation.IntervalIsStationary(firstIntervalIndex)
            shape                    = preparation.IntervalUnionShapes{firstIntervalIndex};
            intervalRegions_units    = obstacleAvoidance.geometry.convexRegions(shape);
            intervalEndRegions_units = intervalRegions_units;
        else
            error('createTimeCells:UnknownGeometryModel', ...
                'The prepared obstacle interval has an unknown geometry model.');
        end
        regionCount      = numel(intervalRegions_units);
        regions_units    = [regions_units; intervalRegions_units]; %#ok<AGROW>
        endRegions_units = [endRegions_units; intervalEndRegions_units]; %#ok<AGROW>
        intervals_s      = [intervals_s; repmat(activeInterval_s, regionCount, 1)]; %#ok<AGROW>
        sourceIndices    = [sourceIndices; repmat(obstacleIndex, regionCount, 1)]; %#ok<AGROW>
        breakTimes_s     = [breakTimes_s; activeInterval_s(:)]; %#ok<AGROW>
    end
end
cells = struct( ...
    'Regions_units',          {regions_units}, ...
    'ActiveTimeInterval_s',   intervals_s, ...
    'EndRegions_units',       {endRegions_units}, ...
    'SourceObstacleIndex',    sourceIndices, ...
    'BreakTime_s',            unique(breakTimes_s));
end
