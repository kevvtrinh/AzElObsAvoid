function timedRegions = createTimeCells(obstacles, initialTime_s, finalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   timedRegions = obstacleAvoidance.obstacles.createTimeCells( ...
%       obstacles, initialTime_s, finalTime_s)
%**************************************************************************
% PURPOSE
%   - Collect convex obstacle regions with their active time intervals and
%     start/end vertex positions. A moving region's vertices travel along
%     straight lines between those positions; a stationary region uses the
%     same positions at both ends.
%**************************************************************************
% INPUTS
%   - obstacles (canonical or prepared obstacle array)
%       Static or time-varying obstacle histories. Raw histories are
%       prepared here; already-prepared histories are reused.
%   - initialTime_s (numeric scalar)
%       Start of the time range needed for planning.
%   - finalTime_s (numeric scalar)
%       End of the planning time range, not earlier than initialTime_s.
%**************************************************************************
% OUTPUTS
%   - timedRegions (scalar struct)
%       Regions_units and EndRegions_units hold each region's start and end
%       vertices. ActiveTimeInterval_s gives its [start end] times, and
%       SourceObstacleIndex identifies its obstacle. BreakTime_s lists the
%       interval boundaries. Invalid input or an unsupported model throws.
%**************************************************************************
% UNITS
%   - Geometry uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Prepare Obstacles For The Requested Time Range

preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles( ...
    obstacles, [initialTime_s, finalTime_s]);

allStartRegions_units = cell(0, 1);
allEndRegions_units   = cell(0, 1);
activeIntervals_s     = zeros(0, 2);
sourceObstacleIndices = zeros(0, 1);
eventTimes_s          = [initialTime_s; finalTime_s];

%% Section 2: Build Regions From Each Verified Motion Model

for obstacleIndex = 1:numel(preparedObstacles)
    obstacle            = preparedObstacles(obstacleIndex);
    obstaclePreparation = obstacle.InternalPreparation;
    if isscalar(obstacle.time_s)
        recordedIntervals_s = [initialTime_s, finalTime_s];
    else
        recordedIntervals_s = [obstacle.time_s(1:end - 1), obstacle.time_s(2:end)];
    end

    % Preparation may combine adjacent intervals with the same vertex motion.
    % Process each combined span once, using its first and last sample times.
    spanStartIndices = 1:size(recordedIntervals_s, 1);
    if ~isscalar(obstacle.time_s)
        spanStartIndices = unique(obstaclePreparation.SpanStartSampleIndex).';
    end
    for firstIntervalIndex = spanStartIndices
        lastIntervalIndex = firstIntervalIndex;
        if ~isscalar(obstacle.time_s)
            lastIntervalIndex = obstaclePreparation.SpanEndSampleIndex(firstIntervalIndex) - 1;
            recordedIntervals_s(firstIntervalIndex, 2) = obstacle.time_s(lastIntervalIndex + 1);
        end

        % Keep only the time shared by this obstacle span and the request.
        % A span ending before the request or starting after it adds no regions.
        activeInterval_s = [max(initialTime_s, recordedIntervals_s(firstIntervalIndex, 1)), ...
            min(finalTime_s, recordedIntervals_s(firstIntervalIndex, 2))];
        if activeInterval_s(1) >= activeInterval_s(2)
            continue;
        end
        if isscalar(obstacle.time_s) || obstaclePreparation.IsTimeInvariant
            obstacleShape = obstaclePreparation.SampleShapes{firstIntervalIndex};
            intervalStartRegions_units = obstacleAvoidance.geometry.convexRegions(obstacleShape);
            intervalEndRegions_units   = intervalStartRegions_units;
        elseif obstaclePreparation.IntervalUsesMovingCells(firstIntervalIndex)
            % This enclosure covers the whole span and stays fixed in space.
            intervalStartRegions_units = obstaclePreparation.IntervalStartRegions_units{firstIntervalIndex};
            intervalEndRegions_units   = intervalStartRegions_units;
        elseif obstaclePreparation.IntervalUsesEndpointHull(firstIntervalIndex)
            % The hull of both endpoint shapes is occupied for the whole span.
            intervalStartRegions_units = obstaclePreparation.IntervalStartRegions_units{firstIntervalIndex};
            intervalEndRegions_units   = intervalStartRegions_units;
            addedArea_units2           = obstaclePreparation.IntervalEndpointHullAddedArea_units2(firstIntervalIndex);
            if ~isfinite(addedArea_units2) || addedArea_units2 < 0
                error('createTimeCells:InvalidEndpointHullDiagnostic', ...
                    'The endpoint hull added-area diagnostic must be finite and nonnegative.');
            end
        elseif obstaclePreparation.MatchingTopology(firstIntervalIndex) && ...
                obstaclePreparation.IntervalSpeedBound_units_s(firstIntervalIndex) == 0
            % This interval can be stationary even if the obstacle moves at
            % other times. Keep its actual holes and separate pieces.
            intervalStartRegions_units = obstacleAvoidance.geometry.convexRegions( ...
                obstaclePreparation.SampleShapes{firstIntervalIndex});
            intervalEndRegions_units = intervalStartRegions_units;
        elseif obstaclePreparation.MatchingTopology(firstIntervalIndex) && ...
                obstaclePreparation.IntervalHasExactPartition(firstIntervalIndex)
            % These pieces were verified for the complete recorded span.
            % Move them to the request's start/end within it: times [2 6] in
            % a [0 10] span use fractions [0.2 0.6] of the vertex movement.
            activeIntervalFractions = (activeInterval_s - recordedIntervals_s(firstIntervalIndex, 1)) / ...
                diff(recordedIntervals_s(firstIntervalIndex, :));
            spanStartRegions_units     = obstaclePreparation.IntervalStartRegions_units{firstIntervalIndex};
            spanEndRegions_units       = obstaclePreparation.IntervalEndRegions_units{lastIntervalIndex};
            intervalStartRegions_units = cell(size(spanStartRegions_units));
            intervalEndRegions_units   = cell(size(spanStartRegions_units));
            for regionIndex = 1:numel(spanStartRegions_units)
                vertexDisplacement_units = spanEndRegions_units{regionIndex} - spanStartRegions_units{regionIndex};
                intervalStartRegions_units{regionIndex} = spanStartRegions_units{regionIndex} + ...
                    activeIntervalFractions(1) * vertexDisplacement_units;
                intervalEndRegions_units{regionIndex} = spanStartRegions_units{regionIndex} + ...
                    activeIntervalFractions(2) * vertexDisplacement_units;
            end
        elseif obstaclePreparation.MatchingTopology(firstIntervalIndex)
            % The complete boundary was verified to stay convex, so one
            % region is enough. Place it at this request's active start/end.
            spanStartVertices_units   = [obstacle.x_units{firstIntervalIndex}, obstacle.y_units{firstIntervalIndex}];
            vertexDisplacement_units = [obstacle.x_units{lastIntervalIndex + 1}, ...
                obstacle.y_units{lastIntervalIndex + 1}] - spanStartVertices_units;
            if lastIntervalIndex == firstIntervalIndex
                vertexDisplacement_units = [obstaclePreparation.DeltaX_units{firstIntervalIndex}, ...
                    obstaclePreparation.DeltaY_units{firstIntervalIndex}];
            end
            activeIntervalFractions = (activeInterval_s - recordedIntervals_s(firstIntervalIndex, 1)) / ...
                diff(recordedIntervals_s(firstIntervalIndex, :));
            intervalStartRegions_units = { ...
                spanStartVertices_units + activeIntervalFractions(1) * vertexDisplacement_units};
            intervalEndRegions_units = { ...
                spanStartVertices_units + activeIntervalFractions(2) * vertexDisplacement_units};
        elseif obstaclePreparation.IntervalIsUnsupported(firstIntervalIndex)
            error('createTimeCells:UnsupportedContinuousDeformation', ...
                'The obstacle interval has no verified exact continuous geometry model.');
        elseif obstaclePreparation.IntervalIsStationary(firstIntervalIndex)
            obstacleShape = obstaclePreparation.IntervalUnionShapes{firstIntervalIndex};
            intervalStartRegions_units = obstacleAvoidance.geometry.convexRegions(obstacleShape);
            intervalEndRegions_units   = intervalStartRegions_units;
        else
            error('createTimeCells:UnknownGeometryModel', ...
                'The prepared obstacle interval has an unknown geometry model.');
        end

        % Keep one time interval and source obstacle index per region so
        % later collision checks can select the right geometry for each time.
        regionCount           = numel(intervalStartRegions_units);
        allStartRegions_units = [allStartRegions_units; intervalStartRegions_units]; %#ok<AGROW>
        allEndRegions_units   = [allEndRegions_units; intervalEndRegions_units]; %#ok<AGROW>
        activeIntervals_s     = [activeIntervals_s; repmat(activeInterval_s, regionCount, 1)]; %#ok<AGROW>
        sourceObstacleIndices = [sourceObstacleIndices; repmat(obstacleIndex, regionCount, 1)]; %#ok<AGROW>
        eventTimes_s          = [eventTimes_s; activeInterval_s(:)]; %#ok<AGROW>
    end
end

%% Section 3: Return Regions And Their Event Times

timedRegions = struct( ...
    'Regions_units',          {allStartRegions_units}, ...
    'ActiveTimeInterval_s',   activeIntervals_s, ...
    'EndRegions_units',       {allEndRegions_units}, ...
    'SourceObstacleIndex',    sourceObstacleIndices, ...
    'BreakTime_s',            unique(eventTimes_s));
end
