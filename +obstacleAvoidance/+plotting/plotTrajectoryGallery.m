function figureHandles = plotTrajectoryGallery(results, labels, figureVisible)
%% Section 0: Header & Readme
% SYNTAX
%   figureHandles = obstacleAvoidance.plotting.plotTrajectoryGallery(results)
%   figureHandles = obstacleAvoidance.plotting.plotTrajectoryGallery(results, labels)
%   figureHandles = obstacleAvoidance.plotting.plotTrajectoryGallery(results, labels, figureVisible)
%**************************************************************************
% PURPOSE
%   - Compare returned planner motions in pages of sixteen spatial plots.
%   - Show protected obstacles at the request start, middle, and end. These
%     three pictures illustrate movement; collision safety comes from the
%     independent validator. An interval using a fixed enclosure displays
%     that enclosure between its recorded endpoint shapes.
%**************************************************************************
% INPUTS
%   - results (nonempty cell array)
%       Public planner results, one scalar struct per cell.
%   - labels (text, optional; default "Case 1", "Case 2", ...)
%       One caption per result.
%   - figureVisible (char row, optional; default 'on')
%       'on' for interactive display or 'off' for export.
%**************************************************************************
% OUTPUTS
%   - figureHandles (pageCount-by-1 figure handles)
%       One handle per gallery page. Failed results show their termination
%       reason instead of a motion length. Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Positions use the input coordinate units; snapshot times use seconds.
%**************************************************************************

%% Section 1: Validate Gallery Inputs

validateattributes(results, {'cell'}, {'nonempty'});
results = results(:);
if nargin < 2
    labels = "Case " + (1:numel(results)).';
end
if nargin < 3
    figureVisible = 'on';
end
labels = string(labels(:));
assert(numel(labels) == numel(results), 'plotTrajectoryGallery:InvalidLabels');
figureVisible = validatestring(figureVisible, {'on', 'off'});
for resultIndex = 1:numel(results)
    result = results{resultIndex};
    resultHasRequiredFields = isstruct(result) && isscalar(result) && ...
        all(isfield(result, {'Success', 'Inputs', 'PreparedObstacles', 'Limits'}));
    assert(resultHasRequiredFields, 'plotTrajectoryGallery:InvalidResult');
end

%% Section 2: Plot Protected Snapshots And Returned Motion

tilesPerPage  = 16;
pageCount     = ceil(numel(results) / tilesPerPage);
figureHandles = gobjects(pageCount, 1);
for pageIndex = 1:pageCount
    precedingResultCount = (pageIndex - 1) * tilesPerPage;
    pageResultCount      = min(tilesPerPage, numel(results) - precedingResultCount);
    pageResultIndices    = precedingResultCount + (1:pageResultCount);
    figureHandles(pageIndex) = figure('Visible', figureVisible, 'Color', 'w', ...
        'Position', [60, 60, 1400, 1000], 'Name', sprintf('BMTP gallery %d', pageIndex));
    layoutHandle = tiledlayout(figureHandles(pageIndex), 4, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(layoutHandle, 'Returned BMTP motion | moving obstacle at start / middle / end');
    for resultIndex = pageResultIndices
        result     = results{resultIndex};
        axesHandle = nexttile(layoutHandle);
        hold(axesHandle, 'on');

        % Preparation can stop when an obstacle's movement is unsupported.
        % Skip any requested picture whose geometry was not prepared, and
        % report that missing picture in the caption.
        displayTimes_s = linspace( ...
            result.Inputs.initialState.time_s, result.Inputs.goalState.time_s, 3);
        unpreparedSnapshotCount = 0;
        for obstacleIndex = 1:numel(result.PreparedObstacles)
            obstacle               = result.PreparedObstacles(obstacleIndex);
            obstacleIsMoving       = ~obstacle.InternalPreparation.IsTimeInvariant;
            obstacleDisplayTimes_s = displayTimes_s;
            obstacleColor          = [0.45, 0.48, 0.52];

            % Moving obstacles get three pictures. A stationary obstacle
            % needs only one, avoiding repeated copies of the same shape.
            if obstacleIsMoving
                obstacleColor = [0.88, 0.38, 0.12];
            else
                obstacleDisplayTimes_s = displayTimes_s(1);
            end
            for time_s = obstacleDisplayTimes_s
                if ~preparedTimeIsCovered(obstacle, time_s)
                    unpreparedSnapshotCount = unpreparedSnapshotCount + 1;
                    continue
                end
                obstacleShape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, time_s);
                plot(axesHandle, obstacleShape, 'FaceColor', obstacleColor, ...
                    'FaceAlpha', 0.12, 'EdgeColor', obstacleColor);
            end
        end

        % The dotted start-to-goal line is a reference. Draw a solid motion
        % path only when planning succeeded.
        startPosition_units = result.Inputs.initialState.position_units;
        goalPosition_units  = result.Inputs.goalState.position_units;
        plot(axesHandle, [startPosition_units(1), goalPosition_units(1)], ...
            [startPosition_units(2), goalPosition_units(2)], ...
            ':', 'Color', [0.65, 0.65, 0.65]);
        if result.Success
            plot(axesHandle, result.position_units(:, 1), result.position_units(:, 2), ...
                'Color', [0.05, 0.34, 0.69], 'LineWidth', 1.4);
            plotCaption = labels(resultIndex) + sprintf(' | L %.2f', result.MotionLength_units);
        else
            plotCaption = labels(resultIndex) + " | " + result.TerminationReason;
        end
        if unpreparedSnapshotCount > 0
            plotCaption = plotCaption + sprintf(' | %d unprepared snapshot(s)', unpreparedSnapshotCount);
        end
        plot(axesHandle, startPosition_units(1), startPosition_units(2), 'o', ...
            'Color', [0.05, 0.45, 0.2], 'MarkerFaceColor', [0.05, 0.45, 0.2]);
        plot(axesHandle, goalPosition_units(1), goalPosition_units(2), 'x', ...
            'Color', [0.05, 0.34, 0.69], 'LineWidth', 1.3);

        title(axesHandle, plotCaption, 'Interpreter', 'none', 'FontSize', 9);
        axis(axesHandle, 'equal');
        grid(axesHandle, 'on');
        xlabel(axesHandle, 'Azimuth / x');
        ylabel(axesHandle, 'Elevation / y');
    end
end
end

%% Section 3: Local Functions

function requiredGeometryIsPrepared = preparedTimeIsCovered(obstacle, requestedTime_s)
    % Check the same prepared samples and intervals that preparedShapeAtTime
    % needs. Outside a history with several samples, the obstacle is inactive
    % and there is no geometry to prepare for that picture.
    obstaclePreparation = obstacle.InternalPreparation;
    sampleTimes_s       = double(obstacle.time_s(:));

    requiredGeometryIsPrepared = true;
    if isempty(sampleTimes_s) || ~isfield(obstaclePreparation, 'SamplePrepared')
        return
    end
    requestedTimeIsOutsideHistory = numel(sampleTimes_s) > 1 && ...
        (requestedTime_s < sampleTimes_s(1) || requestedTime_s > sampleTimes_s(end));
    if requestedTimeIsOutsideHistory
        return
    end
    intervalStartSampleIndex = find(sampleTimes_s <= requestedTime_s, 1, "last");
    intervalEndSampleIndex   = find(sampleTimes_s >= requestedTime_s, 1, "first");
    if isscalar(sampleTimes_s)
        intervalStartSampleIndex = 1;
        intervalEndSampleIndex   = 1;
    end

    % An exact sample needs only that sample. Between two times, both
    % samples and their shared interval must have been prepared.
    requiredGeometryIsPrepared = obstaclePreparation.SamplePrepared(intervalStartSampleIndex) && ...
        obstaclePreparation.SamplePrepared(intervalEndSampleIndex);
    if intervalStartSampleIndex ~= intervalEndSampleIndex
        requiredGeometryIsPrepared = requiredGeometryIsPrepared && ...
            obstaclePreparation.IntervalPrepared(intervalStartSampleIndex);
    end
end
