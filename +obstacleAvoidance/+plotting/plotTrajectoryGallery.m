function figureHandles = plotTrajectoryGallery(results, labels, figureVisible)
%% Section 0: Header & Readme
% SYNTAX
%   figureHandles = obstacleAvoidance.plotting.plotTrajectoryGallery(results)
%   figureHandles = obstacleAvoidance.plotting.plotTrajectoryGallery(results, labels)
%   figureHandles = obstacleAvoidance.plotting.plotTrajectoryGallery(results, labels, figureVisible)
%**************************************************************************
% PURPOSE
%   - Compare returned planner motions in pages of sixteen spatial plots.
%   - Moving protected geometry is shown at the request start, midpoint, and
%     end; these snapshots illustrate motion and are not a collision
%     certificate. Interior swept-model snapshots draw the shared prepared
%     cell union.
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
    result       = results{resultIndex};
    resultIsCore = isstruct(result) && isscalar(result) && ...
        all(isfield(result, {'Success', 'Inputs', 'PreparedObstacles', 'Limits'}));
    assert(resultIsCore, 'plotTrajectoryGallery:InvalidResult');
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

        % A failure result may carry geometry prepared only up to the
        % unsupported interval; unprepared snapshot times are reported, not
        % queried.
        times_s             = linspace(result.Inputs.initialState.time_s, result.Inputs.goalState.time_s, 3);
        unpreparedTimeCount = 0;
        for obstacleIndex = 1:numel(result.PreparedObstacles)
            obstacle         = result.PreparedObstacles(obstacleIndex);
            obstacleIsMoving = ~obstacle.InternalPreparation.IsTimeInvariant;
            queryTimes_s     = times_s;
            obstacleColor    = [0.45, 0.48, 0.52];
            if obstacleIsMoving
                obstacleColor = [0.88, 0.38, 0.12];
            else
                queryTimes_s = times_s(1);
            end
            for time_s = queryTimes_s
                if ~preparedTimeIsCovered(obstacle, time_s)
                    unpreparedTimeCount = unpreparedTimeCount + 1;
                    continue
                end
                obstacleShape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, time_s);
                plot(axesHandle, obstacleShape, 'FaceColor', obstacleColor, ...
                    'FaceAlpha', 0.12, 'EdgeColor', obstacleColor);
            end
        end

        start_units = result.Inputs.initialState.position_units;
        goal_units  = result.Inputs.goalState.position_units;
        plot(axesHandle, [start_units(1), goal_units(1)], [start_units(2), goal_units(2)], ...
            ':', 'Color', [0.65, 0.65, 0.65]);
        if result.Success
            plot(axesHandle, result.position_units(:, 1), result.position_units(:, 2), ...
                'Color', [0.05, 0.34, 0.69], 'LineWidth', 1.4);
            caption = labels(resultIndex) + sprintf(' | L %.2f', result.MotionLength_units);
        else
            caption = labels(resultIndex) + " | " + result.TerminationReason;
        end
        if unpreparedTimeCount > 0
            caption = caption + sprintf(' | %d unprepared snapshot(s)', unpreparedTimeCount);
        end
        plot(axesHandle, start_units(1), start_units(2), 'o', ...
            'Color', [0.05, 0.45, 0.2], 'MarkerFaceColor', [0.05, 0.45, 0.2]);
        plot(axesHandle, goal_units(1), goal_units(2), 'x', ...
            'Color', [0.05, 0.34, 0.69], 'LineWidth', 1.3);

        title(axesHandle, caption, 'Interpreter', 'none', 'FontSize', 9);
        axis(axesHandle, 'equal');
        grid(axesHandle, 'on');
        xlabel(axesHandle, 'Azimuth / x');
        ylabel(axesHandle, 'Elevation / y');
    end
end
end

%% Section 3: Local Functions

function covered = preparedTimeIsCovered(obstacle, queryTime_s)
    % Apply the coverage rule of preparedShapeAtTime without throwing.
    % A time outside a multi-sample history draws as inactive and is covered.
    preparation = obstacle.InternalPreparation;
    time_s      = double(obstacle.time_s(:));
    covered     = true;
    if isempty(time_s) || ~isfield(preparation, 'SamplePrepared')
        return
    end
    queryOutsideHistory = numel(time_s) > 1 && (queryTime_s < time_s(1) || queryTime_s > time_s(end));
    if queryOutsideHistory
        return
    end
    lowerSampleIndex = find(time_s <= queryTime_s, 1, "last");
    upperSampleIndex = find(time_s >= queryTime_s, 1, "first");
    if isscalar(time_s)
        lowerSampleIndex = 1;
        upperSampleIndex = 1;
    end
    covered = preparation.SamplePrepared(lowerSampleIndex) && preparation.SamplePrepared(upperSampleIndex);
    if lowerSampleIndex ~= upperSampleIndex
        covered = covered && preparation.IntervalPrepared(lowerSampleIndex);
    end
end
