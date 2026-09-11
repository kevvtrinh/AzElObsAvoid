function figures = plotTrajectoryGallery(results, labels, figureVisible)
%% Section 0: Header & Readme
% SYNTAX: figures = obstacleAvoidance.plotting.plotTrajectoryGallery(results, labels, figureVisible)
% PURPOSE: Compare returned planner motions in pages of sixteen spatial plots.
%   Moving protected geometry is shown at the request start, midpoint, and end;
%   these snapshots illustrate motion and are not a collision certificate.
% INPUTS: Nonempty cell array of public planner results, optional matching text
%   labels, and figureVisible ('on' by default, or 'off' for export).
% OUTPUTS: Figure handles. Failed results show their termination reason.
% UNITS: Positions use the input coordinate units; snapshot times use seconds.

%% Section 1: Validate Gallery Inputs
validateattributes(results,{'cell'},{'nonempty'});
results = results(:);
if nargin < 2, labels = "Case "+(1:numel(results)).'; end
if nargin < 3, figureVisible = 'on'; end
labels = string(labels(:));
assert(numel(labels)==numel(results),'plotTrajectoryGallery:InvalidLabels');
figureVisible = validatestring(figureVisible,{'on','off'});
for k = 1:numel(results)
    assert(isstruct(results{k}) && isscalar(results{k}) && ...
        all(isfield(results{k},{'Success','Inputs','PreparedObstacles','Limits'})), ...
        'plotTrajectoryGallery:InvalidResult');
end

%% Section 2: Plot Protected Snapshots And Returned Motion
pageCount = ceil(numel(results)/16);
figures = gobjects(pageCount,1);
for pageIndex = 1:pageCount
    indices = (pageIndex-1)*16+(1:min(16,numel(results)-(pageIndex-1)*16));
    figures(pageIndex) = figure('Visible',figureVisible,'Color','w', ...
        'Position',[60,60,1400,1000],'Name',sprintf('BMTP gallery %d',pageIndex));
    layout = tiledlayout(figures(pageIndex),4,4,'TileSpacing','compact','Padding','compact');
    title(layout,'Returned BMTP motion | moving obstacle at start / middle / end');
    for resultIndex = indices
        result = results{resultIndex};
        axesHandle = nexttile(layout);
        hold(axesHandle,'on');
        times_s = linspace(result.Inputs.initialState.time_s,result.Inputs.goalState.time_s,3);
        for obstacleIndex = 1:numel(result.PreparedObstacles)
            obstacle = result.PreparedObstacles(obstacleIndex);
            moving = ~obstacle.InternalPreparation.IsTimeInvariant;
            queryTimes_s = times_s;
            if ~moving, queryTimes_s = times_s(1); end
            for time_s = queryTimes_s
                shape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,time_s);
                color = [0.45,0.48,0.52];
                if moving, color = [0.88,0.38,0.12]; end
                plot(axesHandle,shape,'FaceColor',color,'FaceAlpha',0.12,'EdgeColor',color);
            end
        end
        start_units = result.Inputs.initialState.position_units;
        goal_units = result.Inputs.goalState.position_units;
        plot(axesHandle,[start_units(1),goal_units(1)],[start_units(2),goal_units(2)], ...
            ':','Color',[0.65,0.65,0.65]);
        if result.Success
            plot(axesHandle,result.position_units(:,1),result.position_units(:,2), ...
                'Color',[0.05,0.34,0.69],'LineWidth',1.4);
            caption = labels(resultIndex)+sprintf(' | L %.2f',result.MotionLength_units);
        else
            caption = labels(resultIndex)+" | "+result.TerminationReason;
        end
        plot(axesHandle,start_units(1),start_units(2),'o','Color',[0.05,0.45,0.2],'MarkerFaceColor',[0.05,0.45,0.2]);
        plot(axesHandle,goal_units(1),goal_units(2),'x','Color',[0.05,0.34,0.69],'LineWidth',1.3);
        title(axesHandle,caption,'Interpreter','none','FontSize',9);
        axis(axesHandle,'equal'); grid(axesHandle,'on');
        xlabel(axesHandle,'Azimuth / x'); ylabel(axesHandle,'Elevation / y');
    end
end
end
