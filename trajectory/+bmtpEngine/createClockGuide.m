function graph = createClockGuide(request, phases, axisIndex)
%% Section 0: Header & Readme
% SYNTAX: graph = bmtpEngine.createClockGuide(request, phases, axis)
% PURPOSE: Project source time cells onto a monotone time-limiting motion.
% INPUTS: Validated request, analytic scalar phase states, and locked axis.
% OUTPUTS: Exact monotone visibility graph of the projected convex enclosures.
% UNITS: Coordinate units and seconds.

%% Section 1: Intersect Constant-Velocity Phases With Space-Time Cells

if isfield(request.Coverage,'StaticScene')
    direction = [0,0];
    direction(axisIndex) = sign(request.GoalState.position_units(axisIndex)-request.InitialState.position_units(axisIndex));
    graph = obstacleAvoidance.search.createVisibilityGraph(request.Coverage.StaticScene,request.InitialState.position_units, ...
        request.GoalState.position_units,request.Limits,request.Options,direction);
    return;
end
scene = struct('ProtectedShape',{});
for k = 1:numel(phases.SegmentTime_s)
    phaseStart_s = request.InitialState.time_s+phases.StartTime_s(k);
    phaseEnd_s = phaseStart_s+phases.SegmentTime_s(k);
    position_units = phases.Position_units(k,1);
    velocity_units_s = phases.Velocity_units_s(k,1);
    acceleration_units_s2 = phases.Acceleration_units_s2(k,1);
    jerk_units_s3 = phases.Jerk_units_s3(k,1);
    final_units = position_units+velocity_units_s*phases.SegmentTime_s(k)+ ...
        acceleration_units_s2*phases.SegmentTime_s(k)^2/2+jerk_units_s3*phases.SegmentTime_s(k)^3/6;
    axisInterval_units = sort([position_units,final_units]);
    for region = 1:numel(request.Regions_units)
        interval_s = [phaseStart_s,phaseEnd_s];
        if isfield(request.Coverage,'ActiveTimeInterval_s')
            active_s = request.Coverage.ActiveTimeInterval_s(region,:);
            interval_s = [max(interval_s(1),active_s(1)),min(interval_s(2),active_s(2))];
            if interval_s(1)>=interval_s(2), continue; end
        end
        vertices = bmtpEngine.regionOnInterval(request.Regions_units{region},request.Coverage,region,interval_s);
        first = vertices(:,:,1); last = vertices(:,:,end);
        points = [first;last];
        if max(points(:,axisIndex))<axisInterval_units(1) || min(points(:,axisIndex))>axisInterval_units(2), continue; end
        if acceleration_units_s2==0 && jerk_units_s3==0 && velocity_units_s~=0
            % A constant-velocity axis is a plane in space-time. Every vertex
            % of its convex-cell intersection lies on an edge. Considering
            % all opposite-side vertex pairs includes every such edge and
            % only points in that convex intersection.
            times_s = [repmat(interval_s(1),size(first,1),1);repmat(interval_s(2),size(last,1),1)];
            residual = points(:,axisIndex)-position_units-velocity_units_s*(times_s-phaseStart_s);
            [positive,negative] = ndgrid(find(residual>0),find(residual<0));
            positive = positive(:); negative = negative(:);
            fraction = residual(positive)./(residual(positive)-residual(negative));
            points = [points(residual==0,:);points(positive,:)+fraction.*(points(negative,:)-points(positive,:))];
        end
        if size(points,1)<3, continue; end
        points = unique(points,'rows');
        if size(points,1)<3 || rank(points-points(1,:))<2, continue; end
        hull = convhull(points(:,1),points(:,2));
        shape = polyshape(points(hull(1:end-1),:),'Simplify',false);
        % Nonlinear phases retain a conservative spatial enclosure clipped
        % to their reachable axis range; no source occupancy is discarded.
        if axisIndex==1
            strip = [axisInterval_units(1),request.Limits.yInterval_units(1);axisInterval_units(2),request.Limits.yInterval_units(1); ...
                axisInterval_units(2),request.Limits.yInterval_units(2);axisInterval_units(1),request.Limits.yInterval_units(2)];
        else
            strip = [request.Limits.xInterval_units(1),axisInterval_units(1);request.Limits.xInterval_units(2),axisInterval_units(1); ...
                request.Limits.xInterval_units(2),axisInterval_units(2);request.Limits.xInterval_units(1),axisInterval_units(2)];
        end
        shape = intersect(shape,polyshape(strip));
        if ~isempty(shape.Vertices), scene(end+1).ProtectedShape = shape; end %#ok<AGROW>
    end
end

%% Section 2: Search With The Monotonicity Required By The Locked Axis

direction = [0,0];
direction(axisIndex) = sign(request.GoalState.position_units(axisIndex)-request.InitialState.position_units(axisIndex));
graph = obstacleAvoidance.search.createVisibilityGraph(scene,request.InitialState.position_units, ...
    request.GoalState.position_units,request.Limits,request.Options,direction);
end
