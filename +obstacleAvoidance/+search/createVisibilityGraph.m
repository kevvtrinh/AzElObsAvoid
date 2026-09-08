function visibilityGraph = createVisibilityGraph(scene, start_units, goal_units, limits, options)
%% Section 0: Header & Readme
% SYNTAX: visibilityGraph = obstacleAvoidance.search.createVisibilityGraph(scene,start,goal,limits,options)
% PURPOSE: Find the exact shortest polygonal route in an implicit visibility
%          graph. A* evaluates edges on demand using an admissible distance.
% INPUTS: Protected polygon scene, endpoints, workspace, numerical tolerance.
% OUTPUTS: Exact boundary nodes, examined edges, route, and connectivity.
%          Unexamined edges remain implicit, never classified as blocked.
% UNITS: Coordinate units.

%% Section 1: Form The Occupied Union And Boundary Edge Arrays
validateattributes(start_units, {'numeric'}, {'real','finite','size',[1 2]});
validateattributes(goal_units, {'numeric'}, {'real','finite','size',[1 2]});
tolerance_units = options.ConstraintTolerance;
shape = polyshape();
for k = 1:numel(scene), shape = union(shape,scene(k).ProtectedShape); end
[edgeStart_units,edgeEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(shape,0);
boundary_units = shape.Vertices;
nodes_units = boundary_units(all(isfinite(boundary_units),2),:);
inWorkspace = nodes_units(:,1) > limits.xInterval_units(1) & nodes_units(:,1) < limits.xInterval_units(2) & ...
    nodes_units(:,2) > limits.yInterval_units(1) & nodes_units(:,2) < limits.yInterval_units(2);
nodes_units = unique([start_units;goal_units;nodes_units(inWorkspace,:)],'rows','stable');
sourceFree = pointIsFree(start_units,boundary_units,edgeStart_units,edgeEnd_units,tolerance_units);
goalFree = pointIsFree(goal_units,boundary_units,edgeStart_units,edgeEnd_units,tolerance_units);
endpoints_units = [start_units;goal_units];
workspaceFree = all(endpoints_units >= [limits.xInterval_units(1),limits.yInterval_units(1)] & ...
    endpoints_units <= [limits.xInterval_units(2),limits.yInterval_units(2)],2);
sourceFree = sourceFree && workspaceFree(1); goalFree = goalFree && workspaceFree(2);

%% Section 2: Search The Visibility Graph Lazily
nodeCount = size(nodes_units,1);
cost_units = Inf(nodeCount,1); cost_units(1) = 0;
lowerBound_units = vecnorm(nodes_units-goal_units,2,2);
closed = false(nodeCount,1); parent = zeros(nodeCount,1);
accepted = zeros(0,2); weights_units = zeros(0,1); rejected = zeros(0,2);
if sourceFree && goalFree
    while true
        priority_units = cost_units+lowerBound_units; priority_units(closed) = Inf;
        [priority_units,current] = min(priority_units);
        if ~isfinite(priority_units), break; end
        closed(current) = true;
        if current == 2, break; end
        distances_units = vecnorm(nodes_units-nodes_units(current,:),2,2);
        candidates = find(~closed & cost_units(current)+distances_units < cost_units);
        clear = true(numel(candidates),1);
        queryPoints_units = zeros(0,2); queryOwner = zeros(0,1);
        for k = 1:numel(candidates)
            [clear(k),midpoints_units] = segmentIntervals(nodes_units(current,:),nodes_units(candidates(k),:),edgeStart_units,edgeEnd_units,tolerance_units);
            queryPoints_units = [queryPoints_units;midpoints_units]; %#ok<AGROW>
            queryOwner = [queryOwner;repmat(k,size(midpoints_units,1),1)]; %#ok<AGROW>
        end
        % Keep every contact-partition interval, but classify them together.
        % Repeated scalar calls needlessly preprocess the same polygon rings.
        if ~isempty(queryOwner)
            [inside,on] = inpolygon(queryPoints_units(:,1),queryPoints_units(:,2),boundary_units(:,1),boundary_units(:,2));
            clear(queryOwner(inside & ~on)) = false;
        end
        next = candidates(clear);
        accepted = [accepted;repmat(current,numel(next),1),next]; %#ok<AGROW>
        weights_units = [weights_units;distances_units(next)]; %#ok<AGROW>
        cost_units(next) = cost_units(current)+distances_units(next);
        parent(next) = current;
        rejected = [rejected;repmat(current,nnz(~clear),1),candidates(~clear)]; %#ok<AGROW>
    end
end

%% Section 3: Return The Route And Search Evidence
routeIndex = zeros(1,0); route_units = zeros(0,2);
if closed(2)
    routeIndex = 2;
    while routeIndex(1) ~= 1, routeIndex = [parent(routeIndex(1)),routeIndex]; end %#ok<AGROW>
    route_units = nodes_units(routeIndex,:);
end
visibilityGraph = struct('NodePosition_units',nodes_units,'AcceptedNodeIndex',accepted, ...
    'AcceptedWeight_units',weights_units,'RejectedNodeIndex',rejected, ...
    'RouteNodeIndex',routeIndex,'Route_units',route_units,'RouteLength_units',cost_units(2), ...
    'SourceFree',sourceFree,'GoalFree',goalFree,'IsConnected',closed(2), ...
    'ExpandedCount',nnz(closed),'GraphIsFullyEnumerated',false);
end

function free = pointIsFree(point_units,boundary_units,first_units,last_units,tolerance_units)
    if isempty(first_units), free = true; return; end
    inside = inpolygon(point_units(1),point_units(2),boundary_units(:,1),boundary_units(:,2));
    edge_units = last_units-first_units;
    fraction = min(1,max(0,sum((point_units-first_units).*edge_units,2)./sum(edge_units.^2,2)));
    distance_units = vecnorm(point_units-first_units-fraction.*edge_units,2,2);
    free = ~inside && all(distance_units > tolerance_units);
end

function [clear,midpoints_units] = segmentIntervals(first_units,last_units,edgeStart_units,edgeEnd_units,tolerance_units)
    clear = true;
    midpoints_units = zeros(0,2);
    if isempty(edgeStart_units), return; end
    relevant = all(max(edgeStart_units,edgeEnd_units) >= min(first_units,last_units)-tolerance_units & ...
        min(edgeStart_units,edgeEnd_units) <= max(first_units,last_units)+tolerance_units,2);
    a_units = edgeStart_units(relevant,:); b_units = edgeEnd_units(relevant,:);
    direction_units = last_units-first_units; edge_units = b_units-a_units;
    offset_units = a_units-first_units;
    denominator_units2 = direction_units(1)*edge_units(:,2)-direction_units(2)*edge_units(:,1);
    parameterTolerance = tolerance_units/max(norm(direction_units),realmin);
    nonparallel = abs(denominator_units2) > tolerance_units*max(1,vecnorm(edge_units,2,2));
    t = (offset_units(:,1).*edge_units(:,2)-offset_units(:,2).*edge_units(:,1))./denominator_units2;
    u = (offset_units(:,1)*direction_units(2)-offset_units(:,2)*direction_units(1))./denominator_units2;
    crosses = nonparallel & t > parameterTolerance & t < 1-parameterTolerance & u > parameterTolerance & u < 1-parameterTolerance;
    if any(crosses), clear = false; return; end
    % All contacts partition the segment. Testing every open interval covers
    % concavities and holes, where testing a single midpoint is insufficient.
    contact = nonparallel & t >= 0 & t <= 1 & u >= -parameterTolerance & u <= 1+parameterTolerance;
    projection = (offset_units*direction_units.')/sum(direction_units.^2);
    endProjection = ((b_units-first_units)*direction_units.')/sum(direction_units.^2);
    collinear = ~nonparallel & abs(offset_units(:,1)*direction_units(2)-offset_units(:,2)*direction_units(1)) <= tolerance_units*norm(direction_units);
    cuts = unique([0;1;t(contact);min(1,max(0,projection(collinear)));min(1,max(0,endProjection(collinear)))]);
    midpoints_units = first_units+((cuts(1:end-1)+cuts(2:end))/2).*direction_units;
end
