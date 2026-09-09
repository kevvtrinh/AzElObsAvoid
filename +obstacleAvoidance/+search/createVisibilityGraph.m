function visibilityGraph = createVisibilityGraph(scene, start_units, goal_units, limits, options, monotoneDirection)
%% Section 0: Header & Readme
% SYNTAX: visibilityGraph = obstacleAvoidance.search.createVisibilityGraph(scene,start,goal,limits,options)
% PURPOSE: Find the exact shortest polygonal route in an implicit visibility
%          graph. A* checks optimistic parent edges only when their child
%          reaches the frontier, with exact boundary-cone rejection first.
% INPUTS: Protected polygon scene, endpoints, workspace, numerical tolerance.
%         Optional monotoneDirection requires strictly positive edge progress.
% OUTPUTS: Exact boundary nodes, examined edges, route, and connectivity.
%          Unexamined edges remain implicit, never classified as blocked.
% UNITS: Coordinate units.

%% Section 1: Form The Occupied Union And Boundary Edge Arrays
validateattributes(start_units, {'numeric'}, {'real','finite','size',[1 2]});
validateattributes(goal_units, {'numeric'}, {'real','finite','size',[1 2]});
tolerance_units = options.ConstraintTolerance;
if nargin<6, monotoneDirection = [0,0]; end
validateattributes(monotoneDirection,{'numeric'},{'real','finite','size',[1,2]});
shape = polyshape();
for k = 1:numel(scene), shape = union(shape,scene(k).ProtectedShape); end
[edgeStart_units,edgeEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(shape,0);
edgeVector_units = edgeEnd_units-edgeStart_units;
edgeBounds_units = [min(edgeStart_units,edgeEnd_units),max(edgeStart_units,edgeEnd_units)];
parallelTolerance_units2 = tolerance_units*max(1,vecnorm(edgeVector_units,2,2));
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
blocked = sparse(nodeCount,nodeCount); queryCount=0;
% Only closed nodes offer parent costs. An unchecked edge is a lower bound;
% rejecting it raises that bound and returns its child to the frontier.
if sourceFree && goalFree
    cones=endpointCones(shape,nodes_units,edgeStart_units,edgeEnd_units,tolerance_units);
    while true
        priority_units = cost_units+lowerBound_units; priority_units(closed) = Inf;
        [priority_units,current] = min(priority_units);
        if ~isfinite(priority_units), break; end
        if current~=1
            previous=parent(current);
            queryCount=queryCount+1;
            [clear,queryPoints_units,queryOwner] = segmentIntervals(nodes_units(previous,:),nodes_units(current,:), ...
                edgeStart_units,edgeEnd_units,edgeVector_units,edgeBounds_units,parallelTolerance_units2,tolerance_units);
            if ~isempty(queryOwner)
                [inside,on] = inpolygon(queryPoints_units(:,1),queryPoints_units(:,2),boundary_units(:,1),boundary_units(:,2));
                clear(queryOwner(inside & ~on)) = false;
            end
            if ~clear
                blocked(previous,current)=1;
                rejected(end+1,:)=[previous,current];
                possible=find(closed & ~blocked(:,current));
                if any(monotoneDirection)
                    possible=possible((nodes_units(current,:)-nodes_units(possible,:))*monotoneDirection.'>tolerance_units);
                end
                possible=possible(~entersObstacle(possible,current,nodes_units,cones,tolerance_units));
                alternatives=cost_units(possible)+vecnorm(nodes_units(possible,:)-nodes_units(current,:),2,2);
                cost_units(current)=Inf;
                if ~isempty(possible)
                    [cost_units(current),best]=min(alternatives); parent(current)=possible(best);
                end
                continue;
            end
            accepted(end+1,:)=[previous,current];
            weights_units(end+1,1)=norm(nodes_units(current,:)-nodes_units(previous,:));
        end
        closed(current) = true;
        if current == 2, break; end
        distances_units = vecnorm(nodes_units-nodes_units(current,:),2,2);
        candidates = find(~closed & cost_units(current)+distances_units < cost_units);
        if any(monotoneDirection)
            candidates = candidates((nodes_units(candidates,:)-nodes_units(current,:))*monotoneDirection.'>tolerance_units);
        end
        localBlocked=entersObstacle(current,candidates,nodes_units,cones,tolerance_units);
        rejected=[rejected;repmat(current,nnz(localBlocked),1),candidates(localBlocked)];
        blocked(current,candidates(localBlocked))=1;
        candidates=candidates(~localBlocked);
        cost_units(candidates) = cost_units(current)+distances_units(candidates);
        parent(candidates) = current;
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
    'ExpandedCount',nnz(closed),'GraphIsFullyEnumerated',false,'CollisionQueryCount',queryCount);
end

function free = pointIsFree(point_units,boundary_units,first_units,last_units,tolerance_units)
    if isempty(first_units), free = true; return; end
    inside = inpolygon(point_units(1),point_units(2),boundary_units(:,1),boundary_units(:,2));
    edge_units = last_units-first_units;
    fraction = min(1,max(0,sum((point_units-first_units).*edge_units,2)./sum(edge_units.^2,2)));
    distance_units = vecnorm(point_units-first_units-fraction.*edge_units,2,2);
    free = ~inside && all(distance_units > tolerance_units);
end

function [clear,midpoints_units,owners] = segmentIntervals(first_units,last_units,edgeStart_units,edgeEnd_units,edge_units,bounds_units,parallelTolerance_units2,tolerance_units)
    clear = true(size(last_units,1),1);
    midpoints_units = zeros(0,2); owners = zeros(0,1);
    if isempty(edgeStart_units), return; end
    offset_units = edgeStart_units-first_units;
    endOffset_units = edgeEnd_units-first_units;
    numerator_units2 = offset_units(:,1).*edge_units(:,2)-offset_units(:,2).*edge_units(:,1);
    % Bound temporary edge-by-candidate arrays independently of scene size.
    blockSize = max(1,floor(2^18/size(edge_units,1)));
    points = cell(size(last_units,1),1); pointOwners = points;
    for blockStart = 1:blockSize:size(last_units,1)
        indices = blockStart:min(size(last_units,1),blockStart+blockSize-1);
        direction_units = last_units(indices,:)-first_units;
        lengths_units = vecnorm(direction_units,2,2).';
        parameterTolerance = tolerance_units./max(lengths_units,realmin);
        lower_units = min(first_units,last_units(indices,:))-tolerance_units;
        upper_units = max(first_units,last_units(indices,:))+tolerance_units;
        relevant = bounds_units(:,3)>=lower_units(:,1).' & bounds_units(:,4)>=lower_units(:,2).' & ...
            bounds_units(:,1)<=upper_units(:,1).' & bounds_units(:,2)<=upper_units(:,2).';
        denominator_units2 = edge_units(:,2)*direction_units(:,1).'-edge_units(:,1)*direction_units(:,2).';
        crossOffset_units2 = offset_units(:,1)*direction_units(:,2).'-offset_units(:,2)*direction_units(:,1).';
        nonparallel = abs(denominator_units2)>parallelTolerance_units2;
        t = numerator_units2./denominator_units2; u = crossOffset_units2./denominator_units2;
        crosses = relevant & nonparallel & t>parameterTolerance & t<1-parameterTolerance & u>parameterTolerance & u<1-parameterTolerance;
        clear(indices) = ~any(crosses,1).';
        % Retain every contact-partition interval, including collinear edges.
        contact = relevant & nonparallel & t>=0 & t<=1 & u>=-parameterTolerance & u<=1+parameterTolerance;
        collinear = relevant & ~nonparallel & abs(crossOffset_units2)<=tolerance_units*lengths_units;
        for k = find(clear(indices)).'
            projection = (offset_units(collinear(:,k),:)*direction_units(k,:).')/sum(direction_units(k,:).^2);
            endProjection = (endOffset_units(collinear(:,k),:)*direction_units(k,:).')/sum(direction_units(k,:).^2);
            cuts = unique([0;1;t(contact(:,k),k);min(1,max(0,projection));min(1,max(0,endProjection))]);
            points{indices(k)} = first_units+((cuts(1:end-1)+cuts(2:end))/2).*direction_units(k,:);
            pointOwners{indices(k)} = repmat(indices(k),numel(cuts)-1,1);
        end
    end
    midpoints_units = vertcat(points{:}); owners = vertcat(pointOwners{:});
end


function cones=endpointCones(shape,nodes_units,edgeStart_units,edgeEnd_units,tolerance_units)
    % An adjacent filled triangle identifies each boundary edge's occupied
    % side without assuming ring winding. This also handles hole boundaries.
    % Shared/touching vertices and nearly flat turns retain the full predicate.
    count=size(nodes_units,1);
    cones=struct('Incoming',zeros(count,2),'Outgoing',zeros(count,2), ...
        'Side',zeros(count,1),'Convex',false(count,1),'Enabled',false(count,1));
    if isempty(edgeStart_units), return; end
    mesh=triangulation(shape); faces=mesh.ConnectivityList;
    meshEdges=[faces(:,[1,2]);faces(:,[2,3]);faces(:,[3,1])];
    opposite=[faces(:,3);faces(:,1);faces(:,2)];
    [firstFound,first]=ismember(edgeStart_units,mesh.Points,'rows');
    [lastFound,last]=ismember(edgeEnd_units,mesh.Points,'rows');
    [edgeFound,face]=ismember(sort([first,last],2),sort(meshEdges,2),'rows');
    known=firstFound & lastFound & edgeFound;
    edgeVector_units=edgeEnd_units-edgeStart_units; side=zeros(size(edgeVector_units,1),1);
    direction_units=mesh.Points(opposite(face(known)),:)-edgeStart_units(known,:);
    side(known)=sign(edgeVector_units(known,1).*direction_units(:,2)-edgeVector_units(known,2).*direction_units(:,1));
    [inFound,incoming]=ismember(nodes_units,edgeEnd_units,'rows');
    [outFound,outgoing]=ismember(nodes_units,edgeStart_units,'rows');
    [mapped,node]=ismember(edgeEnd_units,nodes_units,'rows'); inCount=accumarray(node(mapped),1,[count,1]);
    [mapped,node]=ismember(edgeStart_units,nodes_units,'rows'); outCount=accumarray(node(mapped),1,[count,1]);
    incoming=max(1,incoming); outgoing=max(1,outgoing);
    cones.Incoming=edgeVector_units(incoming,:); cones.Outgoing=edgeVector_units(outgoing,:); cones.Side=side(incoming);
    turn_units2=cones.Side.*(cones.Incoming(:,1).*cones.Outgoing(:,2)-cones.Incoming(:,2).*cones.Outgoing(:,1));
    cones.Enabled=inFound & outFound & inCount==1 & outCount==1 & ...
        side(incoming)==side(outgoing) & side(incoming)~=0 & ...
        abs(turn_units2)>tolerance_units*max(1,vecnorm(cones.Incoming,2,2).*vecnorm(cones.Outgoing,2,2));
    cones.Convex=turn_units2>0;
end

function blocked=entersObstacle(first,last,nodes_units,cones,tolerance_units)
    % At a convex corner the interior is the intersection of two halfplanes;
    % at a reentrant corner it is their union. Reject only strict interior
    % directions. Tangencies and numerically ambiguous cases use full checks.
    direction_units=nodes_units(last,:)-nodes_units(first,:); blocked=false(size(direction_units,1),1);
    endpoints={first,last};
    for endpoint=1:2
        indices=endpoints{endpoint};
        incoming=cones.Incoming(indices,:); outgoing=cones.Outgoing(indices,:); side=cones.Side(indices);
        incomingSide=side.*(incoming(:,1).*direction_units(:,2)-incoming(:,2).*direction_units(:,1));
        outgoingSide=side.*(outgoing(:,1).*direction_units(:,2)-outgoing(:,2).*direction_units(:,1));
        roundoff=64*eps(max(1,max(abs(nodes_units(indices,:)),[],2))).*max(1,vecnorm(direction_units,2,2));
        in=incomingSide>(tolerance_units+roundoff).*vecnorm(incoming,2,2);
        out=outgoingSide>(tolerance_units+roundoff).*vecnorm(outgoing,2,2);
        blocked=blocked | (cones.Enabled(indices) & ((cones.Convex(indices) & in & out) | (~cones.Convex(indices) & (in | out))));
        direction_units=-direction_units;
    end
end
