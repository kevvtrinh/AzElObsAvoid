function warmStart = createWarmStart(request)
%% Section 0: Header & Readme
% SYNTAX: warmStart = bmtpEngine.createWarmStart(request)
% PURPOSE: Convert an exact visibility route into BMTP Bezier controls, or
%   preserve a supplied complete polynomial-edge motion without rebuilding it.
% INPUTS: request: validated BMTP request with static or timed exclusion cells;
%   request.Seed.PolynomialEdges optionally carries a complete physical motion.
% OUTPUTS: warmStart: route, control points, time, and all-pair region mask.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Use The Exact Visibility Route
if isfield(request.Seed,'PolynomialEdges') && ...
        ~isempty(request.Seed.PolynomialEdges)
    warmStart=completeMotionWarmStart(request);
    return
end
route_units = double(request.Seed.position_units);
route_units([1 end], :) = [request.InitialState.position_units; request.GoalState.position_units];
suppliedSegmentCount = size(route_units, 1) - 1;
usesLengthBalancedMesh=request.Options.GoalTimeMode=="earliestArrival" && ...
    ~isfield(request.Coverage,'ActiveTimeInterval_s') && request.SplitCount>1;
if usesLengthBalancedMesh
    route_units=removeRedundantRouteVertices(route_units);
end
originalSegmentCount = size(route_units, 1) - 1;
solverRoute_units=route_units;
if usesLengthBalancedMesh
    minimumSteeringSegmentCount=2*(request.Degree-2);
    targetSegmentCount=max(minimumSteeringSegmentCount, ...
        originalSegmentCount*request.SplitCount);
    segmentCountByEdge=allocateSegmentsByLength(route_units,targetSegmentCount);
    solverRoute_units=splitRouteByCount(route_units,segmentCountByEdge);
end
segmentCount = size(solverRoute_units, 1) - 1;
regionActiveBySegment = true(segmentCount, numel(request.Regions_units));

%% Section 2: Create Linear Rest-To-Rest Controls
degree = request.Degree;
fraction = reshape(min(1, max(0, ((0:degree) - 2) / (degree - 4))), 1, [], 1);
start_units = reshape(solverRoute_units(1:end - 1, :), segmentCount, 1, 2);
finish_units = reshape(solverRoute_units(2:end, :), segmentCount, 1, 2);
controlPoint_units = (1 - fraction) .* start_units + fraction .* finish_units;
segmentTime_s = bmtpEngine.findRequiredSegmentTime(controlPoint_units, request.Limits);
if request.SplitCount>1 && ~usesLengthBalancedMesh
    subdivisions = request.SplitCount;
    refined_units = zeros(segmentCount*subdivisions,degree+1,2);
    for k = 1:segmentCount
        for j = 1:subdivisions
            refined_units((k-1)*subdivisions+j,:,:) = bmtpEngine.restrictBezier(squeeze(controlPoint_units(k,:,:)),[(j-1),j]/subdivisions);
        end
    end
    controlPoint_units = refined_units;
    segmentTime_s = repelem(segmentTime_s,subdivisions)/subdivisions;
    regionActiveBySegment = repelem(regionActiveBySegment,subdivisions,1);
    segmentCount = segmentCount*subdivisions;
end

%% Section 3: Return The Solver Initialization
warmStart = struct();
warmStart.Route_units = route_units;
warmStart.ControlPoint_units = controlPoint_units;
warmStart.SegmentTime_s = segmentTime_s(:);
warmStart.Duration_s = sum(segmentTime_s);
warmStart.SegmentRatio = segmentTime_s(:) / mean(segmentTime_s);
warmStart.SegmentCount = segmentCount;
warmStart.RegionActiveBySegment = regionActiveBySegment;
warmStart.OriginalSeedSegmentCount = originalSegmentCount;
warmStart.SuppliedSeedSegmentCount = suppliedSegmentCount;
warmStart.WarmRouteResampled = false;
if request.UsesVariableClock && isfield(request.Coverage,'BreakTime_s')
    sourceBreaks_s = request.Coverage.BreakTime_s;
    % The timed guide's knots are physical events. Preserve every knot and
    % subdivide its normalized intervals so changing the arrival clock scales
    % the complete guide instead of deleting its waits.
    routeTau=double(request.Seed.tau(:));
    minimumSegmentCount=max([8,numel(sourceBreaks_s)-1, ...
        originalSegmentCount*request.SplitCount]);
    segmentCountByEdge=allocateSegmentsByMeasure(diff(routeTau), ...
        minimumSegmentCount);
    meshTau=splitScalarByCount(routeTau,segmentCountByEdge);
    segmentRatio=diff(meshTau)/mean(diff(meshTau));
    segmentTime_s=request.MotionHorizon_s*diff(meshTau);
    segmentCount=numel(segmentTime_s);
    tau=meshTau(1:end-1)+diff(meshTau).*((0:degree)/degree);
    breakTime_s=request.InitialState.time_s+[0;cumsum(segmentTime_s)];
    controls = interp1(request.Seed.tau,route_units,tau(:),'linear');
    controlPoint_units = reshape(controls,segmentCount,degree+1,2);
    controlPoint_units(1,1:3,:) = reshape(repmat(request.InitialState.position_units,3,1),1,3,2);
    controlPoint_units(end,end-2:end,:) = reshape(repmat(request.GoalState.position_units,3,1),1,3,2);
    intervals_s = request.Coverage.ActiveTimeInterval_s;
    warmStart.ControlPoint_units = controlPoint_units;
    warmStart.SegmentTime_s = segmentTime_s(:);
    warmStart.SegmentRatio = segmentRatio;
    warmStart.Duration_s = sum(segmentTime_s);
    warmStart.SegmentCount = segmentCount;
    warmStart.RegionActiveBySegment = breakTime_s(1:end-1) < intervals_s(:,2).' & ...
        breakTime_s(2:end) > intervals_s(:,1).';
    warmStart.WarmRouteResampled = true;
end

if request.Options.GoalTimeMode=="fixedArrival"
    % The motion mesh follows the guide, not the obstacle sampling frequency.
    % Every source interval still constrains its exact overlap with these spans.
    minimumSegmentCount = 8;
    if request.Degree>=8, minimumSegmentCount = 16; end
    segmentCount=max(minimumSegmentCount,originalSegmentCount);
    isTimedSeed = request.UsesTimeScopedSolver;
    if isTimedSeed
        segmentCount=max(segmentCount,originalSegmentCount*request.SplitCount);
        segmentCountByEdge=allocateSegmentsByMeasure(diff(request.Seed.tau), ...
            segmentCount);
        routeTau=splitScalarByCount(request.Seed.tau(:),segmentCountByEdge);
        segmentCount=numel(routeTau)-1;
        timedRoute_units = interp1(request.Seed.tau,route_units,routeTau,'linear');
        start_units = reshape(timedRoute_units(1:end-1,:),segmentCount,1,2);
        finish_units = reshape(timedRoute_units(2:end,:),segmentCount,1,2);
        warmStart.ControlPoint_units = (1-fraction).*start_units+fraction.*finish_units;
        warmStart.Route_units = timedRoute_units;
        warmStart.SegmentTime_s=diff(routeTau)*request.MotionHorizon_s;
        warmStart.SegmentRatio=warmStart.SegmentTime_s/ ...
            mean(warmStart.SegmentTime_s);
    else
        tau=((0:segmentCount-1).'+(0:degree)/degree)/segmentCount;
        controls=interp1(request.Seed.tau,route_units,tau(:),'linear');
        warmStart.ControlPoint_units=reshape(controls,segmentCount,degree+1,2);
        warmStart.SegmentTime_s=repmat( ...
            request.MotionHorizon_s/segmentCount,segmentCount,1);
        warmStart.SegmentRatio=ones(segmentCount,1);
    end
    warmStart.SegmentCount=segmentCount;
    warmStart.RegionActiveBySegment=true(segmentCount,numel(request.Regions_units));
    if isfield(request.Coverage,'ActiveTimeInterval_s')
        breaks_s=request.InitialState.time_s+[0;cumsum(warmStart.SegmentTime_s)];
        intervals_s=request.Coverage.ActiveTimeInterval_s;
        warmStart.RegionActiveBySegment=breaks_s(1:end-1)<intervals_s(:,2).' & ...
            breaks_s(2:end)>intervals_s(:,1).';
    end
    warmStart.WarmRouteResampled=true;
end
if request.Options.GoalTimeMode=="fixedArrival"
    warmStart.SegmentTime_s = warmStart.SegmentTime_s * request.MotionHorizon_s/sum(warmStart.SegmentTime_s);
    warmStart.Duration_s = request.MotionHorizon_s;
end
warmStart.ControlPoint_units = bmtpEngine.imposeEndpointControls(warmStart.ControlPoint_units, ...
    warmStart.SegmentTime_s,request.InitialState,request.GoalState);
if request.UsesVariableClock
    requiredSegmentTime_s=bmtpEngine.findRequiredSegmentTime( ...
        warmStart.ControlPoint_units,request.Limits);
    % Preserve the collision-free timed guide's physical clock for plane
    % initialization without turning that proposal time into an arrival bound.
    commonSegmentTime_s=max([requiredSegmentTime_s./warmStart.SegmentRatio; ...
        request.SeedMotionDuration_s/sum(warmStart.SegmentRatio)]);
    warmStart.SegmentTime_s=commonSegmentTime_s*warmStart.SegmentRatio;
    warmStart.Duration_s=sum(warmStart.SegmentTime_s);
    if isfield(request.Coverage,'ActiveTimeInterval_s')
        breaks_s=request.InitialState.time_s+[0;cumsum(warmStart.SegmentTime_s)];
        intervals_s=request.Coverage.ActiveTimeInterval_s;
        warmStart.RegionActiveBySegment=breaks_s(1:end-1)<intervals_s(:,2).' & ...
            breaks_s(2:end)>intervals_s(:,1).';
    end
end
end

function warmStart=completeMotionWarmStart(request)
    % Validate and preserve the complete edge clock and physical jet state.
    edges=request.Seed.PolynomialEdges(:);
    requiredFields={'StartTime_s','EndTime_s','SegmentDuration_s', ...
        'ControlPoint_units','PositionPower_units','StartJet','EndJet'};
    valid=isstruct(edges) && ~isempty(edges) && all(isfield(edges,requiredFields));
    edgeCount=numel(edges);
    degree=request.Degree;
    controlPoint_units=zeros(edgeCount,degree+1,2);
    positionPower_units=zeros(edgeCount,2,degree+1);
    segmentTime_s=zeros(edgeCount,1);
    startTime_s=zeros(edgeCount,1);
    endTime_s=zeros(edgeCount,1);
    for edgeIndex=1:edgeCount
        control=edges(edgeIndex).ControlPoint_units;
        power=edges(edgeIndex).PositionPower_units;
        valid=valid && isequal(size(control),[degree+1,2]) && ...
            isequal(size(power),[2,degree+1]) && ...
            isequal(size(edges(edgeIndex).StartJet),[4,2]) && ...
            isequal(size(edges(edgeIndex).EndJet),[4,2]);
        if ~valid,break;end
        controlPoint_units(edgeIndex,:,:)=control;
        positionPower_units(edgeIndex,:,:)=power;
        segmentTime_s(edgeIndex)=edges(edgeIndex).SegmentDuration_s;
        startTime_s(edgeIndex)=edges(edgeIndex).StartTime_s;
        endTime_s(edgeIndex)=edges(edgeIndex).EndTime_s;
    end
    tolerance=request.Options.ConstraintTolerance;
    if valid
        startJet=cat(3,edges.StartJet);
        endJet=cat(3,edges.EndJet);
        sharedResidual=0;
        if edgeCount>1
            sharedResidual=max(abs(endJet(:,:,1:end-1)- ...
                startJet(:,:,2:end)),[],'all');
        end
        expectedInitial=[request.InitialState.position_units; ...
            request.InitialState.velocity_units_s; ...
            request.InitialState.acceleration_units_s2];
        expectedGoal=[request.GoalState.position_units; ...
            request.GoalState.velocity_units_s; ...
            request.GoalState.acceleration_units_s2];
        valid=all(isfinite([controlPoint_units(:);positionPower_units(:); ...
            segmentTime_s;startTime_s;endTime_s])) && ...
            all(segmentTime_s>0) && ...
            max(abs(startJet(1:3,:,1)-expectedInitial),[],'all')<=tolerance && ...
            max(abs(endJet(1:3,:,end)-expectedGoal),[],'all')<=tolerance && ...
            sharedResidual<=tolerance && ...
            abs(startTime_s(1)-request.InitialState.time_s)<=tolerance && ...
            max(abs(endTime_s-startTime_s-segmentTime_s))<=tolerance && ...
            (edgeCount==1 || max(abs(startTime_s(2:end)- ...
                endTime_s(1:end-1)))<=tolerance) && ...
            endTime_s(end)<=request.GoalState.time_s+ ...
                request.Options.ArrivalTimeTolerance_s;
        if request.Options.GoalTimeMode=="fixedArrival"
            valid=valid && abs(endTime_s(end)-request.GoalState.time_s)<= ...
                request.Options.ArrivalTimeTolerance_s;
        end
    end
    if ~valid
        error('bmtpEngine:InvalidPolynomialEdges', ...
            'Complete polynomial edges must match the request clock, degree, endpoints, and shared physical jets.');
    end
    route_units=zeros(edgeCount+1,2);
    for edgeIndex=1:edgeCount
        route_units(edgeIndex,:)=edges(edgeIndex).StartJet(1,:);
    end
    route_units(end,:)=edges(end).EndJet(1,:);
    intervals_s=zeros(0,2);
    if isfield(request.Coverage,'ActiveTimeInterval_s')
        intervals_s=request.Coverage.ActiveTimeInterval_s;
    end
    regionActiveBySegment=true(edgeCount,numel(request.Regions_units));
    if ~isempty(intervals_s)
        regionActiveBySegment=startTime_s<intervals_s(:,2).' & ...
            endTime_s>intervals_s(:,1).';
    end
    warmStart=struct('Route_units',route_units, ...
        'ControlPoint_units',controlPoint_units, ...
        'PrescribedPower_units',positionPower_units, ...
        'SegmentTime_s',segmentTime_s, ...
        'Duration_s',sum(segmentTime_s), ...
        'SegmentRatio',segmentTime_s/mean(segmentTime_s), ...
        'SegmentCount',edgeCount, ...
        'RegionActiveBySegment',regionActiveBySegment, ...
        'OriginalSeedSegmentCount',edgeCount, ...
        'SuppliedSeedSegmentCount',edgeCount, ...
        'WarmRouteResampled',false);
end

function route_units=removeRedundantRouteVertices(route_units)
    keep=true(size(route_units,1),1);
    scale_units=max(1,max(abs(route_units),[],'all'));
    tolerance_units=128*eps(scale_units);
    changed=true;
    while changed
        changed=false;
        indices=find(keep);
        for localIndex=2:numel(indices)-1
            previous=route_units(indices(localIndex-1),:);
            current=route_units(indices(localIndex),:);
            following=route_units(indices(localIndex+1),:);
            first=current-previous;
            second=following-current;
            firstLength=norm(first);
            secondLength=norm(second);
            sameDirection=dot(first,second)>=-tolerance_units^2;
            area=abs(first(1)*second(2)-first(2)*second(1));
            collinear=area<=tolerance_units*(firstLength+secondLength);
            if firstLength<=tolerance_units || secondLength<=tolerance_units || ...
                    (sameDirection && collinear)
                keep(indices(localIndex))=false;
                changed=true;
                break
            end
        end
    end
    route_units=route_units(keep,:);
end

function segmentCountByEdge=allocateSegmentsByLength(route_units,targetSegmentCount)
    edgeLength_units=vecnorm(diff(route_units),2,2);
    segmentCountByEdge=allocateSegmentsByMeasure(edgeLength_units,targetSegmentCount);
end

function segmentCountByEdge=allocateSegmentsByMeasure(edgeMeasure,targetSegmentCount)
    edgeMeasure=edgeMeasure(:);
    edgeCount=numel(edgeMeasure);
    segmentCountByEdge=ones(edgeCount,1);
    remainingCount=targetSegmentCount-edgeCount;
    if remainingCount<=0 || sum(edgeMeasure)<=0, return; end
    exactCount=remainingCount*edgeMeasure/sum(edgeMeasure);
    additional=floor(exactCount);
    segmentCountByEdge=segmentCountByEdge+additional;
    unassigned=remainingCount-sum(additional);
    [~,order]=sortrows([-mod(exactCount,1),(1:edgeCount).'],[1 2]);
    segmentCountByEdge(order(1:unassigned))=segmentCountByEdge(order(1:unassigned))+1;
end

function refined=splitScalarByCount(values,segmentCountByEdge)
    refined=zeros(sum(segmentCountByEdge)+1,1);
    target=1;
    for edgeIndex=1:numel(segmentCountByEdge)
        count=segmentCountByEdge(edgeIndex);
        fraction=(0:count-1).'/count;
        rows=target:target+count-1;
        refined(rows)=values(edgeIndex)+ ...
            fraction*(values(edgeIndex+1)-values(edgeIndex));
        target=target+count;
    end
    refined(end)=values(end);
end

function refined_units=splitRouteByCount(route_units,segmentCountByEdge)
    refined_units=zeros(sum(segmentCountByEdge)+1,2);
    target=1;
    for edgeIndex=1:numel(segmentCountByEdge)
        count=segmentCountByEdge(edgeIndex);
        fraction=(0:count-1).'/count;
        rows=target:target+count-1;
        refined_units(rows,:)=route_units(edgeIndex,:)+ ...
            fraction.*(route_units(edgeIndex+1,:)-route_units(edgeIndex,:));
        target=target+count;
    end
    refined_units(end,:)=route_units(end,:);
end
