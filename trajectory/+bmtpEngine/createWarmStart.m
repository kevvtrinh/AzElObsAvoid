function warmStart = createWarmStart(request)
%% Section 0: Header & Readme
% SYNTAX
%   warmStart = bmtpEngine.createWarmStart(request)
%
% PURPOSE
%   - Convert the exact visibility route into quintic BMTP controls.
%
% INPUTS
%   - request: validated BMTP request with static or timed exclusion cells.
%
% OUTPUTS
%   - warmStart: route, control points, time, and all-pair region mask.
%
% UNITS
%   - Position is coordinate units and time is seconds.

%% Section 1: Use The Exact Visibility Route

route_units = double(request.Seed.position_units);
route_units([1 end], :) = [request.InitialState.position_units; request.GoalState.position_units];
segmentCount = size(route_units, 1) - 1;
regionActiveBySegment = true(segmentCount, numel(request.Regions_units));

%% Section 2: Create Linear Rest-To-Rest Controls

degree = request.Degree;
fraction = reshape(min(1, max(0, ((0:degree) - 2) / (degree - 4))), 1, [], 1);
start_units = reshape(route_units(1:end - 1, :), segmentCount, 1, 2);
finish_units = reshape(route_units(2:end, :), segmentCount, 1, 2);
controlPoint_units = (1 - fraction) .* start_units + fraction .* finish_units;
segmentTime_s = bmtpEngine.findRequiredSegmentTime(controlPoint_units, request.Limits);
originalSegmentCount = segmentCount;
if request.SplitCount>1
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
warmStart.WarmRouteResampled = false;
if isfield(request.Coverage,'BreakTime_s') && request.Options.GoalTimeMode~="fixedArrival"
    sourceBreaks_s = request.Coverage.BreakTime_s;
    % Natural obstacle events already supply phases on a detailed clock.
    % Keep at least eight spans (up to four per interval) on sparse clocks
    % so the endpoint constraints do not consume the steering freedom.
    subdivisions = min(4,max(1,ceil(8/(numel(sourceBreaks_s)-1))));
    breakTime_s = unique(reshape(sourceBreaks_s(1:end-1)+diff(sourceBreaks_s)*(0:subdivisions)/subdivisions,[],1));
    segmentTime_s = diff(breakTime_s);
    segmentCount = numel(segmentTime_s);
    tau = (breakTime_s(1:end-1)-request.InitialState.time_s + ...
        segmentTime_s.*((0:degree)/degree))/request.MotionHorizon_s;
    controls = interp1(request.Seed.tau,route_units,tau(:),'linear');
    controlPoint_units = reshape(controls,segmentCount,degree+1,2);
    controlPoint_units(1,1:3,:) = reshape(repmat(request.InitialState.position_units,3,1),1,3,2);
    controlPoint_units(end,end-2:end,:) = reshape(repmat(request.GoalState.position_units,3,1),1,3,2);
    intervals_s = request.Coverage.ActiveTimeInterval_s;
    warmStart.ControlPoint_units = controlPoint_units;
    warmStart.SegmentTime_s = segmentTime_s(:);
    warmStart.SegmentRatio = segmentTime_s/mean(segmentTime_s);
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
    isTimedSeed = isfield(request.Seed,'Source') && ...
        string(request.Seed.Source)=="timeExpandedVisibilityGraph";
    if isTimedSeed
        routeTau = linspace(0,1,segmentCount+1).';
        timedRoute_units = interp1(request.Seed.tau,route_units,routeTau,'linear');
        start_units = reshape(timedRoute_units(1:end-1,:),segmentCount,1,2);
        finish_units = reshape(timedRoute_units(2:end,:),segmentCount,1,2);
        warmStart.ControlPoint_units = (1-fraction).*start_units+fraction.*finish_units;
        warmStart.Route_units = timedRoute_units;
    else
        tau=((0:segmentCount-1).'+(0:degree)/degree)/segmentCount;
        controls=interp1(request.Seed.tau,route_units,tau(:),'linear');
        warmStart.ControlPoint_units=reshape(controls,segmentCount,degree+1,2);
    end
    warmStart.SegmentTime_s=repmat(request.MotionHorizon_s/segmentCount,segmentCount,1);
    warmStart.SegmentRatio=ones(segmentCount,1);
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
if isfield(request.Seed, 'Source') && ...
        string(request.Seed.Source) == "timeExpandedVisibilityGraph"
    commonSegmentTime_s = max(bmtpEngine.findRequiredSegmentTime( ...
        warmStart.ControlPoint_units, request.Limits));
    warmStart.SegmentTime_s = repmat(commonSegmentTime_s, warmStart.SegmentCount, 1);
    warmStart.SegmentRatio = ones(warmStart.SegmentCount, 1);
    warmStart.Duration_s = warmStart.SegmentCount * commonSegmentTime_s;
end
end
