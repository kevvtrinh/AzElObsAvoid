function warmStart = createReachabilityWarmStart(request)
%% Section 0: Header & Readme
% SYNTAX: warm = bmtpEngine.createReachabilityWarmStart(request)
% PURPOSE: Construct a common physical clock at the independent-axis time bound.
% INPUTS: Validated rest-to-rest BMTP request and authoritative time cells.
% OUTPUTS: Exact phase-refined controls, durations, and time-limiting axis locks.
% UNITS: Coordinate units and seconds.

%% Section 1: Solve The Independent Axis Profiles

degree = request.Degree;
initial_units = request.InitialState.position_units;
goal_units = request.GoalState.position_units;
axisControls_units = cell(1,2); axisTime_s = cell(1,2);
axisPhases = cell(1,2);
minimumTime_s = zeros(1,2);
for axis = 1:2
    if initial_units(axis)==goal_units(axis), continue; end
    limits = struct('maxVelocity_units_s',repmat(request.Limits.maxVelocity_units_s(axis),1,2), ...
        'maxAcceleration_units_s2',repmat(request.Limits.maxAcceleration_units_s2(axis),1,2), ...
        'maxJerk_units_s3',repmat(request.Limits.maxJerk_units_s3(axis),1,2));
    [controls_units,durations_s,axisPhases{axis}] = bmtpEngine.createJerkLimitedChord([initial_units(axis),0],[goal_units(axis),0],limits,degree);
    axisControls_units{axis} = controls_units;
    axisTime_s{axis} = durations_s;
    minimumTime_s(axis) = sum(durations_s);
end
[duration_s,lockedAxis] = max(minimumTime_s);
guide = bmtpEngine.createClockGuide(request,axisPhases{lockedAxis},lockedAxis);

%% Section 2: Refine At All Axis Switches And Absolute Obstacle Events

breaks_s = [0;duration_s];
if isfield(request.Coverage,'ActiveTimeInterval_s')
    events_s = request.Coverage.ActiveTimeInterval_s(:)-request.InitialState.time_s;
    breaks_s = unique([breaks_s;events_s(events_s>0 & events_s<duration_s)]);
end
axisBreaks_s = cell(1,2);
for axis = 1:2
    if minimumTime_s(axis)==0, continue; end
    axisTime_s{axis} = axisTime_s{axis}*duration_s/minimumTime_s(axis);
    axisBreaks_s{axis} = [0;cumsum(axisTime_s{axis})];
    axisBreaks_s{axis}(end) = duration_s;
    for event_s = axisBreaks_s{axis}(2:end-1).'
        if min(abs(breaks_s-event_s))>64*eps(duration_s)
            breaks_s(end+1,1) = event_s; %#ok<AGROW>
        end
    end
end
guideTimes_s = zeros(size(guide.Route_units,1),1);
if guide.IsConnected
    phases = axisPhases{lockedAxis};
    direction = sign(goal_units(lockedAxis)-initial_units(lockedAxis));
    phaseEnds_units = [phases.Position_units(2:end,1);goal_units(lockedAxis)];
    for k = 2:size(guide.Route_units,1)-1
        coordinate_units = guide.Route_units(k,lockedAxis);
        phase = find(direction*phaseEnds_units>=direction*coordinate_units,1);
        lo_s = 0; hi_s = phases.SegmentTime_s(phase);
        for refinement = 1:48
            middle_s = (lo_s+hi_s)/2;
            position_units = phases.Position_units(phase,1)+phases.Velocity_units_s(phase,1)*middle_s+ ...
                phases.Acceleration_units_s2(phase,1)*middle_s^2/2+phases.Jerk_units_s3(phase,1)*middle_s^3/6;
            if direction*position_units<direction*coordinate_units, lo_s=middle_s; else, hi_s=middle_s; end
        end
        guideTimes_s(k) = phases.StartTime_s(phase)+(lo_s+hi_s)/2;
        if min(abs(breaks_s-guideTimes_s(k)))>64*eps(duration_s)
            breaks_s(end+1,1) = guideTimes_s(k); %#ok<AGROW>
        end
    end
    guideTimes_s(end) = duration_s;
end
breaks_s = sort(breaks_s);
segmentTime_s = diff(breaks_s);
segmentCount = numel(segmentTime_s);
controlPoint_units = zeros(segmentCount,degree+1,2);
for axis = 1:2
    if minimumTime_s(axis)==0
        controlPoint_units(:,:,axis) = initial_units(axis);
        continue;
    end
    for k = 1:segmentCount
        source = find(axisBreaks_s{axis}<(breaks_s(k)+breaks_s(k+1))/2,1,'last');
        interval = (breaks_s(k:k+1).'-axisBreaks_s{axis}(source))/axisTime_s{axis}(source);
        restricted = bmtpEngine.restrictBezier(squeeze(axisControls_units{axis}(source,:,:)),max(0,min(1,interval)));
        controlPoint_units(k,:,axis) = restricted(:,1);
    end
end
if guide.IsConnected && minimumTime_s(3-lockedAxis)<duration_s
    freeAxis = 3-lockedAxis;
    direction = sign(goal_units(lockedAxis)-initial_units(lockedAxis));
    for k = 1:segmentCount
        coordinate_units = mean(controlPoint_units(k,[1,end],lockedAxis));
        edge = find(direction*guide.Route_units(:,lockedAxis)<direction*coordinate_units,1,'last');
        fraction = (controlPoint_units(k,:,lockedAxis)-guide.Route_units(edge,lockedAxis))/ ...
            diff(guide.Route_units(edge:edge+1,lockedAxis));
        controlPoint_units(k,:,freeAxis) = guide.Route_units(edge,freeAxis)+fraction*diff(guide.Route_units(edge:edge+1,freeAxis));
    end
end

%% Section 3: Retain Exact Time-Limiting Motion And Cell Applicability

fixedControl_units = NaN(size(controlPoint_units));
fixedControl_units(:,:,minimumTime_s==duration_s) = controlPoint_units(:,:,minimumTime_s==duration_s);
fixedPower_units = NaN(segmentCount,2,degree+1);
for axis = find(minimumTime_s==duration_s)
    phases = axisPhases{axis};
    for k = 1:segmentCount
        source = find(phases.StartTime_s<(breaks_s(k)+breaks_s(k+1))/2,1,'last');
        local_s = breaks_s(k)-phases.StartTime_s(source);
        jerk_units_s3 = phases.Jerk_units_s3(source,1);
        acceleration_units_s2 = phases.Acceleration_units_s2(source,1)+jerk_units_s3*local_s;
        velocity_units_s = phases.Velocity_units_s(source,1)+phases.Acceleration_units_s2(source,1)*local_s+jerk_units_s3*local_s^2/2;
        position_units = phases.Position_units(source,1)+phases.Velocity_units_s(source,1)*local_s+ ...
            phases.Acceleration_units_s2(source,1)*local_s^2/2+jerk_units_s3*local_s^3/6;
        fixedPower_units(k,axis,:) = 0;
        fixedPower_units(k,axis,1:4) = [position_units,velocity_units_s*segmentTime_s(k), ...
            acceleration_units_s2*segmentTime_s(k)^2/2,jerk_units_s3*segmentTime_s(k)^3/6];
    end
end
active = true(segmentCount,numel(request.Regions_units));
if isfield(request.Coverage,'ActiveTimeInterval_s')
    intervals_s = request.Coverage.ActiveTimeInterval_s-request.InitialState.time_s;
    active = breaks_s(1:end-1)<intervals_s(:,2).' & breaks_s(2:end)>intervals_s(:,1).';
end
warmStart = struct('Route_units',request.Seed.position_units,'ControlPoint_units',controlPoint_units, ...
    'SegmentTime_s',segmentTime_s,'Duration_s',duration_s,'SegmentRatio',segmentTime_s/mean(segmentTime_s), ...
    'SegmentCount',segmentCount,'RegionActiveBySegment',active,'OriginalSeedSegmentCount',1, ...
    'WarmRouteResampled',true,'FixedControl_units',fixedControl_units,'AxisMinimumTime_s',minimumTime_s, ...
    'ClockGuide',guide,'GuideTimes_s',guideTimes_s,'FixedPower_units',fixedPower_units);
end
