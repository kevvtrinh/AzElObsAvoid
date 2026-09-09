function [warmStart,record] = lookupC3Profile(request,warmStart)
%% Section 0: Header & Readme
% SYNTAX: [warm,record] = bmtpEngine.lookupC3Profile(request,warm)
% PURPOSE: Transfer a normalized profile and deform its guide to current geometry.
% INPUTS: Checked request with a loaded library and ordinary visibility warm start.
% OUTPUTS: Solver guess and retrieval diagnostics. The guess is not certified.
% UNITS: Library coordinates and clocks are normalized; returned data are physical.

%% Section 1: Select A Compatible Shape And Dynamic Regime
timer=tic;
record=struct('Matched',false,'EntryIndex',0,'RouteError',Inf,'LimitError',Inf, ...
    'Reflected',false,'Mode',request.Options.C3ProfileMode,'Reason',"noCompatibleProfile", ...
    'LookupTime_s',0,'Attempted',false,'Accepted',false,'AttemptSucceeded',false, ...
    'AttemptReason',"notAttempted",'AttemptSolverMessage',"",'AttemptConicCalls',0, ...
    'AttemptTime_s',0,'AttemptArrival_s',NaN,'AttemptLength_units',NaN,'FallbackUsed',false,'FallbackTime_s',0, ...
    'Shortlist',zeros(0,4),'InitialTrials',struct([]));
if ~request.IsRest || request.Options.GoalTimeMode~="earliestArrival" || ...
        isfield(request.Coverage,'ActiveTimeInterval_s') || size(warmStart.Route_units,1)<=2
    record.Reason="ordinarySolverPreferred";
    record.LookupTime_s=toc(timer);
    return;
end
descriptor=bmtpEngine.describeC3Profile(warmStart.Route_units,request.Limits);
library=request.Options.C3ProfileLibrary;
matches=zeros(0,5);
% Shape and regime radii screen transfer distance; physical tolerances remain
% unchanged. The conic stage ranks the shortlist using current-scene timing.
for index=1:numel(library.Entries)
    entry=library.Entries{index};
    if ~entry.RestEndpoints || size(entry.Route,1)~=size(descriptor.Route,1), continue; end
    limitError=max(abs(log(entry.LimitSignature./descriptor.LimitSignature)),[],'all');
    if limitError>log(2), continue; end
    bestError=Inf; reflected=false;
    for reflection=[1,-1]
        routeError=max(vecnorm(descriptor.RouteSamples-entry.RouteSamples.*[1,reflection],2,2));
        if routeError<bestError, bestError=routeError; reflected=reflection<0; end
    end
    if bestError<=0.15
        matches(end+1,:)=[index,bestError,limitError,reflected,bestError+0.05*limitError];
    end
end
if isempty(matches), record.LookupTime_s=toc(timer); return; end
matches=sortrows(matches,5);
matches=matches(1:min(4,size(matches,1)),:);
record.Matched=true;
record.Reason="compatibleProfile";
record.Shortlist=matches(:,1:4);
record.EntryIndex=matches(1,1); record.RouteError=matches(1,2);
record.LimitError=matches(1,3); record.Reflected=logical(matches(1,4));
alternatives=cell(size(matches,1),1);
for k=1:size(matches,1)
    alternatives{k}=transferEntry(request,warmStart,descriptor,library.Entries{matches(k,1)},logical(matches(k,4)));
    alternatives{k}.ProfileEntryIndex=matches(k,1);
end
warmStart=alternatives{1};
warmStart.ProfileAlternatives=alternatives(2:end);
record.LookupTime_s=toc(timer);
end

function warmStart=transferEntry(request,warmStart,descriptor,entry,reflected)
    %% Section 2: Transfer The Polynomial And Its Phase Clock
    origin_units=descriptor.Origin_units;
    distance_units=descriptor.Distance_units;
    orientation=diag([1,1-2*reflected])*descriptor.Frame.';
    power_units=entry.PositionPower;
    for order=1:6, power_units(:,:,order)=distance_units*power_units(:,:,order)*orientation; end
    power_units(:,:,1)=power_units(:,:,1)+origin_units;
    conversion=zeros(6);
    for k=0:5
        for j=0:k, conversion(k+1,j+1)=nchoosek(k,j)/nchoosek(5,j); end
    end
    controls_units=permute(pagemtimes(conversion,permute(power_units,[3,2,1])),[3,1,2]);
    duration_s=max(bmtpEngine.findRequiredSegmentTime(controls_units,request.Limits)./entry.SegmentTime);
    polynomial=bmtpEngine.createPowerPolynomial(controls_units,entry.SegmentTime*duration_s,0,power_units);
    count=warmStart.SegmentCount;
    phaseTime=entry.PhaseTime;
    if request.Options.C3ProfileMode=="repair"
        phaseTime=entry.CompactPhaseTime;
        count=numel(phaseTime);
        warmStart.SegmentCount=count;
        warmStart.RegionActiveBySegment=true(count,numel(request.Regions_units));
        warmStart.ControlPoint_units=zeros(count,6,2);
        warmStart.WarmRouteResampled=true;
    end
    breaks_s=duration_s*interp1(linspace(0,1,numel(phaseTime)+1), ...
        [0;cumsum(phaseTime)],linspace(0,1,count+1)).';
    warmStart.SegmentTime_s=diff(breaks_s);
    warmStart.Duration_s=breaks_s(end);
    warmStart.SegmentRatio=diff(breaks_s)/mean(diff(breaks_s));
    warmStart.ProfileMode=request.Options.C3ProfileMode;
    [~,positions_units,velocities_units_s,accelerations_units_s2]=bmtpEngine.evaluatePolynomial(polynomial,breaks_s);

    %% Section 3: Adapt The Curve To The Current Visibility Route
    % Hermite sampling initializes a new grid. The subsequent conic generation
    % imposes C3, current obstacle planes, endpoint states, and physical limits.
    reference_units=distance_units*entry.Route*orientation+origin_units;
    correction_units=warmStart.Route_units-reference_units;
    starts_units=reference_units(1:end-1,:);
    edges_units=diff(reference_units);
    for span=1:count
        h_s=breaks_s(span+1)-breaks_s(span);
        local=[positions_units(span,:);h_s*velocities_units_s(span,:); ...
            h_s^2*accelerations_units_s2(span,:)/2;zeros(3,2)];
        residual=[positions_units(span+1,:)-sum(local,1); ...
            h_s*velocities_units_s(span+1,:)-local(2,:)-2*local(3,:); ...
            h_s^2*accelerations_units_s2(span+1,:)-2*local(3,:)];
        local(4:6,:)=[1,1,1;3,4,5;6,12,20]\residual;
        controls_units=conversion*local;
        for k=1:6
            point_units=controls_units(k,:);
            progress=max(0,min(1,sum((point_units-starts_units).*edges_units,2)./sum(edges_units.^2,2)));
            [~,edge]=min(sum((point_units-(starts_units+progress.*edges_units)).^2,2));
            controls_units(k,:)=point_units+(1-progress(edge))*correction_units(edge,:)+progress(edge)*correction_units(edge+1,:);
        end
        warmStart.ControlPoint_units(span,:,:)=controls_units;
    end
    warmStart.ControlPoint_units=bmtpEngine.imposeEndpointControls(warmStart.ControlPoint_units, ...
        warmStart.SegmentTime_s,request.InitialState,request.GoalState);
end
