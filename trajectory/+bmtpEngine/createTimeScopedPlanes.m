function [planes, activePairs, complete, statistics] = createTimeScopedPlanes( ...
        referenceControl_units, segmentTime_s, request, target_units, reserve_units)
%% Section 0: Header & Readme
% SYNTAX: [planes,activePairs,complete,statistics] =
%   bmtpEngine.createTimeScopedPlanes(referenceControls,segmentTimes,request,target,reserve)
% PURPOSE: Build exact separating planes on the physical overlap between each
%   motion span and moving affine obstacle cell.
% INPUTS: Reference Bezier controls, physical span times, checked BMTP request,
%   and obstacle/trajectory separation reserves.
% OUTPUTS: Complete plane array, actual active-pair mask, construction status,
%   and deterministic plane counts.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Resolve The Actual Clock And Pair Activity
segmentCount=size(referenceControl_units,1);
if isscalar(segmentTime_s)
    segmentTime_s=repmat(segmentTime_s,segmentCount,1);
else
    segmentTime_s=double(segmentTime_s(:));
end
validateattributes(segmentTime_s,{'numeric'}, ...
    {'real','finite','positive','numel',segmentCount});
regionCount=numel(request.Regions_units);
planes=repmat(bmtpEngine.createEmptyPlane(),segmentCount,regionCount);
activePairs=true(segmentCount,regionCount);
breaks_s=request.InitialState.time_s+[0;cumsum(segmentTime_s)];
if isfield(request.Coverage,'ActiveTimeInterval_s')
    intervals_s=request.Coverage.ActiveTimeInterval_s;
    activePairs=breaks_s(1:end-1)<intervals_s(:,2).' & ...
        breaks_s(2:end)>intervals_s(:,1).';
else
    intervals_s=zeros(regionCount,2);
end

%% Section 2: Separate Every Pair On Its Exact Physical Interval
complete=true;
verifiedCount=0;
for segmentIndex=1:segmentCount
    for regionIndex=reshape(find(activePairs(segmentIndex,:)),1,[])
        controls_units=squeeze(referenceControl_units(segmentIndex,:,:));
        interval_s=[];
        timeFraction=[0,1];
        geometry=[];
        if isfield(request.Coverage,'ActiveTimeInterval_s')
            interval_s=[max(breaks_s(segmentIndex),intervals_s(regionIndex,1)), ...
                min(breaks_s(segmentIndex+1),intervals_s(regionIndex,2))];
            timeFraction=(interval_s-breaks_s(segmentIndex))/segmentTime_s(segmentIndex);
            timeFraction=max(0,min(1,timeFraction));
            controls_units=bmtpEngine.restrictBezier(controls_units,timeFraction);
        elseif isfield(request,'SeparatingLineGeometry')
            geometry=request.SeparatingLineGeometry{regionIndex};
        end
        vertices_units=bmtpEngine.regionOnInterval(request.Regions_units{regionIndex}, ...
            request.Coverage,regionIndex,interval_s);
        [plane,exitFlag]=bmtpEngine.solveSeparatingLine(controls_units, ...
            vertices_units,target_units,reserve_units,geometry);
        plane.TimeFraction=timeFraction;
        planes(segmentIndex,regionIndex)=plane;
        if exitFlag<=0 || ~plane.Active || ~plane.Verified
            complete=false;
        else
            verifiedCount=verifiedCount+1;
        end
    end
end
statistics=struct('ActivePairCount',nnz(activePairs), ...
    'VerifiedPairCount',verifiedCount);
end
