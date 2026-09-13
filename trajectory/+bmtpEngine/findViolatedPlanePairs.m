function [selectedPairs,maximumResidual]=findViolatedPlanePairs(x,planes, ...
        activePairs,retainedPairs,degree,slackColumnByPair, ...
        trajectoryReserve_units,tolerance)
%% Section 0: Header & Readme
% SYNTAX: [pairs,maxResidual] = bmtpEngine.findViolatedPlanePairs(...)
% PURPOSE: Separate omitted fixed-plane inequalities exactly and select the
%   greatest violation per motion span for the next constraint round.
% INPUTS: Current decision vector, complete planes/applicability, retained
%   mask, degree, optional slack columns, reserve, and conic tolerance.
% OUTPUTS: Selected violated pairs and maximum residual over every omission.
% UNITS: Coordinate units.

%% Section 1: Evaluate Every Omitted Pair In Physical Control Coordinates
selectedPairs=false(size(activePairs));
maximumResidual=-Inf;
greatestViolationBySegment=-Inf(size(activePairs,1),1);
greatestRegionBySegment=zeros(size(activePairs,1),1);
alpha=1-(0:degree+1).'/(degree+1);
beta=1-alpha;
controls=permute(reshape(x(1:size(planes,1)*(degree+1)*2), ...
    2,degree+1,size(planes,1)),[3,2,1]);
lastTimeFraction=NaN(size(planes,1),2);
lastRestrictedControl=cell(size(planes,1),1);
for pairIndex=reshape(find(activePairs & ~retainedPairs),1,[])
    [segmentIndex,regionIndex]=ind2sub(size(activePairs),pairIndex);
    plane=planes(segmentIndex,regionIndex);
    pairControls=squeeze(controls(segmentIndex,:,:));
    if isfield(plane,'TimeFraction') && ~isequal(plane.TimeFraction,[0,1])
        if isequal(lastTimeFraction(segmentIndex,:),plane.TimeFraction)
            pairControls=lastRestrictedControl{segmentIndex};
        else
            pairControls=bmtpEngine.restrictBezier(pairControls,plane.TimeFraction);
            lastTimeFraction(segmentIndex,:)=plane.TimeFraction;
            lastRestrictedControl{segmentIndex}=pairControls;
        end
    end
    product=alpha.*[pairControls*plane.Normal(1,:).';0]+ ...
        beta.*[0;pairControls*plane.Normal(2,:).'];
    offsets=alpha*plane.Offset_units(1)+beta*plane.Offset_units(2);
    slack=0;
    slackColumn=slackColumnByPair(segmentIndex,regionIndex);
    if slackColumn>0, slack=x(slackColumn); end
    pairResidual=max(product+offsets-slack+trajectoryReserve_units);
    maximumResidual=max(maximumResidual,pairResidual);
    if pairResidual>tolerance && ...
            pairResidual>greatestViolationBySegment(segmentIndex)
        greatestViolationBySegment(segmentIndex)=pairResidual;
        greatestRegionBySegment(segmentIndex)=regionIndex;
    end
end
for segmentIndex=reshape(find(greatestRegionBySegment>0),1,[])
    selectedPairs(segmentIndex,greatestRegionBySegment(segmentIndex))=true;
end
end
