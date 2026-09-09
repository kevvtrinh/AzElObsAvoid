function cones=createVariationCone(jerkMap,times_s,limits,objectiveIndex)
%% Section 0: Header & Readme
% SYNTAX: cones = bmtpEngine.createVariationCone(jerkMap,times_s,limits,indices)
% PURPOSE: Penalize jerk variation within the original motion-generation solve.
% INPUTS: Physical quadratic-jerk control map, phase times, limits, one epigraph
%   index per phase. Jerk rows interleave x/y for each of three phase controls.
% OUTPUTS: Local rotated cones bounding normalized integrated squared snap.
% UNITS: Times in seconds; the objective measure is dimensionless and at most
%   one whenever the original per-axis jerk control bounds hold.

%% Section 1: Integrate The Linear Snap Exactly
count=numel(times_s); derivative=sparse(4*count,6*count);
points=(1+[-1,1]/sqrt(3))/2;
for span=1:count
    local=sqrt(2/times_s(span))*[-(1-points(:)),1-2*points(:),points(:)];
    derivative((span-1)*4+(1:4),(span-1)*6+(1:6))= ...
        kron(local,diag(1./limits.maxJerk_units_s3));
end
% For three jerk controls in [-1,1], the maximum integrated squared
% normalized snap is 16/(3*h) per axis. This gives a bounded tie-break.
normalizer=sqrt((32/3)*sum(1./times_s));
map=derivative*jerkMap/normalizer;
variableCount=size(jerkMap,2);
empty=secondordercone(sparse(5,variableCount),zeros(5,1),sparse(variableCount,1),0);
cones=repmat(empty,count,1);
for span=1:count
    local=[2*map((span-1)*4+(1:4),:);sparse(1,variableCount)];
    local(end,objectiveIndex(span))=1;
    direction=sparse(variableCount,1); direction(objectiveIndex(span))=1;
    cones(span)=secondordercone(local,[zeros(4,1);1],direction,-1);
end
end
