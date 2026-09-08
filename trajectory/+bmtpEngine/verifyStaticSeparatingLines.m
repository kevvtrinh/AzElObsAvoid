function planes = verifyStaticSeparatingLines(planes,controlPoint_units,regions_units,reserve_units,target_units)
%% Section 0: Header & Readme
% SYNTAX: planes = bmtpEngine.verifyStaticSeparatingLines(planes,controls,regions,reserve,target)
% PURPOSE: Batch the complete scalar Bernstein plane checks for static cells.
% INPUTS: One plane per convex region, a common N-by-2 Bezier curve, and the
%   same physical target and numerical reserve used by verifySeparatingLine.
% OUTPUTS: Corrected offsets, signed gaps, and independently computed flags.
% UNITS: Coordinate units; normals are dimensionless.

%% Section 1: Evaluate All Obstacle And Curve Product Coefficients
regionCount = numel(regions_units);
assert(numel(planes)==regionCount,'bmtpEngine:InvalidPlaneBatch','Every static source region requires a plane.');
if regionCount==0, return; end
normalPages = reshape([planes.Normal],2,2,regionCount);
firstNormal = reshape(normalPages(1,:,:),2,regionCount).';
lastNormal = reshape(normalPages(2,:,:),2,regionCount).';
offsets_units = reshape([planes.Offset_units],2,regionCount).';
vertices_units = vertcat(regions_units{:});
owners = repelem((1:regionCount).',cellfun(@(v)size(v,1),regions_units));
owners = owners(:);
firstSide_units = sum(vertices_units.*firstNormal(owners,:),2);
lastSide_units = sum(vertices_units.*lastNormal(owners,:),2);
obstacleSide_units = min([firstSide_units+offsets_units(owners,1), ...
    (lastSide_units+firstSide_units+sum(offsets_units(owners,:),2))/2, ...
    lastSide_units+offsets_units(owners,2)],[],2);
minimumObstacle_units = accumarray(owners,obstacleSide_units,[regionCount,1],@min);
degree = size(controlPoint_units,1)-1;
beta = (0:degree+1).'/(degree+1); alpha = 1-beta;
firstProjection_units = controlPoint_units(:,1)*firstNormal(:,1).'+controlPoint_units(:,2)*firstNormal(:,2).';
lastProjection_units = controlPoint_units(:,1)*lastNormal(:,1).'+controlPoint_units(:,2)*lastNormal(:,2).';
product_units = alpha.*[firstProjection_units;zeros(1,regionCount)]+ ...
    beta.*[zeros(1,regionCount);lastProjection_units]+alpha*offsets_units(:,1).'+beta*offsets_units(:,2).';
maximumTrajectory_units = max(product_units,[],1).';
maximumNormalNorm = max([vecnorm(firstNormal,2,2),vecnorm(lastNormal,2,2)],[],2);

%% Section 2: Apply The Same Roundoff Correction And Clearance Checks
minimumCorrection_units = target_units-minimumObstacle_units;
maximumCorrection_units = -reserve_units-maximumTrajectory_units;
scale_units = accumarray(owners,max(abs(vertices_units),[],2),[regionCount,1],@max);
scale_units = max([scale_units,max(abs(offsets_units),[],2), ...
    repmat(max(1,max(abs(controlPoint_units),[],'all')),regionCount,1)],[],2);
roundoff_units = 16*eps(scale_units);
robustMinimum_units = minimumCorrection_units+roundoff_units;
robustMaximum_units = maximumCorrection_units-roundoff_units;
correction_units = zeros(regionCount,1);
possible = minimumCorrection_units<=maximumCorrection_units;
robust = possible & robustMinimum_units<=robustMaximum_units;
correction_units(robust) = min(max(0,robustMinimum_units(robust)),robustMaximum_units(robust));
correction_units(possible & ~robust) = (minimumCorrection_units(possible & ~robust)+maximumCorrection_units(possible & ~robust))/2;
offsets_units = offsets_units+correction_units;
minimumObstacle_units = minimumObstacle_units+correction_units;
maximumTrajectory_units = maximumTrajectory_units+correction_units;
signedGap_units = minimumObstacle_units-maximumTrajectory_units;
normalNormLimit = 1+2^20*eps;
clearanceTarget_units = (target_units-reserve_units)/normalNormLimit;
certifiedClearance_units = (signedGap_units-2*reserve_units)./max(maximumNormalNorm,realmin);
verified = minimumObstacle_units>=target_units & maximumTrajectory_units<=-reserve_units & ...
    signedGap_units>=target_units+reserve_units & certifiedClearance_units>=clearanceTarget_units & maximumNormalNorm<=normalNormLimit;
offsetCells = num2cell(offsets_units,2); gapCells = num2cell(signedGap_units); flags = num2cell(verified);
[planes.Offset_units] = offsetCells{:};
[planes.SignedGap_units] = gapCells{:};
[planes.Verified] = flags{:};
end
