function [plane, exitFlag, output] = solveSeparatingLine(controlPoint_units, vertices_units, target_units, reserve_units, obstacleGeometry)
%% Section 0: Header & Readme
% SYNTAX: [plane,exitFlag,output] = bmtpEngine.solveSeparatingLine(controls,vertices,target,reserve,geometry)
% PURPOSE: Compute a convex supporting plane and verify its exact Bernstein
%          clearance. Overlap selects the least-penetrating nonzero axis for
%          the elastic trajectory subproblem; it is never marked verified.
% INPUTS: Bezier control hull, N-by-2 static obstacle vertices or N-by-2-by-2
%         affine endpoint vertices, clearance and reserve. Optional geometry
%         caches obstacle-edge normals/supports for these exact vertices.
% OUTPUTS: Supporting plane, construction status, and analytic diagnostics.
% UNITS: Coordinate units.

%% Section 1: Evaluate Convex Supporting Axes
first_units = vertices_units(:,:,1);
last_units = vertices_units(:,:,end);
fraction = (0:size(controlPoint_units,1)-1)'/(size(controlPoint_units,1)-1);
relativeControl_units = controlPoint_units-fraction.*(mean(last_units,1)-mean(first_units,1));
if nargin<5 || isempty(obstacleGeometry)
    edges_units = diff([first_units;first_units(1,:)],1,1);
    if size(vertices_units,3)>1
        edges_units = [edges_units;diff([last_units;last_units(1,:)],1,1)];
    end
    [secondControl,firstControl] = find(tril(true(size(controlPoint_units,1)),-1));
    edges_units = [edges_units;relativeControl_units(secondControl,:)-relativeControl_units(firstControl,:)];
    length_units = vecnorm(edges_units,2,2);
    edges_units = edges_units(length_units>0,:);
    length_units = length_units(length_units>0);
    normals = [-edges_units(:,2),edges_units(:,1)]./length_units;
    normals = [normals;-normals];
    firstSupport_units = min(first_units*normals.',[],1);
    lastSupport_units = min(last_units*normals.',[],1);
else
    positiveObstacleNormals = obstacleGeometry.PositiveNormals;
    firstObstaclePositive_units = obstacleGeometry.FirstPositiveSupport_units;
    lastObstaclePositive_units = obstacleGeometry.LastPositiveSupport_units;
    firstObstacleNegative_units = obstacleGeometry.FirstNegativeSupport_units;
    lastObstacleNegative_units = obstacleGeometry.LastNegativeSupport_units;
    [secondControl,firstControl] = find(tril(true(size(controlPoint_units,1)),-1));
    controlEdges_units = relativeControl_units(secondControl,:)-relativeControl_units(firstControl,:);
    controlLength_units = vecnorm(controlEdges_units,2,2);
    controlEdges_units = controlEdges_units(controlLength_units>0,:);
    controlLength_units = controlLength_units(controlLength_units>0);
    positiveControlNormals = [-controlEdges_units(:,2),controlEdges_units(:,1)]./controlLength_units;
    positiveNormals = [positiveObstacleNormals;positiveControlNormals];
    normals = [positiveNormals;-positiveNormals];
    firstControlProjection_units = first_units*positiveControlNormals.';
    lastControlProjection_units = last_units*positiveControlNormals.';
    firstControlPositive_units = min(firstControlProjection_units,[],1);
    lastControlPositive_units = min(lastControlProjection_units,[],1);
    firstControlNegative_units = -max(firstControlProjection_units,[],1);
    lastControlNegative_units = -max(lastControlProjection_units,[],1);
    firstSupport_units = [firstObstaclePositive_units,firstControlPositive_units, ...
        firstObstacleNegative_units,firstControlNegative_units];
    lastSupport_units = [lastObstaclePositive_units,lastControlPositive_units, ...
        lastObstacleNegative_units,lastControlNegative_units];
end
supportDifference_units = controlPoint_units*normals.'-(1-fraction).*firstSupport_units-fraction.*lastSupport_units;
gaps_units = -max(supportDifference_units,[],1);
% Rank supporting directions by the original hull, but retain every direction
% certified by the exact degree-D by degree-one product used by the verifier.
beta = (0:size(controlPoint_units,1))'/size(controlPoint_units,1);
productGaps_units = -max((1-beta).*[supportDifference_units;zeros(1,size(normals,1))]+ ...
    beta.*[zeros(1,size(normals,1));supportDifference_units],[],1);
certifiable = productGaps_units>=target_units+reserve_units;
if any(certifiable), gaps_units(~certifiable) = -Inf; end
[gap_units,index] = max(gaps_units);
plane = struct('Active',false,'Verified',false,'ExitFlag',-2, ...
    'Normal',zeros(2,2),'Offset_units',zeros(1,2),'SignedGap_units',NaN);
exitFlag = -2;
output = struct('TotalTime_s',0,'IsAnalytic',true,'message','Convex supporting-axis subproblem.');
if isempty(gap_units), return; end

%% Section 2: Verify The Proposed Separation
plane.Active = true; plane.ExitFlag = 1; exitFlag = 1;
plane.Normal = repmat(normals(index,:),2,1);
plane.Offset_units = target_units-[firstSupport_units(index),lastSupport_units(index)];
plane = bmtpEngine.verifySeparatingLine(plane,controlPoint_units,vertices_units,reserve_units,target_units);
end
