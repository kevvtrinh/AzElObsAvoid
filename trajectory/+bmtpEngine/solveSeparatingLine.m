function [plane, exitFlag, output] = solveSeparatingLine(controlPoint_units, vertices_units, target_units, reserve_units)
%% Section 0: Header & Readme
% SYNTAX: [plane,exitFlag,output] = bmtpEngine.solveSeparatingLine(controls,vertices,target,reserve)
% PURPOSE: Compute a convex supporting plane and verify its exact Bernstein
%          clearance. Overlap selects the least-penetrating nonzero axis for
%          the elastic trajectory subproblem; it is never marked verified.
% INPUTS: Bezier control hull, convex obstacle vertices, clearance and reserve.
% OUTPUTS: Supporting plane, construction status, and analytic diagnostics.
% UNITS: Coordinate units.

%% Section 1: Evaluate Convex Supporting Axes
edges_units = diff([vertices_units;vertices_units(1,:)],1,1);
controlPairs = nchoosek(1:size(controlPoint_units,1),2);
edges_units = [edges_units;controlPoint_units(controlPairs(:,2),:)-controlPoint_units(controlPairs(:,1),:)];
length_units = vecnorm(edges_units,2,2);
edges_units = edges_units(length_units>0,:); length_units = length_units(length_units>0);
normals = [-edges_units(:,2),edges_units(:,1)]./length_units;
normals = [normals;-normals];
gaps_units = min(vertices_units*normals.',[],1)-max(controlPoint_units*normals.',[],1);
[gap_units,index] = max(gaps_units);
plane = struct('Active',false,'Verified',false,'ExitFlag',-2, ...
    'Normal',zeros(2,2),'Offset_units',zeros(1,2),'SignedGap_units',NaN);
exitFlag = -2;
output = struct('TotalTime_s',0,'IsAnalytic',true,'message','Convex supporting-axis subproblem.');
if isempty(gap_units), return; end

%% Section 2: Verify The Proposed Separation
plane.Active = true; plane.ExitFlag = 1; exitFlag = 1;
plane.Normal = repmat(normals(index,:),2,1);
plane.Offset_units = repmat(target_units-min(vertices_units*normals(index,:).'),1,2);
plane = bmtpEngine.verifySeparatingLine(plane,controlPoint_units,vertices_units,reserve_units,target_units);
end
