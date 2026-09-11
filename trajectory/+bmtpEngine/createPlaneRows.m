function [rows, offset_units] = createPlaneRows(plane, degree, variableCount, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX: [rows,offset_units] = bmtpEngine.createPlaneRows(plane,degree,variableCount,segmentIndex)
% PURPOSE: Multiply fixed affine separating planes by variable Bezier controls.
% INPUTS: Plane, degree, variable count and span index; optional exact TimeFraction.
% OUTPUTS: Sparse product rows and Bernstein offsets, in original constraint order.
% UNITS: Coordinate units and seconds.

%% Section 1: Assemble The Exact Constraints
% Exact degree-N by degree-one Bernstein product weights.
beta  = (0:degree + 1).' / (degree + 1);
alpha = 1 - beta;
controlColumns = (segmentIndex-1)*2*(degree+1)+(1:2*(degree+1)).';
rowIndices = [repelem((1:degree+1).',2);repelem((2:degree+2).',2)];
values = [reshape((alpha(1:end-1)*plane.Normal(1,:)).',[],1); ...
    reshape((beta(2:end)*plane.Normal(2,:)).',[],1)];
rows = sparse(rowIndices,[controlColumns;controlColumns],values,degree+2,variableCount);
if isfield(plane,'TimeFraction') && ~isequal(plane.TimeFraction,[0,1])
    % Restrict the unknown controls exactly before forming the existing
    % Bernstein plane product; keep every source interval as a constraint.
    restriction=bmtpEngine.restrictBezier(eye(degree+1),plane.TimeFraction);
    rows(:,controlColumns)=rows(:,controlColumns)*kron(restriction,speye(2));
end
offset_units = alpha * plane.Offset_units(1) + beta * plane.Offset_units(2);
end
