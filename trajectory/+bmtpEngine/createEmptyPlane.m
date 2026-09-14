function plane = createEmptyPlane()
%% Section 0: Header & Readme
% SYNTAX: plane = bmtpEngine.createEmptyPlane()
% PURPOSE: Create the one inactive separating-plane record shared by every solver, certificate, and
%   verifier so a plane written by one stage is assignable into any other stage's plane array.
% INPUTS: None.
% OUTPUTS: plane (scalar struct) Active and Verified state, construction ExitFlag, the two endpoint
%   Normal rows, their Offset_units, the certified SignedGap_units, and the closed normalized span
%   TimeFraction the plane constrains.
% UNITS: Offsets and gaps are coordinate units; normals and TimeFraction are dimensionless.

%% Section 1: Return The Complete Inactive Record
plane = struct('Active',false,'Verified',false,'ExitFlag',NaN, ...
    'Normal',zeros(2,2),'Offset_units',zeros(1,2), ...
    'SignedGap_units',NaN,'TimeFraction',[0,1]);
end
