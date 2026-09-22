function plane = createEmptyPlane()
%% Section 0: Header & Readme
% SYNTAX
%   plane = bmtpEngine.separation.createEmptyPlane()
%**************************************************************************
% PURPOSE
%   - Create an unused separating-line record with the fields every BMTP
%     separation stage expects. The stored name "plane" refers to a line
%     separating the curve from an obstacle in this two-axis planner.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Active says a line is available as a constraint; Verified says it
%       passed the curve/obstacle separation checks. Both start false.
%       Normal and Offset_units store the line at the interval start and end:
%       normal x [x; y] + offset = 0. TimeFraction selects the curve portion,
%       where [0 1] means the full segment. SignedGap_units stores the bound
%       on obstacle-side value minus curve-side value; NaN means unmeasured.
%**************************************************************************
% UNITS
%   - Offsets and gaps are coordinate units; TimeFraction is dimensionless.
%**************************************************************************

%% Section 1: Return The Unused Line Record

plane = struct( ...
    'Active',          false, ...
    'Verified',        false, ...
    'ExitFlag',        NaN, ...
    'Normal',          zeros(2, 2), ...
    'Offset_units',    zeros(1, 2), ...
    'SignedGap_units', NaN, ...
    'TimeFraction',    [0, 1]);
end
