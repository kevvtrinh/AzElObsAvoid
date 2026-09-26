function plane = createEmptyPlane()
%% Section 0: Header & Readme
% SYNTAX
%   plane = bmtpEngine.separation.createEmptyPlane()
%**************************************************************************
% PURPOSE
%   - Give every separation stage the same starting record before it finds
%     a line between a motion curve and an obstacle. This two-axis planner
%     calls the record a "plane," but its boundary is a line in x/y.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Active is true once a line is available for a constraint; Verified
%       is true only after its separation check passes. Both start false.
%       Normal has one [x, y] row at each interval end; Offset_units has
%       one value at each end. Together they define the line
%       dot(normal, [x, y]) + offset = 0. Initial zeros are placeholders.
%       TimeFraction selects the curve portion; [0, 1] means all of it.
%       SignedGap_units is a lower bound on obstacle-side minus curve-side
%       values along the normal. A positive bound shows space between them;
%       NaN means no bound has been computed. ExitFlag is an unset solver
%       status.
%**************************************************************************
% UNITS
%   - Offsets and gaps are coordinate units. Normal and TimeFraction have
%     no physical units.
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
