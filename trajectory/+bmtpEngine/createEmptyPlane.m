function plane = createEmptyPlane()
%% Section 0: Header & Readme
% SYNTAX
%   plane = bmtpEngine.createEmptyPlane()
%**************************************************************************
% PURPOSE
%   - Create the shared inactive separating-plane record.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Complete inactive record used by all BMTP plane stages.
%**************************************************************************
% UNITS
%   - Offsets and gaps are coordinate units; TimeFraction is dimensionless.
%**************************************************************************

%% Section 1: Return The Complete Inactive Record
plane = struct( ...
    'Active',          false, ...
    'Verified',        false, ...
    'ExitFlag',        NaN, ...
    'Normal',          zeros(2, 2), ...
    'Offset_units',    zeros(1, 2), ...
    'SignedGap_units', NaN, ...
    'TimeFraction',    [0, 1]);
end
