function azElData = makeAzElObstacleData(obstacleName, time_s, ...
        azimuthBoundary_deg, elevationBoundary_deg, safetyMargin_deg)
%% Section 0: Header & Readme
% SYNTAX
%   azElData = makeAzElObstacleData( ...
%       obstacleName, time_s, azimuthBoundary_deg, elevationBoundary_deg)
%   azElData = makeAzElObstacleData( ...
%       obstacleName, time_s, azimuthBoundary_deg, ...
%       elevationBoundary_deg, safetyMargin_deg)
%**************************************************************************
% PURPOSE
%   - Build validated canonical static or time-varying obstacle data.
%   - Optionally inflate every polygon before publishing the final data.
%**************************************************************************
% INPUTS
%   - obstacleName (scalar text)
%       Obstacle display and diagnostic name.
%   - time_s (numeric vector)
%       Strictly increasing sample times.
%   - azimuthBoundary_deg (numeric vector or cell array)
%       Static boundary or one boundary vector per sample.
%   - elevationBoundary_deg (numeric vector or cell array)
%       Boundary representation matching azimuthBoundary_deg.
%   - safetyMargin_deg (nonnegative scalar, optional; default 0)
%       Euclidean polygon inflation applied here before data is returned.
%**************************************************************************
% OUTPUTS
%   - azElData (scalar struct)
%       Canonical azElData record accepted by planners and packers.
%**************************************************************************
% UNITS
%   - Boundary coordinates are degrees and time_s is seconds.

if nargin < 5 || isempty(safetyMargin_deg)
    safetyMargin_deg = 0;
end
validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});

%% Section 1: Normalize Static & Sampled Boundaries
time_s = double(time_s(:));
sampleCount = numel(time_s);
% Numeric boundaries describe a static polygon and are repeated across the
% supplied time base. Cell inputs preserve independent moving slices.
if ~iscell(azimuthBoundary_deg)
    azimuthBoundary_deg = repmat( ...
        {double(azimuthBoundary_deg(:))}, sampleCount, 1);
end
if ~iscell(elevationBoundary_deg)
    elevationBoundary_deg = repmat( ...
        {double(elevationBoundary_deg(:))}, sampleCount, 1);
end
%% Section 2: Assemble & Validate The Output
azElData = struct( ...
    "targetName", string(obstacleName), ...
    "time_s", time_s, ...
    "az_deg", {reshape(azimuthBoundary_deg, [], 1)}, ...
    "el_deg", {reshape(elevationBoundary_deg, [], 1)}, ...
    "status", repmat("visible", sampleCount, 1));
% Route synthetic examples through the same validator as measured input.
% This keeps test fixtures from relying on shapes the public planner would
% reject in operational data.
azElData = normalizeAzElTimeObstacleData(azElData);
% Inflation belongs to obstacle construction. Packers, collision queries,
% visibility graphs, and planners receive this final geometry and must use
% zero additional safety margin.
if safetyMargin_deg > 0
    azElData = inflateAzElObstacleData(azElData, safetyMargin_deg);
end
end
