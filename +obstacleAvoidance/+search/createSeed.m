function seed = createSeed()
%% Section 0: Header & Readme
% SYNTAX
%   seed = obstacleAvoidance.search.createSeed()
%**************************************************************************
% PURPOSE
%   - Create the one stable geometric-route seed record used by search,
%     planning, diagnostics, and empty results.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - seed (scalar struct)
%       Stable route record containing documented empty values.
%**************************************************************************
% UNITS
%   - Position, boundary, and length are degrees. Duration is seconds. tau is
%     dimensionless.
%**************************************************************************

%% Section 1: Assemble The Stable Seed

seed = struct( ...
    "Index", 0, "Source", "", ...
    "position_deg", zeros(0, 2), "tau", zeros(0, 1), ...
    "CorridorBoundary_deg", zeros(0, 2), ...
    "UsesReducedGeometry", false, ...
    "MaximumTimedSegmentCount", 0, ...
    "EstimatedDuration_s", NaN, "Length_deg", NaN);
end
