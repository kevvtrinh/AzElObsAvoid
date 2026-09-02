function certificate = checkFinalMotion( ...
        request, warmStart, preparedMotion, roundoffReserve_deg, ...
        obstacleTarget_deg, operations)
%% Section 0: Header & Readme
% SYNTAX
%   certificate = bmtpEngine.checkFinalMotion( ...
%       request, warmStart, preparedMotion, roundoffReserve_deg, ...
%       obstacleTarget_deg, operations)
%**************************************************************************
% PURPOSE
%   - Check every applicable final curve span against each supplied convex
%     obstacle region using direct separating-plane certificates.
%**************************************************************************
% INPUTS
%   - request, warmStart, preparedMotion (scalar structs)
%       Checked request, region applicability, and final prepared curve.
%   - roundoffReserve_deg, obstacleTarget_deg (finite scalars)
%       Numerical reserve and required obstacle-side target in degrees.
%   - operations (scalar struct of function handles)
%       Direct curve-region certificate kernel owned by BMTP solve.
%**************************************************************************
% OUTPUTS
%   - certificate (scalar struct)
%       Pair coverage, separating planes, counts, and passing state.
%**************************************************************************
% UNITS
%   - Position, gaps, and reserves are degrees.
%**************************************************************************

%% Section 1: Check All Curve And Obstacle Pairs

% Splitting each optimized segment creates two output spans, so repeat the
% original applicability mask exactly once to preserve timed-region coverage.
regionActiveBySegment = repelem( ...
    warmStart.RegionActiveBySegment, 2, 1);
certificate = operations.checkAllCurveObstaclePairs( ...
    preparedMotion.CertifiedControlPoint_deg, request.Regions_deg, ...
    request.Coverage, regionActiveBySegment, roundoffReserve_deg, ...
    obstacleTarget_deg, request.TightPlaneOptions);
end
