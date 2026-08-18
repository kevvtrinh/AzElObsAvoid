function position_deg = sampleAzElSeedRoute( ...
        seedRoute_deg, queryTau, initialTime_s, finalTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   position_deg = azElInternal.sampleAzElSeedRoute( ...
%       seedRoute_deg, queryTau, initialTime_s, finalTime_s, options)
%**************************************************************************
% PURPOSE
%   - Sample a geometric or timed route law at normalized motion times.
%**************************************************************************
% INPUTS
%   - seedRoute_deg (N-by-2 numeric matrix)
%       Ordered azimuth and elevation route vertices.
%   - queryTau (M-by-1 numeric vector)
%       Normalized motion times to sample.
%   - initialTime_s, finalTime_s (finite numeric scalars)
%       Absolute motion interval.
%   - options (scalar struct)
%       Optional SeedRouteTimeFraction, SeedRouteProgress, and
%       SeedSnapshotTime_s fields.
%**************************************************************************
% OUTPUTS
%   - position_deg (M-by-2 numeric matrix)
%       Interpolated azimuth and elevation positions.
%**************************************************************************
% UNITS
%   - Position is degrees. Time is seconds. queryTau is dimensionless.
%**************************************************************************

%% Section 1: Build The Route Arc Coordinate

route_deg = double(seedRoute_deg);
keepPoint = [true; any(abs(diff(route_deg, 1, 1)) > 1e-12, 2)];
route_deg = route_deg(keepPoint, :);
routeStep_deg = diff(route_deg, 1, 1);
routeArc_deg = [0; cumsum(vecnorm(routeStep_deg, 2, 2))];
if routeArc_deg(end) <= eps
    route_deg = [route_deg(1, :); route_deg(end, :)];
    routeFraction = [0; 1];
else
    routeFraction = routeArc_deg / routeArc_deg(end);
end

%% Section 2: Apply The Available Time Law

routeQueryFraction = queryTau(:);
hasTimedRoute = isfield(options, "SeedRouteTimeFraction") && ...
    isfield(options, "SeedRouteProgress") && ...
    numel(options.SeedRouteTimeFraction) >= 2 && ...
    numel(options.SeedRouteTimeFraction) == ...
    numel(options.SeedRouteProgress) && ...
    all(diff(options.SeedRouteTimeFraction) > 0) && ...
    all(diff(options.SeedRouteProgress) >= 0);
if hasTimedRoute
    routeQueryFraction = interp1( ...
        options.SeedRouteTimeFraction, options.SeedRouteProgress, ...
        routeQueryFraction, "linear", "extrap");
    routeQueryFraction = min(1, max(0, routeQueryFraction));
elseif isfield(options, "SeedSnapshotTime_s") && ...
        isfinite(options.SeedSnapshotTime_s)
    snapshotFraction = (options.SeedSnapshotTime_s - initialTime_s) / ...
        (finalTime_s - initialTime_s);
    if snapshotFraction > 1e-6 && snapshotFraction < 1 - 1e-6
        % The graph has no time at each vertex. Put its geometric midpoint
        % at the source snapshot so equal routes can seed different laws.
        routeQueryFraction = interp1([0; snapshotFraction; 1], ...
            [0; 0.5; 1], routeQueryFraction, "linear");
    end
end
position_deg = interp1(routeFraction, route_deg, ...
    routeQueryFraction, "linear", "extrap");
end
