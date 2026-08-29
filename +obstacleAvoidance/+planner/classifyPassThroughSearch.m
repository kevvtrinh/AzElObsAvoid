function classification = classifyPassThroughSearch( ...
        position_deg, constraintTolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   classification = ...
%       obstacleAvoidance.planner.classifyPassThroughSearch( ...
%       position_deg, constraintTolerance_deg)
%**************************************************************************
% PURPOSE
%   - Classify a waypoint chain for the shared-state Ruckig search so profile
%     probing and full refinement use one geometric invariant.
%**************************************************************************
% INPUTS
%   - position_deg (N-by-2 finite numeric array)
%       Ordered azimuth/elevation route points with N at least two.
%   - constraintTolerance_deg (nonnegative finite numeric scalar)
%       Tolerance used to distinguish active and globally monotone axes.
%**************************************************************************
% OUTPUTS
%   - classification (scalar struct)
%       Mode is oneDimensionalScalar, coupledMonotone, or coupledReversing;
%       ActiveAxisIndex retains the axes used by waypoint-state search.
%**************************************************************************
% UNITS
%   - Positions and constraintTolerance_deg are degrees. Axis indices and
%     the returned mode are dimensionless.
%**************************************************************************

%% Section 1: Validate Inputs

validateattributes(position_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2, 'nonempty'});
if size(position_deg, 1) < 2
    error("classifyPassThroughSearch:InsufficientRoutePoints", ...
        "position_deg must contain at least two route points; observed %d.", ...
        size(position_deg, 1));
end
validateattributes(constraintTolerance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
position_deg = double(position_deg);

%% Section 2: Classify Route Geometry

activeAxisTolerance_deg = max(1e-10, constraintTolerance_deg);
edgeDelta_deg = diff(position_deg, 1, 1);
routeRange_deg = max(position_deg, [], 1) - min(position_deg, [], 1);
isNondecreasingAxis = all( ...
    edgeDelta_deg >= -activeAxisTolerance_deg, 1);
isNonincreasingAxis = all( ...
    edgeDelta_deg <= activeAxisTolerance_deg, 1);
isGloballyMonotoneAxis = ...
    (isNondecreasingAxis | isNonincreasingAxis) & ...
    routeRange_deg > activeAxisTolerance_deg;
globallyMonotoneAxisIndex = find(isGloballyMonotoneAxis);
activeAxisIndex = find(routeRange_deg > activeAxisTolerance_deg);
if isscalar(globallyMonotoneAxisIndex)
    mode = "oneDimensionalScalar";
elseif isempty(globallyMonotoneAxisIndex)
    mode = "coupledReversing";
else
    mode = "coupledMonotone";
end
classification = struct("Mode", mode, "ActiveAxisIndex", activeAxisIndex);
end
