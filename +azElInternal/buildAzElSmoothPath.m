function smoothPath = buildAzElSmoothPath( ...
        route_deg, obstacleField, collisionTime_s, options, failureMessage)
%% Section 0: Header & Readme
% SYNTAX
%   smoothPath = azElInternal.buildAzElSmoothPath( ...
%       route_deg, obstacleField, collisionTime_s, options)
%   smoothPath = azElInternal.buildAzElSmoothPath( ...
%       route_deg, obstacleField, collisionTime_s, options, failureMessage)
%**************************************************************************
% PURPOSE
%   - Convert one visibility polyline into collision-checked line/quintic
%     geometry independently of any motion-retiming implementation.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 finite numeric, N >= 2)
%       Ordered [azimuth elevation] visibility route.
%   - obstacleField (scalar packed obstacle struct)
%       Protected geometry used to accept or reject each corner blend.
%   - collisionTime_s (finite numeric scalar)
%       Snapshot time at which proposed blend geometry is checked.
%   - options (scalar struct)
%       TurnRadius_deg, CollisionTimePaddingSamples, and related resolved
%       planner settings.
%   - failureMessage (scalar text, optional; default "")
%       A nonempty upstream failure returns the stable unsuccessful schema.
%**************************************************************************
% OUTPUTS
%   - smoothPath (scalar struct)
%       Ordered primitives, arc-length samples, mandatory stops, and corner
%       diagnostics. Invalid contracts throw; geometric failure throws an
%       identified error for the planner to convert into a normal result.
%**************************************************************************
% UNITS
%   - Position and arc length are degrees and time is seconds.
%**************************************************************************

%% Section 1: Validate And Normalize The Polyline

if nargin < 5
    failureMessage = "";
end
failureMessage = string(failureMessage);
if ~isscalar(failureMessage)
    error("buildAzElSmoothPath:InvalidFailureMessage", ...
        "failureMessage must be scalar text.");
end
validateattributes(route_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2});
if strlength(failureMessage) > 0
    smoothPath = emptySmoothPath(route_deg);
    smoothPath.Message = failureMessage;
    return;
end
validateattributes(collisionTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
if ~isstruct(options) || ~isscalar(options) || ...
        ~all(isfield(options, [ ...
        "TurnRadius_deg", "CollisionTimePaddingSamples"]))
    error("buildAzElSmoothPath:InvalidOptions", ...
        "options must contain TurnRadius_deg and " + ...
        "CollisionTimePaddingSamples.");
end
validateattributes(options.TurnRadius_deg, {'numeric'}, ...
    {'real', 'finite', 'positive', 'scalar'});

step_deg = diff(route_deg, 1, 1);
route_deg = double(route_deg([true; ...
    vecnorm(step_deg, 2, 2) > 1e-9], :));
if size(route_deg, 1) < 2
    error("buildAzElSmoothPath:ZeroLengthRoute", ...
        "route_deg must contain at least two distinct points.");
end

%% Section 2: Find Collision-Free Corner Blends

cornerCount = size(route_deg, 1) - 2;
cornerTemplate = struct( ...
    "PathPointIndex", 0, ...
    "Position_deg", zeros(1, 2), ...
    "DeflectionAngle_rad", 0, ...
    "AppliedRadius_deg", 0, ...
    "EntryPosition_deg", zeros(1, 2), ...
    "ExitPosition_deg", zeros(1, 2), ...
    "ControlPoints_deg", zeros(6, 2), ...
    "Smoothed", false, ...
    "MandatoryStop", false, ...
    "Reason", "");
corners = repmat(cornerTemplate, cornerCount, 1);
minimumRadius_deg = min(0.02, options.TurnRadius_deg);

for cornerIndex = 1:cornerCount
    pointIndex = cornerIndex + 1;
    cornerPosition_deg = route_deg(pointIndex, :);
    incomingVector_deg = ...
        cornerPosition_deg - route_deg(pointIndex - 1, :);
    outgoingVector_deg = ...
        route_deg(pointIndex + 1, :) - cornerPosition_deg;
    incomingLength_deg = norm(incomingVector_deg);
    outgoingLength_deg = norm(outgoingVector_deg);
    incomingDirection = incomingVector_deg / incomingLength_deg;
    outgoingDirection = outgoingVector_deg / outgoingLength_deg;
    deflectionAngle_rad = acos(min(1, max(-1, ...
        dot(incomingDirection, outgoingDirection))));
    turnCross = incomingDirection(1) * outgoingDirection(2) - ...
        incomingDirection(2) * outgoingDirection(1);

    diagnostic = cornerTemplate;
    diagnostic.PathPointIndex = pointIndex;
    diagnostic.Position_deg = cornerPosition_deg;
    diagnostic.EntryPosition_deg = cornerPosition_deg;
    diagnostic.ExitPosition_deg = cornerPosition_deg;
    diagnostic.DeflectionAngle_rad = deflectionAngle_rad;

    if deflectionAngle_rad <= 1e-9
        diagnostic.Reason = "collinear";
        corners(cornerIndex) = diagnostic;
        continue;
    end
    if pi - deflectionAngle_rad <= 1e-6 || abs(turnCross) <= 1e-12
        diagnostic.MandatoryStop = true;
        diagnostic.Reason = "unresolved reversal";
        corners(cornerIndex) = diagnostic;
        continue;
    end

    tangentScale = (384 / 125) * sin(deflectionAngle_rad / 2) / ...
        cos(deflectionAngle_rad / 2)^2;
    maximumRadius_deg = 0.45 * min( ...
        incomingLength_deg, outgoingLength_deg) / tangentScale;
    requestedRadius_deg = min(options.TurnRadius_deg, maximumRadius_deg);
    trialRadii_deg = requestedRadius_deg * 0.65 .^ (0:60);
    trialRadii_deg = trialRadii_deg( ...
        trialRadii_deg >= minimumRadius_deg);
    needsMinimumTrial = requestedRadius_deg >= minimumRadius_deg && ...
        (isempty(trialRadii_deg) || ...
        trialRadii_deg(end) > minimumRadius_deg * (1 + eps));
    if needsMinimumTrial
        % At most one endpoint trial is appended to this bounded vector.
        trialRadii_deg(end + 1) = minimumRadius_deg; %#ok<AGROW>
    end

    for radius_deg = trialRadii_deg
        trim_deg = radius_deg * tangentScale;
        % Repeated controls make q'' and q''' vanish at the joins, which
        % lets a nonzero path speed cross a line/blend boundary smoothly.
        controlPoints_deg = [ ...
            cornerPosition_deg - trim_deg * incomingDirection; ...
            cornerPosition_deg - 0.5 * trim_deg * incomingDirection; ...
            cornerPosition_deg; ...
            cornerPosition_deg; ...
            cornerPosition_deg + 0.5 * trim_deg * outgoingDirection; ...
            cornerPosition_deg + trim_deg * outgoingDirection];
        primitive = makeQuinticPrimitive(controlPoints_deg);
        checkCount = max(21, ...
            ceil(primitive.Length_deg / 0.02) + 1);
        checkS_deg = linspace(0, primitive.Length_deg, checkCount).';
        checkParameter = interp1( ...
            primitive.ArcLengthGrid_deg, primitive.ParameterGrid, ...
            checkS_deg, "pchip");
        checkPosition_deg = azElInternal.evaluateAzElQuintic( ...
            controlPoints_deg, min(max(checkParameter, 0), 1));
        blocked = queryAzElTimedPathCollision( ...
            obstacleField, collisionTime_s, checkPosition_deg, struct( ...
            "TimePaddingSamples", options.CollisionTimePaddingSamples, ...
            "BoundaryIsOccupied", false));
        if any(blocked)
            continue;
        end

        diagnostic.AppliedRadius_deg = radius_deg;
        diagnostic.EntryPosition_deg = controlPoints_deg(1, :);
        diagnostic.ExitPosition_deg = controlPoints_deg(end, :);
        diagnostic.ControlPoints_deg = controlPoints_deg;
        diagnostic.Smoothed = true;
        diagnostic.Reason = "collision-free G3 blend";
        break;
    end

    if ~diagnostic.Smoothed
        diagnostic.MandatoryStop = true;
        diagnostic.Reason = "no collision-free blend";
    end
    corners(cornerIndex) = diagnostic;
end

%% Section 3: Assemble Ordered Path Primitives

primitiveTemplate = emptyPrimitive();
primitives = repmat(primitiveTemplate, 0, 1);
mandatoryStopArcLength_deg = zeros(0, 1);
currentPosition_deg = route_deg(1, :);
currentArcLength_deg = 0;

for cornerIndex = 1:cornerCount
    corner = corners(cornerIndex);
    [primitives, currentArcLength_deg] = appendLine( ...
        primitives, primitiveTemplate, currentPosition_deg, ...
        corner.EntryPosition_deg, currentArcLength_deg);
    if corner.Smoothed
        primitive = makeQuinticPrimitive(corner.ControlPoints_deg);
        primitive.StartArcLength_deg = currentArcLength_deg;
        primitive.EndArcLength_deg = ...
            currentArcLength_deg + primitive.Length_deg;
        primitive.CornerPathPointIndex = corner.PathPointIndex;
        % A route contributes at most two primitives per corner plus one.
        primitives(end + 1, 1) = primitive; %#ok<AGROW>
        currentArcLength_deg = primitive.EndArcLength_deg;
        currentPosition_deg = corner.ExitPosition_deg;
    else
        currentPosition_deg = corner.Position_deg;
        if corner.MandatoryStop
            % The stop vector is bounded by the number of route corners.
            mandatoryStopArcLength_deg(end + 1, 1) = ...
                currentArcLength_deg; %#ok<AGROW>
        end
    end
end

[primitives, currentArcLength_deg] = appendLine( ...
    primitives, primitiveTemplate, currentPosition_deg, ...
    route_deg(end, :), currentArcLength_deg);
if isempty(primitives)
    error("buildAzElSmoothPath:EmptySmoothPath", ...
        "The route produced no nonzero path primitive.");
end

%% Section 4: Sample And Publish The Geometry

primitiveBoundaryS_deg = [primitives.EndArcLength_deg].';
sampleS_deg = unique([ ...
    0; ...
    (0:0.05:currentArcLength_deg).'; ...
    primitiveBoundaryS_deg; ...
    mandatoryStopArcLength_deg; ...
    currentArcLength_deg]);
definition = struct( ...
    "Primitives", primitives, ...
    "TotalLength_deg", currentArcLength_deg);
samples = azElInternal.sampleAzElSmoothPath(definition, sampleS_deg);
mandatoryStop = false(size(sampleS_deg));
for stopIndex = 1:numel(mandatoryStopArcLength_deg)
    [~, sampleIndex] = min(abs( ...
        sampleS_deg - mandatoryStopArcLength_deg(stopIndex)));
    mandatoryStop(sampleIndex) = true;
end

smoothPath = samples;
smoothPath.Success = true;
smoothPath.Message = sprintf( ...
    "Rounded %d corners; %d stops remain.", ...
    nnz([corners.Smoothed]), nnz([corners.MandatoryStop]));
smoothPath.OriginalPathPosition_deg = route_deg;
smoothPath.Primitives = primitives;
smoothPath.TotalLength_deg = currentArcLength_deg;
smoothPath.SampleArcLength_deg = smoothPath.arcLength_deg;
smoothPath = rmfield(smoothPath, "arcLength_deg");
smoothPath.MandatoryStop = mandatoryStop;
smoothPath.MandatoryStopArcLength_deg = mandatoryStopArcLength_deg;
smoothPath.CornerDiagnostics = corners;
smoothPath.RoundedCornerCount = nnz([corners.Smoothed]);
smoothPath.MandatoryStopCount = nnz([corners.MandatoryStop]);
smoothPath.Options = struct( ...
    "TurnRadius_deg", options.TurnRadius_deg);
smoothPath = orderfields(smoothPath, emptySmoothPath(route_deg));
end

%% Section 5: Local Functions

function primitive = makeQuinticPrimitive(controlPoints_deg)
%% Section 0: Header & Readme
% SYNTAX
%   primitive = makeQuinticPrimitive(controlPoints_deg)
%**************************************************************************
% PURPOSE
%   - Build a monotone parameter-to-arc-length lookup for one blend.
%**************************************************************************
% INPUTS
%   - controlPoints_deg (6-by-2 numeric)
%**************************************************************************
% OUTPUTS
%   - primitive (scalar maintained path-primitive record)
%**************************************************************************
% UNITS
%   - Positions and arc lengths are degrees.
%**************************************************************************
controlLength_deg = sum(vecnorm(diff(controlPoints_deg), 2, 2));
parameterCount = max(100, ceil(controlLength_deg / 0.004)) + 1;
parameterGrid = linspace(0, 1, parameterCount).';
[~, firstDerivative] = azElInternal.evaluateAzElQuintic( ...
    controlPoints_deg, parameterGrid);
parameterSpeed_deg = vecnorm(firstDerivative, 2, 2);
if any(parameterSpeed_deg <= 1e-10)
    error("buildAzElSmoothPath:DegenerateQuintic", ...
        "A quintic blend has a zero parameter derivative.");
end
arcLengthGrid_deg = cumtrapz(parameterGrid, parameterSpeed_deg);
primitive = emptyPrimitive();
primitive.Type = "quintic";
primitive.StartPosition_deg = controlPoints_deg(1, :);
primitive.EndPosition_deg = controlPoints_deg(end, :);
primitive.Length_deg = arcLengthGrid_deg(end);
primitive.ControlPoints_deg = controlPoints_deg;
primitive.ParameterGrid = parameterGrid;
primitive.ArcLengthGrid_deg = arcLengthGrid_deg;
end

function [primitives, endS_deg] = appendLine( ...
        primitives, template, start_deg, goal_deg, startS_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [primitives, endS_deg] = appendLine( ...
%       primitives, template, start_deg, goal_deg, startS_deg)
%**************************************************************************
% PURPOSE
%   - Append one nonzero straight primitive with exact arc metadata.
%**************************************************************************
% INPUTS
%   - primitives (structure array), template (scalar struct)
%   - start_deg, goal_deg (1-by-2), startS_deg (scalar)
%**************************************************************************
% OUTPUTS
%   - primitives (structure array), endS_deg (scalar)
%**************************************************************************
% UNITS
%   - Positions and arc lengths are degrees.
%**************************************************************************
delta_deg = goal_deg - start_deg;
length_deg = norm(delta_deg);
endS_deg = startS_deg;
if length_deg <= 1e-9
    return;
end
primitive = template;
primitive.Type = "line";
primitive.StartPosition_deg = start_deg;
primitive.EndPosition_deg = goal_deg;
primitive.Direction = delta_deg / length_deg;
primitive.Length_deg = length_deg;
primitive.StartArcLength_deg = startS_deg;
endS_deg = startS_deg + length_deg;
primitive.EndArcLength_deg = endS_deg;
primitives(end + 1, 1) = primitive;
end

function primitive = emptyPrimitive()
%% Section 0: Header & Readme
% SYNTAX
%   primitive = emptyPrimitive()
%**************************************************************************
% PURPOSE
%   - Define the single stable geometric primitive schema.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - primitive (scalar struct)
%**************************************************************************
% UNITS
%   - Field suffixes state degree-based units.
%**************************************************************************
primitive = struct( ...
    "Type", "", ...
    "StartPosition_deg", zeros(1, 2), ...
    "EndPosition_deg", zeros(1, 2), ...
    "Direction", zeros(1, 2), ...
    "Length_deg", 0, ...
    "StartArcLength_deg", 0, ...
    "EndArcLength_deg", 0, ...
    "ControlPoints_deg", zeros(6, 2), ...
    "ParameterGrid", zeros(0, 1), ...
    "ArcLengthGrid_deg", zeros(0, 1), ...
    "CornerPathPointIndex", 0);
end

function smoothPath = emptySmoothPath(route_deg)
%% Section 0: Header & Readme
% SYNTAX
%   smoothPath = emptySmoothPath(route_deg)
%**************************************************************************
% PURPOSE
%   - Define the canonical smooth-path schema for successful and failed
%     geometric construction.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 numeric)
%**************************************************************************
% OUTPUTS
%   - smoothPath (scalar struct)
%**************************************************************************
% UNITS
%   - Route positions are degrees.
%**************************************************************************
smoothPath = struct( ...
    "Success", false, ...
    "Message", "No smooth path was produced.", ...
    "OriginalPathPosition_deg", route_deg, ...
    "Primitives", struct([]), ...
    "TotalLength_deg", 0, ...
    "SampleArcLength_deg", zeros(0, 1), ...
    "position_deg", zeros(0, 2), ...
    "tangent", zeros(0, 2), ...
    "secondDerivative_deg_inv", zeros(0, 2), ...
    "thirdDerivative_deg_inv2", zeros(0, 2), ...
    "curvature_deg_inv", zeros(0, 1), ...
    "PrimitiveIndex", zeros(0, 1), ...
    "PrimitiveType", strings(0, 1), ...
    "MandatoryStop", false(0, 1), ...
    "MandatoryStopArcLength_deg", zeros(0, 1), ...
    "CornerDiagnostics", struct([]), ...
    "RoundedCornerCount", 0, ...
    "MandatoryStopCount", 0, ...
    "Options", struct());
end
