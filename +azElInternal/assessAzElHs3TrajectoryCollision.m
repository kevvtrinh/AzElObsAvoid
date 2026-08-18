function [collisionMask, collisionDetails, assessment] = ...
        assessAzElHs3TrajectoryCollision( ...
        obstacleField, trajectory, options)
%% Section 0: Header & Readme
% SYNTAX
%   [collisionMask, collisionDetails, assessment] = ...
%       azElInternal.assessAzElHs3TrajectoryCollision( ...
%       obstacleField, trajectory, options)
%**************************************************************************
% PURPOSE
%   - Combine timed chord collision queries with a conservative HS-3 curve
%     clearance certificate.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed-obstacle struct)
%       Canonical original or safety-adjusted obstacle geometry.
%   - trajectory (scalar struct)
%       Dense HS-3 motion with time, position, segment indices, and curve
%       deviation bounds.
%   - options (scalar struct)
%       Resolved collision time padding and boundary policy controls.
%**************************************************************************
% OUTPUTS
%   - collisionMask (N-by-1 logical vector)
%       Occupied samples and unresolved curve segments.
%   - collisionDetails (scalar struct)
%       Details from the public timed-path collision query.
%   - assessment (scalar struct)
%       CollisionFree, buffered chord, certificate, clearance, and
%       refinement fields.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Time is seconds.
%**************************************************************************

%% Section 1: Check Buffered Chords

[collisionMask, collisionDetails] = queryAzElTimedPathCollision( ...
    obstacleField, trajectory.time_s, trajectory.position_deg, struct( ...
    "TimePaddingSamples", options.CollisionTimePaddingSamples, ...
    "BoundaryIsOccupied", true, "StopAtFirstCollision", false));
bufferedChordQueryClear = ~any(collisionMask);
clearanceDiagnostics = azElInternal.emptyCollisionClearanceDiagnostics();
collisionSafetyProven = false;
pathSegmentCount = max(0, numel(trajectory.time_s) - 1);
curveSubdivisionConverged = ...
    ~isempty(trajectory.CurveSubdivisionConvergedBySegment) && ...
    all(trajectory.CurveSubdivisionConvergedBySegment);

%% Section 2: Certify The Curve Tube

if bufferedChordQueryClear && curveSubdivisionConverged
    hsSegmentIndex = trajectory.segmentIndex(1:pathSegmentCount);
    hsSegmentIndex = min( ...
        numel(trajectory.CurveDeviationBoundBySegment_deg), ...
        max(1, hsSegmentIndex));
    clearanceBuffer_deg = ...
        trajectory.CurveDeviationBoundBySegment_deg(hsSegmentIndex);
    [collisionSafetyProven, clearanceDiagnostics] = ...
        azElInternal.certifyAzElTimedPathClearance( ...
        obstacleField, trajectory.time_s, trajectory.position_deg, ...
        clearanceBuffer_deg);
end

%% Section 3: Assemble The Assessment

refinementMask = collisionMask;
if bufferedChordQueryClear && ~collisionSafetyProven
    unresolvedPathSegmentMask = ...
        clearanceDiagnostics.UnresolvedPathSegmentMask;
    if isempty(unresolvedPathSegmentMask) && pathSegmentCount > 0
        unresolvedPathSegmentMask = true(pathSegmentCount, 1);
    end
    refinementMask(2:end) = refinementMask(2:end) | ...
        unresolvedPathSegmentMask;
end
assessment = struct( ...
    "CollisionFree", bufferedChordQueryClear && collisionSafetyProven, ...
    "BufferedChordQueryClear", bufferedChordQueryClear, ...
    "CollisionSafetyProven", collisionSafetyProven, ...
    "ClearanceDiagnostics", clearanceDiagnostics, ...
    "RefinementMask", refinementMask);
end
