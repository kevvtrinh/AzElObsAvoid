function [badSegment, refinementPriority] = ...
        selectAzElHs3RefinementSegments(solution, meshTau, certificate, ...
        limits, collisionMask, trajectory, obstacleField, options)
%% Section 0: Header & Readme
% SYNTAX
%   [badSegment, refinementPriority] = ...
%       azElInternal.selectAzElHs3RefinementSegments( ...
%       solution, meshTau, certificate, limits, collisionMask, ...
%       trajectory, obstacleField, options)
%**************************************************************************
% PURPOSE
%   - Select HS-3 mesh segments that need local refinement.
%   - Rank collision stress before limit and collocation stress.
%**************************************************************************
% INPUTS
%   - solution (scalar struct)
%       Current HS-3 knot and midpoint solution.
%   - meshTau (N-by-1 numeric vector)
%       Normalized collocation mesh from zero through one.
%   - certificate (scalar struct)
%       Per-segment dynamics and motion-limit certificate.
%   - limits (scalar struct)
%       Velocity, acceleration, and jerk limits.
%   - collisionMask (M-by-1 logical vector)
%       Dense trajectory samples that need collision refinement.
%   - trajectory (scalar struct)
%       Dense trajectory and source-segment indices.
%   - obstacleField (scalar packed-obstacle struct)
%       Canonical planning obstacle geometry.
%   - options (scalar struct)
%       Resolved collocation and clearance tolerances.
%**************************************************************************
% OUTPUTS
%   - badSegment ((N-1)-by-1 logical vector)
%       True for each segment selected for refinement.
%   - refinementPriority ((N-1)-by-4 numeric matrix)
%       Lexicographic stress values used to limit segment growth.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Motion units follow limits.
%**************************************************************************

%% Section 1: Measure Dynamics And Limit Stress

segmentCount = numel(meshTau) - 1;
dynamicsStress = certificate.SegmentDynamicsDefect / ...
    options.CollocationErrorTolerance;
collocationStress = certificate.SegmentCollocationDefect / ...
    options.NlpConstraintTolerance;
badSegment = dynamicsStress > 1 | collocationStress > 1;
velocityStress = max(certificate.MaximumVelocityBySegment_deg_s ./ ...
    limits.maxVelocity_deg_s, [], 2);
accelerationStress = max( ...
    certificate.MaximumAccelerationBySegment_deg_s2 ./ ...
    limits.maxAcceleration_deg_s2, [], 2);
jerkStress = max(certificate.MaximumJerkBySegment_deg_s3 ./ ...
    limits.maxJerk_deg_s3, [], 2);
limitStress = max([velocityStress, accelerationStress, jerkStress], [], 2);
badSegment = badSegment | limitStress > 0.98;

%% Section 2: Measure Clearance And Collision Stress

[~, ~, sampleClearance_deg] = ...
    azElInternal.sampleAzElObstacleClearance( ...
    obstacleField, trajectory.time_s, trajectory.position_deg);
minimumClearanceBySegment_deg = inf(segmentCount, 1);
for sampleIndex = 1:numel(sampleClearance_deg)
    segmentIndex = trajectory.segmentIndex(sampleIndex);
    if segmentIndex >= 1 && segmentIndex <= segmentCount
        minimumClearanceBySegment_deg(segmentIndex) = min( ...
            minimumClearanceBySegment_deg(segmentIndex), ...
            sampleClearance_deg(sampleIndex));
    end
end
clearanceRefinementThreshold_deg = max( ...
    10 * options.ObstacleConstraintTolerance_deg, ...
    0.5 * options.VisibilitySampleStep_deg);
clearanceStress = clearanceRefinementThreshold_deg ./ ...
    max(minimumClearanceBySegment_deg, eps);
badSegment = badSegment | minimumClearanceBySegment_deg <= ...
    clearanceRefinementThreshold_deg;

collisionStress = zeros(segmentCount, 1);
if any(collisionMask)
    collisionSegment = unique(trajectory.segmentIndex(collisionMask));
    collisionSegment = collisionSegment( ...
        collisionSegment >= 1 & collisionSegment <= segmentCount);
    badSegment(collisionSegment) = true;
    collisionStress(collisionSegment) = 1;
end

%% Section 3: Measure Direction Change And Rank Segments

velocity = solution.KnotState(:, 3:4);
curvatureStress = zeros(segmentCount, 1);
for segmentIndex = 1:segmentCount
    firstSpeed = norm(velocity(segmentIndex, :));
    lastSpeed = norm(velocity(segmentIndex + 1, :));
    if firstSpeed <= 1e-8 || lastSpeed <= 1e-8
        continue;
    end
    directionCosine = dot(velocity(segmentIndex, :), ...
        velocity(segmentIndex + 1, :)) / (firstSpeed * lastSpeed);
    if directionCosine < cosd(20)
        badSegment(segmentIndex) = true;
        curvatureStress(segmentIndex) = 1;
    end
end
limitOrCurvatureStress = max( ...
    [limitStress, curvatureStress, clearanceStress], [], 2);
refinementPriority = [collisionStress, limitOrCurvatureStress, ...
    collocationStress, dynamicsStress];
end
