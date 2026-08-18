function [repairTau, repairBuffer_deg] = ...
        findAzElHs3CollisionRepairs(trajectory, sampleOccupied, ...
        segmentOccupied, initialTime_s, finalTime_s, ...
        minimumRepairBuffer_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [repairTau, repairBuffer_deg] = ...
%       azElInternal.findAzElHs3CollisionRepairs( ...
%       trajectory, sampleOccupied, segmentOccupied, initialTime_s, ...
%       finalTime_s, minimumRepairBuffer_deg)
%**************************************************************************
% PURPOSE
%   - Select one repair time and recovery tube for each collided HS-3 cell.
%**************************************************************************
% INPUTS
%   - trajectory (scalar struct)
%       Dense trajectory samples, segment indices, and curve bounds.
%   - sampleOccupied (N-by-1 logical vector)
%       Occupancy or refinement state of each trajectory sample.
%   - segmentOccupied ((N-1)-by-1 logical vector)
%       Occupancy state of each dense trajectory chord.
%   - initialTime_s (numeric scalar)
%       Absolute initial time.
%   - finalTime_s (numeric scalar)
%       Absolute final time.
%   - minimumRepairBuffer_deg (nonnegative numeric scalar)
%       Minimum recovery-tube radius.
%**************************************************************************
% OUTPUTS
%   - repairTau (M-by-1 numeric vector)
%       Normalized repair times.
%   - repairBuffer_deg (M-by-1 numeric vector)
%       Recovery-tube radii at the repair times.
%**************************************************************************
% UNITS
%   - Time is seconds. Buffers are degrees. Tau is dimensionless.
%**************************************************************************

%% Section 1: Select One Time Per Collided Segment

occupiedIndex = find(sampleOccupied(:));
repairTau = zeros(0, 1);
repairBuffer_deg = zeros(0, 1);
if isempty(occupiedIndex) || finalTime_s <= initialTime_s
    return;
end
occupiedSegmentIndex = trajectory.segmentIndex(occupiedIndex);
collidingSegmentIndex = unique(occupiedSegmentIndex, "stable");
repairTime_s = zeros(numel(collidingSegmentIndex), 1);
for collisionIndex = 1:numel(collidingSegmentIndex)
    inSegment = occupiedIndex(occupiedSegmentIndex == ...
        collidingSegmentIndex(collisionIndex));
    representativeIndex = inSegment(ceil(numel(inSegment) / 2));
    representativeTime_s = trajectory.time_s(representativeIndex);
    inboundSegmentIndex = representativeIndex - 1;
    if inboundSegmentIndex >= 1 && ...
            inboundSegmentIndex <= numel(segmentOccupied) && ...
            segmentOccupied(inboundSegmentIndex)
        representativeTime_s = 0.5 * ( ...
            trajectory.time_s(representativeIndex - 1) + ...
            trajectory.time_s(representativeIndex));
    end
    repairTime_s(collisionIndex) = representativeTime_s;
end
repairTau = (repairTime_s - initialTime_s) / ...
    (finalTime_s - initialTime_s);
repairTau = min(1, max(0, repairTau));

%% Section 2: Set The Recovery Tube Radius

repairBuffer_deg = zeros(numel(collidingSegmentIndex), 1);
hasCurveBounds = isfield(trajectory, ...
    "CurveDeviationBoundBySegment_deg") && ...
    ~isempty(trajectory.CurveDeviationBoundBySegment_deg);
if hasCurveBounds
    boundedSegmentIndex = min( ...
        numel(trajectory.CurveDeviationBoundBySegment_deg), ...
        max(1, collidingSegmentIndex));
    repairBuffer_deg = ...
        trajectory.CurveDeviationBoundBySegment_deg(boundedSegmentIndex);
end
repairBuffer_deg = max(repairBuffer_deg, minimumRepairBuffer_deg);
end
