function evidence = validateAzElCollision(command, request, motionEvidence)
%% Section 0: Header & Readme
% SYNTAX
%   evidence = validateAzElCollision(command, request, motionEvidence)
%**************************************************************************
% PURPOSE
%   - Independently certify full-interval obstacle clearance by recursively
%     bounding relative command and linearly interpolated polygon motion.
%**************************************************************************
% INPUTS
%   - command (scalar piecewise-quintic command)
%   - request (normalized scalar planning request)
%   - motionEvidence (scalar struct)
%       Exact polynomial motion extrema, including a speed bound.
%**************************************************************************
% OUTPUTS
%   - evidence (scalar struct)
%       collisionFree, resolved, minimumClearance_deg, and diagnostics.
%**************************************************************************
% UNITS
%   - Clearance is degrees and time is seconds.

%% Section 1: Build Continuity Breakpoints
evidence = collisionEvidenceTemplate();
if isempty(request.obstacles)
    evidence.collisionFree = true;
    evidence.resolved = true;
    evidence.minimumClearance_deg = Inf;
    evidence.message = "No obstacles were supplied.";
    return;
end

startTime_s = command.time_s(1);
endTime_s = command.time_s(end);
breakTime_s = command.time_s(:);
padding_s = request.options.temporalPadding_s;
for obstacleIndex = 1:numel(request.obstacles)
    obstacleTime_s = request.obstacles{obstacleIndex}.time_s;
    breakTime_s = [breakTime_s; obstacleTime_s; ...
        obstacleTime_s - padding_s; obstacleTime_s + padding_s]; ...
        %#ok<AGROW>
end
breakTime_s = breakTime_s(breakTime_s >= startTime_s & ...
    breakTime_s <= endTime_s);
breakTime_s = unique([startTime_s; breakTime_s; endTime_s]);

%% Section 2: Certify Each Smooth Time Interval
minimumClearance_deg = Inf;
checkedPointCount = 0;
subdivisionCount = 0;
maximumDepth = 22;
for intervalIndex = 1:(numel(breakTime_s) - 1)
    intervalStart_s = breakTime_s(intervalIndex);
    intervalEnd_s = breakTime_s(intervalIndex + 1);
    [intervalStatus, intervalEvidence] = certifyInterval( ...
        intervalStart_s, intervalEnd_s, 0, maximumDepth, command, ...
        request, motionEvidence.maximumSpeed_deg_s);
    minimumClearance_deg = min(minimumClearance_deg, ...
        intervalEvidence.minimumClearance_deg);
    checkedPointCount = checkedPointCount + ...
        intervalEvidence.checkedPointCount;
    subdivisionCount = subdivisionCount + ...
        intervalEvidence.subdivisionCount;
    if intervalStatus == "collision"
        evidence.collisionFree = false;
        evidence.resolved = true;
        evidence.minimumClearance_deg = minimumClearance_deg;
        evidence.firstFailureTime_s = ...
            intervalEvidence.firstFailureTime_s;
        evidence.firstFailureTargetName = ...
            intervalEvidence.firstFailureTargetName;
        evidence.checkedPointCount = checkedPointCount;
        evidence.subdivisionCount = subdivisionCount;
        evidence.message = "A safety-margin collision was found.";
        return;
    elseif intervalStatus == "unresolved"
        evidence.collisionFree = false;
        evidence.resolved = false;
        evidence.minimumClearance_deg = minimumClearance_deg;
        evidence.firstFailureTime_s = ...
            intervalEvidence.firstFailureTime_s;
        evidence.firstFailureTargetName = ...
            intervalEvidence.firstFailureTargetName;
        evidence.checkedPointCount = checkedPointCount;
        evidence.subdivisionCount = subdivisionCount;
        evidence.message = ["Clearance could not be certified at the " ...
            "available numerical depth."];
        return;
    end
end

evidence.collisionFree = true;
evidence.resolved = true;
evidence.minimumClearance_deg = minimumClearance_deg;
evidence.checkedPointCount = checkedPointCount;
evidence.subdivisionCount = subdivisionCount;
evidence.message = "Every command interval has a continuous clearance bound.";
end

function [status, evidence] = certifyInterval(startTime_s, endTime_s, ...
        depth, maximumDepth, command, request, commandSpeedBound_deg_s)
%% Section 0: Header & Readme
% SYNTAX
%   [status, evidence] = certifyInterval(startTime_s, endTime_s, depth, ...
%       maximumDepth, command, request, commandSpeedBound_deg_s)
%**************************************************************************
% PURPOSE
%   - Certify or recursively subdivide one time interval using a Lipschitz
%     clearance bound from command and obstacle speed.
%**************************************************************************
% INPUTS
%   - startTime_s, endTime_s (finite scalars)
%   - depth, maximumDepth (nonnegative integers)
%   - command (scalar struct)
%   - request (normalized scalar struct)
%   - commandSpeedBound_deg_s (nonnegative scalar)
%**************************************************************************
% OUTPUTS
%   - status ("safe", "collision", or "unresolved")
%   - evidence (scalar struct)
%**************************************************************************
% UNITS
%   - Time is seconds, speed is deg/s, and clearance is degrees.

middleTime_s = 0.5 .* (startTime_s + endTime_s);
sampleTime_s = [startTime_s; middleTime_s; endTime_s];
sampledState = sampleAzElCommand(command, sampleTime_s);
clearance_deg = zeros(3, 1);
obstacleSpeedBound_deg_s = 0;
nearest = repmat(struct( ...
    "obstacleIndex", 0, ...
    "regionIndex", 0, ...
    "targetName", "", ...
    "motionSpeedBound_deg_s", 0), 3, 1);
for sampleIndex = 1:3
    [clearance_deg(sampleIndex), nearest(sampleIndex)] = ...
        azElObstacleClearance(request.obstacles, ...
            sampledState.unwrappedPosition_deg(sampleIndex, :), ...
            sampleTime_s(sampleIndex), request.options);
    obstacleSpeedBound_deg_s = max(obstacleSpeedBound_deg_s, ...
        nearest(sampleIndex).motionSpeedBound_deg_s);
end

[minimumClearance_deg, minimumIndex] = min(clearance_deg);
evidence = struct( ...
    "minimumClearance_deg", minimumClearance_deg, ...
    "checkedPointCount", 3, ...
    "subdivisionCount", 0, ...
    "firstFailureTime_s", sampleTime_s(minimumIndex), ...
    "firstFailureTargetName", nearest(minimumIndex).targetName);

requiredClearance_deg = request.options.safetyMargin_deg;
clearanceTolerance_deg = request.options.clearanceTolerance_deg;
if minimumClearance_deg < ...
        requiredClearance_deg - clearanceTolerance_deg
    status = "collision";
    return;
end

intervalDuration_s = endTime_s - startTime_s;
relativeSpeedBound_deg_s = commandSpeedBound_deg_s + ...
    obstacleSpeedBound_deg_s;
% Endpoints and midpoint leave every instant at most one quarter interval
% away from a checked instant. Signed distance to a moving closed set is
% Lipschitz under the conservative relative-speed bound.
unseenClearanceLoss_deg = relativeSpeedBound_deg_s .* ...
    intervalDuration_s ./ 4;
if all(minimumClearance_deg - unseenClearanceLoss_deg > ...
        requiredClearance_deg + clearanceTolerance_deg)
    status = "safe";
    return;
end

if depth >= maximumDepth || ...
        intervalDuration_s <= 64 * eps(max(1, abs(middleTime_s)))
    status = "unresolved";
    return;
end

[leftStatus, leftEvidence] = certifyInterval(startTime_s, ...
    middleTime_s, depth + 1, maximumDepth, command, request, ...
    commandSpeedBound_deg_s);
evidence.checkedPointCount = evidence.checkedPointCount + ...
    leftEvidence.checkedPointCount;
evidence.subdivisionCount = evidence.subdivisionCount + ...
    leftEvidence.subdivisionCount + 1;
if leftEvidence.minimumClearance_deg < evidence.minimumClearance_deg
    evidence.minimumClearance_deg = leftEvidence.minimumClearance_deg;
    evidence.firstFailureTime_s = leftEvidence.firstFailureTime_s;
    evidence.firstFailureTargetName = ...
        leftEvidence.firstFailureTargetName;
end
if leftStatus ~= "safe"
    status = leftStatus;
    return;
end

[rightStatus, rightEvidence] = certifyInterval(middleTime_s, ...
    endTime_s, depth + 1, maximumDepth, command, request, ...
    commandSpeedBound_deg_s);
evidence.checkedPointCount = evidence.checkedPointCount + ...
    rightEvidence.checkedPointCount;
evidence.subdivisionCount = evidence.subdivisionCount + ...
    rightEvidence.subdivisionCount + 1;
if rightEvidence.minimumClearance_deg < evidence.minimumClearance_deg
    evidence.minimumClearance_deg = rightEvidence.minimumClearance_deg;
    evidence.firstFailureTime_s = rightEvidence.firstFailureTime_s;
    evidence.firstFailureTargetName = ...
        rightEvidence.firstFailureTargetName;
end
status = rightStatus;
end

function evidence = collisionEvidenceTemplate()
%% Section 0: Header & Readme
% SYNTAX
%   evidence = collisionEvidenceTemplate()
%**************************************************************************
% PURPOSE
%   - Return the stable collision-evidence schema.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - evidence (scalar struct)
%**************************************************************************
% UNITS
%   - Field names state their units.

evidence = struct( ...
    "collisionFree", false, ...
    "resolved", false, ...
    "minimumClearance_deg", -Inf, ...
    "firstFailureTime_s", NaN, ...
    "firstFailureTargetName", "", ...
    "checkedPointCount", 0, ...
    "subdivisionCount", 0, ...
    "message", "Collision validation did not complete.");
end
