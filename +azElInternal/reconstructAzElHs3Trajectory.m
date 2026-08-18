function trajectory = reconstructAzElHs3Trajectory( ...
        solution, meshTau, initialTime_s, sampleTime_s, ...
        curveTolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   trajectory = azElInternal.reconstructAzElHs3Trajectory()
%   trajectory = azElInternal.reconstructAzElHs3Trajectory( ...
%       solution, meshTau, initialTime_s, sampleTime_s, ...
%       curveTolerance_deg)
%**************************************************************************
% PURPOSE
%   - Reconstruct a dense, dynamically consistent HS-3 trajectory.
%   - Subdivide each curve until its deviation from checked chords has a
%     conservative bound.
%**************************************************************************
% INPUTS
%   - solution (scalar propagated HS-3 solution struct)
%       KnotState, MidpointState, KnotControl, MidpointControl, and final
%       time use the schema from propagateAzElHs3Control.
%   - meshTau (increasing finite numeric vector)
%       Normalized knot locations from zero to one.
%   - initialTime_s (finite numeric scalar)
%       Physical motion start time.
%   - sampleTime_s (positive finite numeric scalar)
%       Maximum nominal output sampling interval.
%   - curveTolerance_deg (positive finite numeric scalar)
%       Maximum permitted curve-to-chord deviation before subdivision.
%**************************************************************************
% OUTPUTS
%   - trajectory (scalar struct)
%       Dense time, position, velocity, acceleration, jerk, segment index,
%       and curve-subdivision certificate histories. A zero-input call
%       returns the stable empty schema.
%**************************************************************************
% UNITS
%   - Time is seconds. Position is degrees. Derivative units are stated in
%     field names. Axis order is [azimuth elevation].
%**************************************************************************

%% Section 1: Return The Empty Schema When Requested

if nargin == 0
    trajectory = emptyTrajectory();
    return;
end

%% Section 2: Reconstruct And Subdivide Each Segment

segmentCount = numel(meshTau) - 1;
timeBuffer = cell(segmentCount, 1);
stateBuffer = cell(segmentCount, 1);
controlBuffer = cell(segmentCount, 1);
segmentIndexBuffer = cell(segmentCount, 1);
curveDeviationBoundBySegment_deg = zeros(segmentCount, 1);
curveSubdivisionConvergedBySegment = false(segmentCount, 1);
duration_s = solution.FinalTime_s - initialTime_s;
for segmentIndex = 1:segmentCount
    segmentDuration_s = duration_s * ...
        (meshTau(segmentIndex + 1) - meshTau(segmentIndex));
    sampleCount = max(2, ceil(segmentDuration_s / sampleTime_s) + 1);
    localTau = unique([linspace(0, 1, sampleCount).'; 0.5]);
    [localTau, curveDeviationBoundBySegment_deg(segmentIndex), ...
        curveSubdivisionConvergedBySegment(segmentIndex)] = ...
        refineCurveSampleTau(solution, segmentIndex, ...
        segmentDuration_s, localTau, curveTolerance_deg);
    if segmentIndex < segmentCount
        localTau(end) = [];
    end
    [state, control] = azElInternal.evaluateAzElHs3Segment( ...
        solution, segmentIndex, segmentDuration_s, localTau);
    normalizedTime = meshTau(segmentIndex) + localTau * ...
        (meshTau(segmentIndex + 1) - meshTau(segmentIndex));
    timeBuffer{segmentIndex} = initialTime_s + duration_s * normalizedTime;
    stateBuffer{segmentIndex} = state;
    controlBuffer{segmentIndex} = control;
    segmentIndexBuffer{segmentIndex} = repmat( ...
        segmentIndex, numel(localTau), 1);
end
state = vertcat(stateBuffer{:});
trajectory = struct( ...
    "time_s", vertcat(timeBuffer{:}), ...
    "position_deg", state(:, 1:2), ...
    "velocity_deg_s", state(:, 3:4), ...
    "acceleration_deg_s2", state(:, 5:6), ...
    "jerk_deg_s3", vertcat(controlBuffer{:}), ...
    "segmentIndex", vertcat(segmentIndexBuffer{:}), ...
    "CurveDeviationBoundBySegment_deg", ...
    curveDeviationBoundBySegment_deg, ...
    "CurveSubdivisionConvergedBySegment", ...
    curveSubdivisionConvergedBySegment);
end

%% Section 3: Local Functions

function [localTau, maximumDeviationBound_deg, converged] = ...
        refineCurveSampleTau(solution, segmentIndex, ...
        segmentDuration_s, localTau, curveTolerance_deg)
% PURPOSE
%   - Bound each quintic arc's deviation from its checked linear chords.
statePower = azElInternal.buildAzElHs3SegmentPolynomials( ...
    solution.KnotState(segmentIndex, :), ...
    solution.KnotControl(segmentIndex, :), ...
    solution.MidpointControl(segmentIndex, :), ...
    solution.KnotControl(segmentIndex + 1, :), segmentDuration_s);
for refinementIndex = 1:12
    intervalStart = localTau(1:end - 1);
    intervalEnd = localTau(2:end);
    splitInterval = false(size(intervalStart));
    for intervalIndex = 1:numel(intervalStart)
        deviation_deg = maximumChordDeviation( ...
            statePower(:, 1:2), intervalStart(intervalIndex), ...
            intervalEnd(intervalIndex));
        splitInterval(intervalIndex) = ...
            deviation_deg > curveTolerance_deg;
    end
    if ~any(splitInterval)
        break;
    end
    midpoint = 0.5 * (intervalStart(splitInterval) + ...
        intervalEnd(splitInterval));
    localTau = unique([localTau; midpoint]);
end
intervalStart = localTau(1:end - 1);
intervalEnd = localTau(2:end);
deviationBound_deg = zeros(size(intervalStart));
for intervalIndex = 1:numel(intervalStart)
    deviationBound_deg(intervalIndex) = maximumChordDeviation( ...
        statePower(:, 1:2), intervalStart(intervalIndex), ...
        intervalEnd(intervalIndex));
end
maximumDeviationBound_deg = max(deviationBound_deg);
converged = maximumDeviationBound_deg <= curveTolerance_deg;
end

function deviationBound = maximumChordDeviation( ...
        positionPower, intervalStart, intervalEnd)
% PURPOSE
%   - Bound quintic-to-chord distance through exact component extrema.
endpointPosition = evaluatePowerPolynomial(positionPower, ...
    [intervalStart; intervalEnd]);
chordSlope = (endpointPosition(2, :) - endpointPosition(1, :)) / ...
    (intervalEnd - intervalStart);
chordIntercept = endpointPosition(1, :) - intervalStart * chordSlope;
deviationPower = positionPower;
deviationPower(1, :) = deviationPower(1, :) - chordIntercept;
deviationPower(2, :) = deviationPower(2, :) - chordSlope;
componentBound = polynomialMaximumAbsoluteOnInterval( ...
    deviationPower, intervalStart, intervalEnd);
deviationBound = norm(componentBound);
end

function maximumAbsolute = polynomialMaximumAbsoluteOnInterval( ...
        powerCoefficient, intervalStart, intervalEnd)
% PURPOSE
%   - Bound component magnitudes on a subinterval with a Bernstein hull.
restrictedPower = restrictPowerPolynomial( ...
    powerCoefficient, intervalStart, intervalEnd);
restrictedBernstein = azElInternal.powerToBernstein(restrictedPower);
maximumAbsolute = max(abs(restrictedBernstein), [], 1);
end

function restrictedPower = restrictPowerPolynomial( ...
        powerCoefficient, intervalStart, intervalEnd)
% PURPOSE
%   - Express p(a + (b-a)s) in ascending powers of s on [0,1].
degree = size(powerCoefficient, 1) - 1;
intervalWidth = intervalEnd - intervalStart;
persistent restrictionPatternByDegree
if isempty(restrictionPatternByDegree)
    restrictionPatternByDegree = cell(0, 1);
end
cacheIndex = degree + 1;
if numel(restrictionPatternByDegree) < cacheIndex || ...
        isempty(restrictionPatternByDegree{cacheIndex})
    [sourcePower, targetPower] = meshgrid(0:degree, 0:degree);
    validEntry = sourcePower >= targetPower;
    startExponent = sourcePower - targetPower;
    startExponent(~validEntry) = 0;
    binomialCoefficient = zeros(cacheIndex);
    for targetIndex = 0:degree
        for sourceIndex = targetIndex:degree
            binomialCoefficient(targetIndex + 1, sourceIndex + 1) = ...
                nchoosek(sourceIndex, targetIndex);
        end
    end
    restrictionPatternByDegree{cacheIndex} = struct( ...
        "BinomialCoefficient", binomialCoefficient, ...
        "StartExponent", startExponent, ...
        "WidthExponent", targetPower, ...
        "ValidEntry", validEntry);
end
pattern = restrictionPatternByDegree{cacheIndex};
restrictionTransform = pattern.BinomialCoefficient .* ...
    intervalStart .^ pattern.StartExponent .* ...
    intervalWidth .^ pattern.WidthExponent;
restrictionTransform(~pattern.ValidEntry) = 0;
restrictedPower = restrictionTransform * powerCoefficient;
end

function trajectory = emptyTrajectory()
% PURPOSE
%   - Define the stable empty trajectory schema.
trajectory = struct( ...
    "time_s", zeros(0, 1), ...
    "position_deg", zeros(0, 2), ...
    "velocity_deg_s", zeros(0, 2), ...
    "acceleration_deg_s2", zeros(0, 2), ...
    "jerk_deg_s3", zeros(0, 2), ...
    "segmentIndex", zeros(0, 1), ...
    "CurveDeviationBoundBySegment_deg", zeros(0, 1), ...
    "CurveSubdivisionConvergedBySegment", false(0, 1));
end

function value = evaluatePowerPolynomial(powerCoefficient, localTau)
% PURPOSE
%   - Evaluate ascending power coefficients at normalized segment times.
localTau = localTau(:);
power = localTau .^ (0:size(powerCoefficient, 1) - 1);
value = power * powerCoefficient;
end
