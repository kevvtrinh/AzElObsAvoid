function [isOccupied, certificate] = queryAzElTimedCurveCollision( ...
        obstacleField, timedPath, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   defaults = queryAzElTimedCurveCollision()
%   isOccupied = queryAzElTimedCurveCollision(obstacleField, timedPath)
%   [isOccupied, certificate] = queryAzElTimedCurveCollision( ...
%       obstacleField, timedPath, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Certify a nonlinear timed curve against protected moving obstacles.
%   - Enclose each nonlinear interval around its linear time chord before
%     the exact linear collision kernel is used.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%       Protected geometry with source and safety-margin provenance.
%   - timedPath (scalar retimer result)
%       Successful time/position history. Its continuous Cartesian
%       acceleration bound and all motion-profile phase boundaries are
%       required. Unresolved input is rejected, not called collision-free.
%   - optionOverrides (scalar struct, optional; default struct())
%       .MaximumNumericalEnvelope_deg limits accepted chord error (0.001).
%       .TimePaddingSamples selects neighboring obstacle slices (default 1).
%**************************************************************************
% OUTPUTS
%   - isOccupied (N-by-1 logical)
%       Protected-geometry occupancy for the conservative chord envelope.
%       Unresolved certification returns all true.
%   - certificate (scalar struct)
%       Resolution, envelope, collision, phase-boundary, and safety-margin
%       diagnostics. CollisionFree is true only when Resolved is true.
%**************************************************************************
% UNITS
%   - Position, envelope, and obstacle margins are degrees. Time is seconds.
%**************************************************************************

%% Section 1: Validate The Timed Curve & Options

defaults = struct( ...
    "MaximumNumericalEnvelope_deg", 0.001, ...
    "TimePaddingSamples", 1);
if nargin == 0
    isOccupied = defaults;
    certificate = struct();
    return;
end
if nargin < 3 || isempty(optionOverrides)
    optionOverrides = struct();
end
[options, unknown] = azElInternal.resolveOptions(defaults, optionOverrides);
if ~isempty(unknown)
    warning("queryAzElTimedCurveCollision:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknown, ", "));
end
validateattributes(options.MaximumNumericalEnvelope_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(options.TimePaddingSamples, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
requiredFields = ["Success" "time_s" "position_deg" ...
    "ConstraintDiagnostics" "SegmentProfiles"];
if ~isstruct(timedPath) || ~isscalar(timedPath) || ...
        ~all(isfield(timedPath, requiredFields))
    error("queryAzElTimedCurveCollision:InvalidTimedPath", ...
        "timedPath must contain every documented retimer result field.");
end
time_s = double(timedPath.time_s(:));
position_deg = double(timedPath.position_deg);
if ~timedPath.Success || numel(time_s) < 2 || ...
        size(position_deg, 1) ~= numel(time_s) || ...
        size(position_deg, 2) ~= 2 || any(~isfinite(time_s)) || ...
        any(~isfinite(position_deg), "all") || any(diff(time_s) <= 0)
    error("queryAzElTimedCurveCollision:UnusableTimedPath", ...
        "timedPath must be successful with finite, strictly increasing data.");
end
if ~isfield(timedPath.ConstraintDiagnostics, ...
        "PeakAcceleration_deg_s2")
    error("queryAzElTimedCurveCollision:MissingAccelerationBound", ...
        "timedPath must contain a continuous acceleration certificate.");
end
accelerationBound_deg_s2 = double( ...
    timedPath.ConstraintDiagnostics.PeakAcceleration_deg_s2);
validateattributes(accelerationBound_deg_s2, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2, 'nonnegative'});
accelerationBound_deg_s2 = reshape(accelerationBound_deg_s2, 1, 2);

%% Section 2: Prove The Linearization Envelope

requiredEventTime_s = profileEventTimes(timedPath.SegmentProfiles);
timeScale_s = max(1, max(abs(time_s)));
timeTolerance_s = 1e-11 * timeScale_s;
phaseBoundarySampled = true;
for eventIndex = 1:numel(requiredEventTime_s)
    phaseBoundarySampled = phaseBoundarySampled && any( ...
        abs(time_s - requiredEventTime_s(eventIndex)) <= timeTolerance_s);
end
intervalDuration_s = diff(time_s);
accelerationNorm_deg_s2 = norm(accelerationBound_deg_s2);
% The M*h^2/8 linear-interpolation remainder is listed in citation.md.
intervalEnvelope_deg = accelerationNorm_deg_s2 * ...
    intervalDuration_s.^2 / 8;
positionScale_deg = max(1, max(abs(position_deg), [], "all"));
roundoffEnvelope_deg = 4096 * eps(positionScale_deg);
maximumEnvelope_deg = max(intervalEnvelope_deg, [], "all") + ...
    roundoffEnvelope_deg;
resolved = phaseBoundarySampled && ...
    maximumEnvelope_deg <= options.MaximumNumericalEnvelope_deg;
certificate = certificateTemplate();
certificate.Resolved = resolved;
certificate.PhaseBoundarySampled = phaseBoundarySampled;
certificate.AccelerationBound_deg_s2 = accelerationBound_deg_s2;
certificate.IntervalEnvelope_deg = intervalEnvelope_deg;
certificate.MaximumEnvelope_deg = maximumEnvelope_deg;
certificate.MaximumNumericalEnvelope_deg = ...
    options.MaximumNumericalEnvelope_deg;
certificate.RoundoffEnvelope_deg = roundoffEnvelope_deg;
certificate.OriginalSafetyMargins_deg = reshape( ...
    double(obstacleField.SafetyMarginsDeg), [], 1);
certificate.SafetyMarginPreserved = true;
if ~resolved
    isOccupied = true(numel(time_s), 1);
    certificate.Message = ...
        "Timed-curve chord envelope is unresolved. Refine the time grid.";
    return;
end

%% Section 3: Query The Conservative Numerical Envelope

numericalEnvelopeField = expandForNumericalEnvelope( ...
    obstacleField, maximumEnvelope_deg);
[isOccupied, pathDetails] = queryAzElTimedPathCollision( ...
    numericalEnvelopeField, time_s, position_deg, struct( ...
        "TimePaddingSamples", options.TimePaddingSamples, ...
        "BoundaryIsOccupied", true, ...
        "StopAtFirstCollision", false));
certificate.CollisionFree = ~any(isOccupied);
certificate.SampleOccupied = pathDetails.SampleOccupied;
certificate.SegmentOccupied = pathDetails.SegmentOccupied;
certificate.BlockingObstacleIndex = pathDetails.BlockingObstacleIndex;
certificate.BlockingSliceIndex = pathDetails.BlockingSliceIndex;
certificate.time_s = time_s;
certificate.position_deg = position_deg;
certificate.Message = "Conservative nonlinear timed-curve query completed.";
end

%% Section 4: Local Functions

function eventTime_s = profileEventTimes(profiles)
% Collect every analytic profile phase boundary in absolute time.
eventTime_s = zeros(0, 1);
for profileIndex = 1:numel(profiles)
    profile = profiles(profileIndex);
    localEventTime_s = unique([0; profile.PhaseStartTime_s(:); ...
        cumsum(profile.PhaseDuration_s(:)); profile.Duration_s]);
    localEventTime_s = localEventTime_s( ...
        localEventTime_s >= 0 & ...
        localEventTime_s <= profile.Duration_s);
    eventTime_s = [eventTime_s; ...
        profile.StartTime_s + localEventTime_s]; %#ok<AGROW>
end
eventTime_s = unique(eventTime_s);
end

function expandedField = expandForNumericalEnvelope( ...
        obstacleField, numericalEnvelope_deg)
% Build a temporary larger field without changing the safety-margin record.
if isempty(obstacleField.Obstacles) || numericalEnvelope_deg <= 0
    expandedField = obstacleField;
    return;
end
hasProvenance = isfield(obstacleField, "SourceAzElData") && ...
    isfield(obstacleField, "SafetyMarginsDeg");
if ~hasProvenance
    error("queryAzElTimedCurveCollision:MissingSafetyProvenance", ...
        "obstacleField must retain source data and safety margins.");
end
sourceData = obstacleField.SourceAzElData;
expandedData = sourceData;
for obstacleIndex = 1:numel(sourceData)
    expandedData(obstacleIndex) = inflateAzElObstacleData( ...
        sourceData(obstacleIndex), ...
        obstacleField.SafetyMarginsDeg(obstacleIndex) + ...
        numericalEnvelope_deg);
end
expandedField = buildAzElTimeObstacleField(expandedData, struct( ...
    "ReferenceTime", obstacleField.ReferenceTime));
end

function certificate = certificateTemplate()
% Define the stable curve-query certificate schema.
certificate = struct( ...
    "Resolved", false, ...
    "CollisionFree", false, ...
    "Message", "not evaluated", ...
    "PhaseBoundarySampled", false, ...
    "AccelerationBound_deg_s2", [NaN NaN], ...
    "IntervalEnvelope_deg", zeros(0, 1), ...
    "MaximumEnvelope_deg", NaN, ...
    "MaximumNumericalEnvelope_deg", NaN, ...
    "RoundoffEnvelope_deg", NaN, ...
    "OriginalSafetyMargins_deg", zeros(0, 1), ...
    "SafetyMarginPreserved", false, ...
    "SampleOccupied", false(0, 1), ...
    "SegmentOccupied", false(0, 1), ...
    "BlockingObstacleIndex", zeros(0, 1, "uint32"), ...
    "BlockingSliceIndex", zeros(0, 1, "uint32"), ...
    "time_s", zeros(0, 1), ...
    "position_deg", zeros(0, 2));
end
