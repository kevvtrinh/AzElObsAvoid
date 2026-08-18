function [result, diagnostics] = solveAzElHs3Segments( ...
        segments, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = solveAzElHs3Segments()
%   result = solveAzElHs3Segments(segments)
%   result = solveAzElHs3Segments(segments, optionOverrides)
%   [result, diagnostics] = solveAzElHs3Segments( ...
%       segments, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Build one certified HS-3 motion profile from connected user segments.
%   - Use the maintained public planner for every segment.
%**************************************************************************
% INPUTS
%   - segments (nonempty struct array)
%       Records based on azElHs3SegmentTemplate. Adjacent records must have
%       equal position, velocity, and acceleration at their shared state.
%   - optionOverrides (scalar struct, optional; default struct())
%       .SampleTime_s is positive (default 0.05).
%       .MaximumPlanningTimePerSegment_s is positive (default 30).
%       .ContinuityTolerance is nonnegative (default 1e-7).
%       .Verbose is logical (default false).
%       .PlannerOptions is a scalar partial planAzElMotion option struct.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable combined motion record. Normal planner failure returns
%       Success=false. Invalid segment data causes an identified error.
%   - diagnostics (scalar struct, optional)
%       Per-segment compact planner results and expert diagnostics.
%**************************************************************************
% UNITS
%   - Position is degrees. Time is seconds. Histories are N-by-2 in
%     [azimuth elevation] order. Derivative units are in field names.
%**************************************************************************

%% Section 1: Validate Inputs & Apply Defaults

defaults = struct( ...
    "SampleTime_s", 0.05, ...
    "MaximumPlanningTimePerSegment_s", 30, ...
    "ContinuityTolerance", 1e-7, ...
    "Verbose", false, ...
    "PlannerOptions", struct());
if nargin == 0
    result = defaults;
    diagnostics = struct();
    return;
end
if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("solveAzElHs3Segments:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
[options, unknownOptions] = azElInternal.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownOptions)
    warning("solveAzElHs3Segments:UnknownOptions", ...
        "Ignored unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownOptions, ", "));
end
validateattributes(options.SampleTime_s, {'numeric'}, ...
    {'real','finite','scalar','positive'});
validateattributes(options.MaximumPlanningTimePerSegment_s, {'numeric'}, ...
    {'real','finite','scalar','positive'});
validateattributes(options.ContinuityTolerance, {'numeric'}, ...
    {'real','finite','scalar','nonnegative'});
options.Verbose = azElInternal.normalizeLogicalScalar( ...
    options.Verbose, "Verbose", ...
    "solveAzElHs3Segments:InvalidVerbose");
if ~isstruct(options.PlannerOptions) || ...
        ~isscalar(options.PlannerOptions)
    error("solveAzElHs3Segments:InvalidPlannerOptions", ...
        "PlannerOptions must be a scalar struct.");
end
if ~isstruct(segments) || isempty(segments)
    error("solveAzElHs3Segments:InvalidSegments", ...
        "segments must be a nonempty struct array.");
end

resolvedSegments = resolveSegments(segments);
validateSegmentContinuity(resolvedSegments, options.ContinuityTolerance);

%% Section 2: Solve Each Segment

solveTimer = tic;
segmentCount = numel(resolvedSegments);
segmentResults = cell(segmentCount, 1);
plannerDiagnostics = cell(segmentCount, 1);
result = emptyResult(resolvedSegments, options);
diagnostics = struct( ...
    "SegmentResults", {segmentResults}, ...
    "PlannerDiagnostics", {plannerDiagnostics}, ...
    "FailedSegmentIndex", NaN);

currentTime_s = 0;
previousTerminalJerk_deg_s3 = zeros(0, 2);
jerkJumpByJunction_deg_s3 = zeros(max(0, segmentCount - 1), 2);
for segmentIndex = 1:segmentCount
    segment = resolvedSegments(segmentIndex);
    initialState = struct( ...
        "time_s", currentTime_s, ...
        "position_deg", segment.startPosition_deg, ...
        "velocity_deg_s", segment.initialVelocity_deg_s, ...
        "acceleration_deg_s2", segment.initialAcceleration_deg_s2);
    goalState = struct( ...
        "time_s", currentTime_s + segment.maximumDuration_s, ...
        "position_deg", segment.endPosition_deg, ...
        "velocity_deg_s", segment.finalVelocity_deg_s, ...
        "acceleration_deg_s2", segment.finalAcceleration_deg_s2);
    plannerOptions = segmentPlannerOptions(options, segment);
    [segmentResult, segmentDiagnostics] = planAzElMotion( ...
        [], initialState, goalState, segment.limits, plannerOptions);
    segmentResults{segmentIndex} = segmentResult;
    plannerDiagnostics{segmentIndex} = segmentDiagnostics;
    diagnostics.SegmentResults = segmentResults;
    diagnostics.PlannerDiagnostics = plannerDiagnostics;

    if ~segmentResult.Success || ~segmentResult.Validation.Passed
        result.Message = "Segment " + segmentIndex + ...
            " failed: " + segmentResult.Message;
        result.TerminationReason = "segmentFailure";
        result.FailedSegmentIndex = segmentIndex;
        result.ElapsedTime_s = toc(solveTimer);
        diagnostics.FailedSegmentIndex = segmentIndex;
        return;
    end

    if segmentIndex > 1
        jerkJumpByJunction_deg_s3(segmentIndex - 1, :) = ...
            segmentResult.jerk_deg_s3(1, :) - ...
            previousTerminalJerk_deg_s3;
    end
    previousTerminalJerk_deg_s3 = ...
        segmentResult.jerk_deg_s3(end, :);
    result = appendSegment(result, segmentResult, ...
        segment.limits, segmentIndex);
    currentTime_s = segmentResult.time_s(end);
end

%% Section 3: Validate And Return The Combined Motion

result.Success = true;
result.Message = "Built " + segmentCount + ...
    " certified HS-3 segment(s).";
result.TerminationReason = "completed";
result.FailedSegmentIndex = NaN;
result.ElapsedTime_s = toc(solveTimer);
maximumJerkJump_deg_s3 = [0 0];
if ~isempty(jerkJumpByJunction_deg_s3)
    maximumJerkJump_deg_s3 = max( ...
        abs(jerkJumpByJunction_deg_s3), [], 1);
end
jerkContinuityTolerance_deg_s3 = max( ...
    options.ContinuityTolerance, 1e-7);
segmentValidationPassed = false(segmentCount, 1);
for segmentIndex = 1:segmentCount
    segmentValidationPassed(segmentIndex) = ...
        segmentResults{segmentIndex}.Validation.Passed;
end
result.Validation = struct( ...
    "Passed", all(segmentValidationPassed), ...
    "SegmentValidationPassed", segmentValidationPassed, ...
    "TimeIsStrictlyIncreasing", all(diff(result.time_s) > 0), ...
    "MaximumJerkJump_deg_s3", maximumJerkJump_deg_s3, ...
    "JerkIsContinuous", all(maximumJerkJump_deg_s3 <= ...
        jerkContinuityTolerance_deg_s3), ...
    "TerminalJerk_deg_s3", result.jerk_deg_s3(end, :), ...
    "TerminalJerkIsZero", all(abs(result.jerk_deg_s3(end, :)) <= ...
        jerkContinuityTolerance_deg_s3));
result.Validation.Passed = result.Validation.Passed && ...
    result.Validation.TimeIsStrictlyIncreasing;
diagnostics.JerkJumpByJunction_deg_s3 = jerkJumpByJunction_deg_s3;
end

%% Section 4: Local Functions

function resolvedSegments = resolveSegments(segments)
% PURPOSE
%   - Apply the segment template and validate each record once.
template = azElHs3SegmentTemplate();
resolvedSegments = repmat(template, size(segments));
templateNames = string(fieldnames(template));
for segmentIndex = 1:numel(segments)
    segment = segments(segmentIndex);
    unknownNames = setdiff(string(fieldnames(segment)), ...
        templateNames, "stable");
    if ~isempty(unknownNames)
        warning("solveAzElHs3Segments:UnknownSegmentFields", ...
            "Segment %d ignored fields: %s.", segmentIndex, ...
            strjoin(unknownNames, ", "));
    end
    for nameIndex = 1:numel(templateNames)
        name = templateNames(nameIndex);
        if isfield(segment, name) && ~isempty(segment.(name))
            resolvedSegments(segmentIndex).(name) = segment.(name);
        end
    end
    resolvedSegments(segmentIndex).limits = resolveLimits( ...
        segment, template.limits, segmentIndex);
    resolvedSegments(segmentIndex) = validateSegment( ...
        resolvedSegments(segmentIndex), segmentIndex);
end
resolvedSegments = resolvedSegments(:);
end

function limits = resolveLimits(segment, defaultLimits, segmentIndex)
% PURPOSE
%   - Resolve a partial nested limits record without duplicate defaults.
limits = defaultLimits;
if ~isfield(segment, "limits") || isempty(segment.limits)
    return;
end
if ~isstruct(segment.limits) || ~isscalar(segment.limits)
    error("solveAzElHs3Segments:InvalidLimits", ...
        "Segment %d limits must be a scalar struct.", segmentIndex);
end
limitNames = string(fieldnames(defaultLimits));
unknownNames = setdiff(string(fieldnames(segment.limits)), ...
    limitNames, "stable");
if ~isempty(unknownNames)
    warning("solveAzElHs3Segments:UnknownLimitFields", ...
        "Segment %d ignored limit fields: %s.", segmentIndex, ...
        strjoin(unknownNames, ", "));
end
for nameIndex = 1:numel(limitNames)
    name = limitNames(nameIndex);
    if isfield(segment.limits, name) && ...
            ~isempty(segment.limits.(name))
        limits.(name) = segment.limits.(name);
    end
end
end

function segment = validateSegment(segment, segmentIndex)
% PURPOSE
%   - Normalize one segment and reject an invalid physical request.
vectorNames = ["startPosition_deg" "endPosition_deg" ...
    "initialVelocity_deg_s" "finalVelocity_deg_s" ...
    "initialAcceleration_deg_s2" "finalAcceleration_deg_s2"];
for nameIndex = 1:numel(vectorNames)
    name = vectorNames(nameIndex);
    value = segment.(name);
    validateattributes(value, {'numeric'}, ...
        {'real','finite','vector','numel',2});
    segment.(name) = reshape(double(value), 1, 2);
end
limitNames = ["maxVelocity_deg_s" "maxAcceleration_deg_s2" ...
    "maxJerk_deg_s3"];
for nameIndex = 1:numel(limitNames)
    name = limitNames(nameIndex);
    value = segment.limits.(name);
    validateattributes(value, {'numeric'}, ...
        {'real','finite','positive','vector','numel',2});
    segment.limits.(name) = reshape(double(value), 1, 2);
end
validateattributes(segment.maximumDuration_s, {'numeric'}, ...
    {'real','finite','scalar','positive'});
segment.maximumDuration_s = double(segment.maximumDuration_s);
segment.arrivalMode = lower(string(segment.arrivalMode));
if ~isscalar(segment.arrivalMode) || ~any( ...
        segment.arrivalMode == ["earliestarrival" "fixedarrival"])
    error("solveAzElHs3Segments:InvalidArrivalMode", ...
        "Segment %d arrivalMode must be earliestArrival or fixedArrival.", ...
        segmentIndex);
end
end

function validateSegmentContinuity(segments, tolerance)
% PURPOSE
%   - Prevent an invalid combined profile with a state jump.
for segmentIndex = 2:numel(segments)
    previous = segments(segmentIndex - 1);
    current = segments(segmentIndex);
    positionGap = max(abs( ...
        previous.endPosition_deg - current.startPosition_deg));
    velocityGap = max(abs( ...
        previous.finalVelocity_deg_s - current.initialVelocity_deg_s));
    accelerationGap = max(abs(previous.finalAcceleration_deg_s2 - ...
        current.initialAcceleration_deg_s2));
    maximumGap = max([positionGap velocityGap accelerationGap]);
    if maximumGap > tolerance
        error("solveAzElHs3Segments:DiscontinuousSegments", ...
            "Segments %d and %d have a shared-state gap of %.9g. " + ...
            "The continuity tolerance is %.9g.", ...
            segmentIndex - 1, segmentIndex, maximumGap, tolerance);
    end
end
end

function plannerOptions = segmentPlannerOptions(options, segment)
% PURPOSE
%   - Apply sandbox controls while keeping one public planner entry point.
plannerOptions = options.PlannerOptions;
plannerOptions.GoalTimeMode = segment.arrivalMode;
plannerOptions.SampleTime_s = options.SampleTime_s;
plannerOptions.MaximumPlanningTime_s = ...
    options.MaximumPlanningTimePerSegment_s;
plannerOptions.Verbose = options.Verbose;
if ~isfield(plannerOptions, "UseSpaceTimeVisibilityGraph")
    plannerOptions.UseSpaceTimeVisibilityGraph = false;
end
if ~isfield(plannerOptions, "MaximumDirectCollocationSeeds")
    plannerOptions.MaximumDirectCollocationSeeds = 1;
end
end

function result = emptyResult(segments, options)
% PURPOSE
%   - Create the stable combined result before any segment is solved.
result = struct( ...
    "Success", false, ...
    "Message", "No segment was solved.", ...
    "TerminationReason", "notStarted", ...
    "Segments", segments, ...
    "Options", options, ...
    "time_s", zeros(0, 1), ...
    "position_deg", zeros(0, 2), ...
    "velocity_deg_s", zeros(0, 2), ...
    "acceleration_deg_s2", zeros(0, 2), ...
    "jerk_deg_s3", zeros(0, 2), ...
    "maxVelocity_deg_s", zeros(0, 2), ...
    "maxAcceleration_deg_s2", zeros(0, 2), ...
    "maxJerk_deg_s3", zeros(0, 2), ...
    "SegmentIndex", zeros(0, 1), ...
    "FailedSegmentIndex", NaN, ...
    "ElapsedTime_s", 0, ...
    "Validation", struct( ...
        "Passed", false, ...
        "SegmentValidationPassed", false(0, 1), ...
        "TimeIsStrictlyIncreasing", false, ...
        "MaximumJerkJump_deg_s3", [NaN NaN], ...
        "JerkIsContinuous", false, ...
        "TerminalJerk_deg_s3", [NaN NaN], ...
        "TerminalJerkIsZero", false));
end

function result = appendSegment(result, segmentResult, limits, segmentIndex)
% PURPOSE
%   - Concatenate a certified segment without duplicating its first sample.
sampleIndex = (1:numel(segmentResult.time_s)).';
if ~isempty(result.time_s)
    sampleIndex = sampleIndex(2:end);
end
sampleCount = numel(sampleIndex);
result.time_s = [result.time_s; segmentResult.time_s(sampleIndex)];
result.position_deg = [result.position_deg; ...
    segmentResult.position_deg(sampleIndex, :)];
result.velocity_deg_s = [result.velocity_deg_s; ...
    segmentResult.velocity_deg_s(sampleIndex, :)];
result.acceleration_deg_s2 = [result.acceleration_deg_s2; ...
    segmentResult.acceleration_deg_s2(sampleIndex, :)];
result.jerk_deg_s3 = [result.jerk_deg_s3; ...
    segmentResult.jerk_deg_s3(sampleIndex, :)];
result.maxVelocity_deg_s = [result.maxVelocity_deg_s; ...
    repmat(limits.maxVelocity_deg_s, sampleCount, 1)];
result.maxAcceleration_deg_s2 = [result.maxAcceleration_deg_s2; ...
    repmat(limits.maxAcceleration_deg_s2, sampleCount, 1)];
result.maxJerk_deg_s3 = [result.maxJerk_deg_s3; ...
    repmat(limits.maxJerk_deg_s3, sampleCount, 1)];
result.SegmentIndex = [result.SegmentIndex; ...
    repmat(segmentIndex, sampleCount, 1)];
end
