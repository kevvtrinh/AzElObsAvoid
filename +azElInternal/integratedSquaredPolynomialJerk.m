function value_deg2_s5 = integratedSquaredPolynomialJerk(polynomial)
%% Section 0: Header & Readme
% SYNTAX
%   value_deg2_s5 = azElInternal.integratedSquaredPolynomialJerk(polynomial)
%**************************************************************************
% PURPOSE
%   - Integrate squared physical jerk exactly from the shared ascending-power
%     polynomial representation so different motion backends receive one
%     method-neutral quality score.
%**************************************************************************

%% Section 1: Validate The Shared Polynomial Record

requiredFields = {'SegmentCount', 'SegmentDuration_s', 'jerkPower_deg_s3'};
if ~isstruct(polynomial) || ~isscalar(polynomial) || ...
        ~all(isfield(polynomial, requiredFields))
    error("azElInternal:integratedSquaredPolynomialJerk:InvalidPolynomial", ...
        "polynomial must contain SegmentCount, SegmentDuration_s, and jerkPower_deg_s3.");
end
segmentCount = double(polynomial.SegmentCount);
coefficient = double(polynomial.jerkPower_deg_s3);
if segmentCount == 0
    value_deg2_s5 = Inf;
    return;
end
validateattributes(segmentCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
if ndims(coefficient) ~= 3 || size(coefficient, 1) ~= segmentCount || ...
        isempty(coefficient) || any(~isfinite(coefficient), "all")
    error("azElInternal:integratedSquaredPolynomialJerk:InvalidCoefficients", ...
        "jerkPower_deg_s3 must be a finite segment-by-axis-by-power array.");
end
duration_s = double(polynomial.SegmentDuration_s(:));
if isscalar(duration_s)
    duration_s = repmat(duration_s, segmentCount, 1);
end
validateattributes(duration_s, {'numeric'}, ...
    {'real', 'finite', 'column', 'numel', segmentCount, 'positive'});

%% Section 2: Integrate Each Segment Analytically

powerCount = size(coefficient, 3);
[leftPower, rightPower] = ndgrid(0:powerCount - 1);
integralGram = 1 ./ (leftPower + rightPower + 1);
value_deg2_s5 = 0;
for segmentIndex = 1:segmentCount
    segmentCoefficient = reshape( ...
        coefficient(segmentIndex, :, :), size(coefficient, 2), powerCount);
    value_deg2_s5 = value_deg2_s5 + duration_s(segmentIndex) * ...
        sum((segmentCoefficient.' * segmentCoefficient) .* integralGram, "all");
end
end
