function portfolio = evaluateArrivalCertificatePortfolio( ...
        validatedUpper_s, motionLength_deg, lowerBound_s, maximumGap_s)
%% Section 0: Header & Readme
% SYNTAX
%   emptyPortfolio = ...
%       obstacleAvoidance.planner.evaluateArrivalCertificatePortfolio()
%   portfolio = ...
%       obstacleAvoidance.planner.evaluateArrivalCertificatePortfolio( ...
%       validatedUpper_s, motionLength_deg, lowerBound_s, maximumGap_s)
%**************************************************************************
% PURPOSE
%   - Pair the best publicly validated arrival upper with the strongest
%     request-wide lower certificate without coupling their source indices.
%   - Reject a negative lower/upper gap as corrupt internal evidence.
%**************************************************************************
% INPUTS
%   - validatedUpper_s (numeric vector)
%       Authoritative trajectory durations; NaN marks an invalid candidate.
%   - motionLength_deg (numeric vector)
%       Secondary ranking value for each candidate; finite for valid uppers.
%   - lowerBound_s (numeric vector)
%       Request-wide certified lowers; NaN marks a rejected certificate.
%   - maximumGap_s (nonnegative finite scalar)
%       Positive near-infimum policy; it is not an exact-optimality claim.
%**************************************************************************
% OUTPUTS
%   - portfolio (scalar struct)
%       Stable counts, source indices, lower, upper, gap, and policy result.
%       Inconsistent bounds throw an identified corrupt-state error.
%**************************************************************************
% UNITS
%   - Arrival bounds and gaps are seconds; motion length is degrees.
%**************************************************************************

%% Section 1: Create The Stable Empty Record

portfolio = struct("AttemptedSeedCount", 0, "ValidatedCandidateCount", 0, ...
    "PassedLowerCertificateCount", 0, "MaximumInfimumGap_s", NaN, ...
    "RequestLowerBound_s", NaN, "BestValidatedUpper_s", NaN, ...
    "InfimumGap_s", NaN, "BoundsConsistent", true, ...
    "InfimumGapWithinPolicy", false, "BestLowerCertificateSeedIndex", 0, ...
    "SelectedSeedIndex", 0, "Attempts", {{}});
if nargin == 0
    return;
end

%% Section 2: Validate And Normalize Evidence Vectors

if nargin ~= 4
    error("evaluateArrivalCertificatePortfolio:InvalidCall", ...
        "Use zero inputs or supply all four evidence inputs.");
end
validateattributes(validatedUpper_s, {'numeric'}, {'real', 'vector'});
validateattributes(motionLength_deg, {'numeric'}, {'real', 'vector'});
validateattributes(lowerBound_s, {'numeric'}, {'real', 'vector'});
validatedUpper_s = double(validatedUpper_s(:));
motionLength_deg = double(motionLength_deg(:));
lowerBound_s = double(lowerBound_s(:));
if any(isinf(validatedUpper_s)) || any(isinf(motionLength_deg)) || ...
        any(isinf(lowerBound_s))
    error("evaluateArrivalCertificatePortfolio:InfiniteEvidence", ...
        "Evidence vectors may contain finite values or NaN, not Inf.");
end
candidateCount = numel(validatedUpper_s);
if numel(motionLength_deg) ~= candidateCount || ...
        numel(lowerBound_s) ~= candidateCount
    error("evaluateArrivalCertificatePortfolio:SizeMismatch", ...
        "All evidence vectors must have the same number of candidates.");
end
validateattributes(maximumGap_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validUpper = ~isnan(validatedUpper_s);
validLower = ~isnan(lowerBound_s);
if any(validatedUpper_s(validUpper) < 0) || ...
        any(lowerBound_s(validLower) < 0) || ...
        any(~isfinite(motionLength_deg(validUpper))) || ...
        any(motionLength_deg(validUpper) < 0)
    error("evaluateArrivalCertificatePortfolio:InvalidEvidence", ...
        "Available bounds and valid-candidate motion lengths must be finite and nonnegative.");
end

%% Section 3: Select Independent Upper And Lower Sources

portfolio.AttemptedSeedCount = candidateCount;
portfolio.ValidatedCandidateCount = nnz(validUpper);
portfolio.PassedLowerCertificateCount = nnz(validLower);
portfolio.MaximumInfimumGap_s = double(maximumGap_s);
if any(validUpper)
    upperIndices = find(validUpper);
    ranking = sortrows([validatedUpper_s(upperIndices), ...
        motionLength_deg(upperIndices), upperIndices], 1:3);
    portfolio.SelectedSeedIndex = ranking(1, 3);
    portfolio.BestValidatedUpper_s = ranking(1, 1);
end
if any(validLower)
    lowerIndices = find(validLower);
    [portfolio.RequestLowerBound_s, localIndex] = max(lowerBound_s(lowerIndices));
    portfolio.BestLowerCertificateSeedIndex = lowerIndices(localIndex);
end

%% Section 4: Enforce The Physical Bound Invariant

portfolio.InfimumGap_s = ...
    portfolio.BestValidatedUpper_s - portfolio.RequestLowerBound_s;
if isfinite(portfolio.InfimumGap_s) && portfolio.InfimumGap_s < 0
    portfolio.BoundsConsistent = false;
    error("evaluateArrivalCertificatePortfolio:InconsistentBounds", ...
        "Validated upper %.17g s at index %d is below lower %.17g s at index %d.", ...
        portfolio.BestValidatedUpper_s, portfolio.SelectedSeedIndex, ...
        portfolio.RequestLowerBound_s, ...
        portfolio.BestLowerCertificateSeedIndex);
end
portfolio.InfimumGapWithinPolicy = isfinite(portfolio.InfimumGap_s) && ...
    portfolio.InfimumGap_s <= portfolio.MaximumInfimumGap_s;
end
