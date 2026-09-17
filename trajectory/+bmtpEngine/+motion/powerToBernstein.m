function bernstein_units = powerToBernstein(powerCoefficient_units, degree)
%% Section 0: Header & Readme
% SYNTAX
%   bernstein_units = bmtpEngine.motion.powerToBernstein(powerCoefficient_units)
%   bernstein_units = bmtpEngine.motion.powerToBernstein(powerCoefficient_units, degree)
%**************************************************************************
% PURPOSE
%   - Convert ascending-power coefficients on normalized time to same-degree
%     Bezier controls using the one exact transform
%     nchoosek(b, p) / nchoosek(degree, p).
%**************************************************************************
% INPUTS
%   - powerCoefficient_units (P-by-M matrix or S-by-2-by-P page array)
%       Ascending powers per column, or one page of ascending powers per
%       motion span.
%   - degree (positive integer scalar, optional)
%       Target Bezier degree when the controls are elevated above the
%       supplied power count. Defaults to the exact degree the coefficients
%       already carry.
%**************************************************************************
% OUTPUTS
%   - bernstein_units ((degree+1)-by-M matrix or S-by-(degree+1)-by-2 array)
%       Control points of the identical curve.
%**************************************************************************
% UNITS
%   - Coefficients and controls share the caller's coordinate units;
%     normalized time is dimensionless.
%**************************************************************************

%% Section 1: Resolve The Requested Degree And Transform

isPageArray = ndims(powerCoefficient_units) == 3;
if isPageArray
    powerCount = size(powerCoefficient_units, 3);
else
    powerCount = size(powerCoefficient_units, 1);
end
if nargin < 2
    degree = powerCount - 1;
end
transform = createTransform(degree, powerCount);

%% Section 2: Apply The Transform To Every Supplied Page

if ~isPageArray
    bernstein_units = transform * powerCoefficient_units;
    return
end
powerPages      = permute(powerCoefficient_units, [3 1 2]);
bernstein_units = permute(pagemtimes(transform, powerPages), [2 1 3]);
end

%% Section 3: Local Functions

function transform = createTransform(degree, powerCount)
    % The weights depend only on the two sizes, so cache each one once.
    persistent transformBySize
    if isempty(transformBySize)
        transformBySize = cell(0, 0);
    end
    transformIsCached = size(transformBySize, 1) >= degree + 1 && ...
        size(transformBySize, 2) >= powerCount && ...
        ~isempty(transformBySize{degree + 1, powerCount});
    if transformIsCached
        transform = transformBySize{degree + 1, powerCount};
        return
    end
    transform = zeros(degree + 1, powerCount);
    for bernsteinIndex = 0:degree
        for powerIndex = 0:min(bernsteinIndex, powerCount - 1)
            transform(bernsteinIndex + 1, powerIndex + 1) = ...
                nchoosek(bernsteinIndex, powerIndex) / nchoosek(degree, powerIndex);
        end
    end
    transformBySize{degree + 1, powerCount} = transform;
end
