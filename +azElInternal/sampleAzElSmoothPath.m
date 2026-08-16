function samples = sampleAzElSmoothPath(smoothPath, arcLength_deg)
%% Section 0: Header & Readme
% SYNTAX
%   samples = azElInternal.sampleAzElSmoothPath( ...
%       smoothPath, arcLength_deg)
%**************************************************************************
% PURPOSE
%   - Evaluate position and the first three arc-length derivatives of one
%     maintained line/quintic smooth path.
%**************************************************************************
% INPUTS
%   - smoothPath (scalar struct)
%       Must contain ordered Primitives and TotalLength_deg.
%   - arcLength_deg (numeric vector)
%       Cumulative path coordinates in the closed path interval.
%**************************************************************************
% OUTPUTS
%   - samples (scalar struct)
%       Position, tangent, higher path derivatives, curvature, and
%       primitive provenance for every requested coordinate.
%**************************************************************************
% UNITS
%   - Position and arc length are degrees. Second and third arc derivatives
%     use inverse degrees and inverse square degrees, respectively.
%**************************************************************************

%% Section 1: Validate The Path And Queries

requiredFields = ["Primitives", "TotalLength_deg"];
if ~isstruct(smoothPath) || ~isscalar(smoothPath) || ...
        ~all(isfield(smoothPath, requiredFields))
    error("sampleAzElSmoothPath:InvalidPath", ...
        "smoothPath must contain Primitives and TotalLength_deg.");
end
validateattributes(arcLength_deg, {'numeric'}, ...
    {'real', 'finite', 'vector'});
queryS_deg = double(arcLength_deg(:));
totalLength_deg = double(smoothPath.TotalLength_deg);
validateattributes(totalLength_deg, {'numeric'}, ...
    {'real', 'finite', 'positive', 'scalar'});
tolerance_deg = 1e-10 * max(1, totalLength_deg);
if any(queryS_deg < -tolerance_deg) || ...
        any(queryS_deg > totalLength_deg + tolerance_deg)
    error("sampleAzElSmoothPath:ArcLengthOutsidePath", ...
        "arcLength_deg spans [%.17g, %.17g] deg, outside [0, %.17g] deg.", ...
        min(queryS_deg), max(queryS_deg), totalLength_deg);
end
queryS_deg = min(max(queryS_deg, 0), totalLength_deg);

%% Section 2: Evaluate Each Primitive

sampleCount = numel(queryS_deg);
position_deg = zeros(sampleCount, 2);
tangent = zeros(sampleCount, 2);
secondDerivative_deg_inv = zeros(sampleCount, 2);
thirdDerivative_deg_inv2 = zeros(sampleCount, 2);
primitiveIndex = zeros(sampleCount, 1);
primitiveType = strings(sampleCount, 1);
primitives = smoothPath.Primitives;

for pathPrimitiveIndex = 1:numel(primitives)
    primitive = primitives(pathPrimitiveIndex);
    requiredPrimitiveFields = ["Type", "StartArcLength_deg", ...
        "EndArcLength_deg", "Length_deg"];
    if ~all(isfield(primitive, requiredPrimitiveFields))
        error("sampleAzElSmoothPath:InvalidPrimitive", ...
            "Primitive %d does not contain the maintained schema.", ...
            pathPrimitiveIndex);
    end

    % Half-open ownership at internal joins prevents geometry derivatives
    % from one primitive being paired with timing data from its neighbor.
    belongs = queryS_deg >= primitive.StartArcLength_deg;
    if pathPrimitiveIndex < numel(primitives)
        belongs = belongs & queryS_deg < primitive.EndArcLength_deg;
    else
        belongs = belongs & queryS_deg <= primitive.EndArcLength_deg;
    end
    belongs = belongs & primitiveIndex == 0;
    if ~any(belongs)
        continue;
    end

    localS_deg = min(max(queryS_deg(belongs) - ...
        primitive.StartArcLength_deg, 0), primitive.Length_deg);
    primitiveKind = lower(string(primitive.Type));
    if primitiveKind == "line"
        position_deg(belongs, :) = primitive.StartPosition_deg + ...
            localS_deg .* primitive.Direction;
        tangent(belongs, :) = repmat( ...
            primitive.Direction, nnz(belongs), 1);
    elseif primitiveKind == "quintic"
        parameter = interp1(primitive.ArcLengthGrid_deg, ...
            primitive.ParameterGrid, localS_deg, "pchip");
        [position, first, parameterSecond, parameterThird] = ...
            azElInternal.evaluateAzElQuintic( ...
            primitive.ControlPoints_deg, ...
            min(max(parameter, 0), 1));
        parameterSpeed_deg = vecnorm(first, 2, 2);
        firstSecond = sum(first .* parameterSecond, 2);
        speedSquared_deg2 = parameterSpeed_deg.^2;
        second = parameterSecond ./ speedSquared_deg2 - ...
            first .* firstSecond ./ parameterSpeed_deg.^4;
        third = parameterThird ./ parameterSpeed_deg.^3 - ...
            3 * parameterSecond .* firstSecond ./ ...
            parameterSpeed_deg.^5 - ...
            first .* (sum(parameterSecond.^2, 2) + ...
            sum(first .* parameterThird, 2)) ./ ...
            parameterSpeed_deg.^5 + ...
            4 * first .* firstSecond.^2 ./ parameterSpeed_deg.^7;
        position_deg(belongs, :) = position;
        tangent(belongs, :) = first ./ parameterSpeed_deg;
        secondDerivative_deg_inv(belongs, :) = second;
        thirdDerivative_deg_inv2(belongs, :) = third;
    else
        error("sampleAzElSmoothPath:UnsupportedPrimitive", ...
            "Primitive %d has unsupported Type '%s'.", ...
            pathPrimitiveIndex, primitiveKind);
    end
    primitiveIndex(belongs) = pathPrimitiveIndex;
    primitiveType(belongs) = primitiveKind;
end

if any(primitiveIndex == 0)
    error("sampleAzElSmoothPath:PrimitiveCoverage", ...
        "Smooth primitives do not cover every requested arc length.");
end

%% Section 3: Assemble The Stable Sample Record

samples = struct( ...
    "arcLength_deg", queryS_deg, ...
    "position_deg", position_deg, ...
    "tangent", tangent, ...
    "secondDerivative_deg_inv", secondDerivative_deg_inv, ...
    "thirdDerivative_deg_inv2", thirdDerivative_deg_inv2, ...
    "curvature_deg_inv", vecnorm(secondDerivative_deg_inv, 2, 2), ...
    "PrimitiveIndex", primitiveIndex, ...
    "PrimitiveType", primitiveType);
end
