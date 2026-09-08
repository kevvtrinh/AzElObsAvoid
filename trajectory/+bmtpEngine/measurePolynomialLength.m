function length_units = measurePolynomialLength(polynomial)
%% Section 0: Header & Readme
% SYNTAX: length_units = bmtpEngine.measurePolynomialLength(polynomial)
% PURPOSE: Integrate actual polynomial speed independently of display samples.
% INPUTS: Engine polynomial with normalized positionPower_units coefficients.
% OUTPUTS: Adaptive arc length, using positive 16/32-point quadrature rules.
%   This is a numerical length measurement, not a motion safety certificate.
% UNITS: Coordinate units; normalized-time integration cancels span duration.

%% Section 1: Reuse Only Input-Independent Quadrature Constants
persistent nodes weights
if isempty(nodes)
    nodes = cell(1,2); weights = nodes;
    for rule = 1:2
        order = 16 * rule;
        index = (1:order-1).'; offDiagonal = index ./ sqrt(4*index.^2-1);
        [vectors, values] = eig(diag(offDiagonal,1)+diag(offDiagonal,-1));
        nodes{rule} = (diag(values).'+1)/2;
        weights{rule} = vectors(1,:).'.^2;
    end
end
coefficients = polynomial.positionPower_units;
degree = size(coefficients,3)-1;
derivatives = permute(coefficients(:,:,2:end).*reshape(1:degree,1,1,[]),[1 3 2]);
spanCount = size(coefficients,1);
length_units = 0;

%% Section 2: Refine Only Unresolved Integration Intervals
% Bound the largest speed-evaluation array to 65,536 quadrature points.
for batchStart = 1:2048:spanCount
    owners = (batchStart:min(spanCount,batchStart+2047)).';
    left = zeros(size(owners)); right = ones(size(owners));
    for depth = 0:12
        estimates = zeros(numel(owners),2);
        for rule = 1:2
            tau = left+(right-left).*nodes{rule};
            velocity = zeros(numel(owners),numel(nodes{rule}),size(coefficients,2));
            for order = degree:-1:1
                velocity = velocity.*tau+derivatives(owners,order,:);
            end
            estimates(:,rule) = (right-left).*(sqrt(sum(velocity.^2,3))*weights{rule});
        end
        tolerance_units = max(1e-11*(right-left)/spanCount,1e-11*abs(estimates(:,2)));
        accepted = abs(diff(estimates,1,2)) <= tolerance_units;
        length_units = length_units+sum(estimates(accepted,2));
        owners = owners(~accepted); left = left(~accepted); right = right(~accepted);
        if isempty(owners), break; end
        if depth == 12 || numel(owners)>1024
            for interval = 1:numel(owners)
                derivative = reshape(derivatives(owners(interval),:,:),degree,[]).';
                speed = @(tau) sqrt(sum(evaluateAxes(derivative,tau).^2,1));
                length_units = length_units+integral(speed,left(interval),right(interval),'AbsTol',1e-11*(right(interval)-left(interval))/spanCount,'RelTol',1e-11);
            end
            break;
        end
        middle = (left+right)/2;
        owners = [owners;owners];
        [left,right] = deal([left;middle],[middle;right]);
    end
end
end

function values = evaluateAxes(coefficients,tau)
    values = zeros(size(coefficients,1),numel(tau));
    for axisIndex = 1:size(coefficients,1)
        values(axisIndex,:) = polyval(fliplr(coefficients(axisIndex,:)),tau);
    end
end
