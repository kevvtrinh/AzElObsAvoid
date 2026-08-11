function bound = estimateAzElLowerBound(initialState, goalState, limits)
%% Section 0: Header & Readme
% SYNTAX
%   bound = estimateAzElLowerBound(initialState, goalState, limits)
%**************************************************************************
% PURPOSE
%   - Compute an obstacle-free synchronized lower bound and exact
%     rest-to-rest double-integrator oracle where applicable.
%**************************************************************************
% INPUTS
%   - initialState (scalar complete-state struct)
%   - goalState (scalar complete-state struct)
%   - limits (scalar physical-limit struct)
%**************************************************************************
% OUTPUTS
%   - bound (scalar struct)
%       duration_s, axisDuration_s, isExactRestToRest, and model fields.
%**************************************************************************
% UNITS
%   - Duration is seconds; state and limit fields carry their named units.

positionDelta_deg = goalState.position_deg - initialState.position_deg;
axisDuration_s = zeros(1, 2);
isRestToRest = all(abs(initialState.velocity_deg_s) <= 1e-12) && ...
    all(abs(goalState.velocity_deg_s) <= 1e-12) && ...
    all(abs(initialState.acceleration_deg_s2) <= 1e-12) && ...
    all(abs(goalState.acceleration_deg_s2) <= 1e-12);

for axisIndex = 1:2
    distance_deg = abs(positionDelta_deg(axisIndex));
    maximumVelocity_deg_s = limits.maxVelocity_deg_s(axisIndex);
    maximumAcceleration_deg_s2 = ...
        limits.maxAcceleration_deg_s2(axisIndex);
    if isRestToRest
        accelerationDistance_deg = maximumVelocity_deg_s.^2 ./ ...
            maximumAcceleration_deg_s2;
        if all(distance_deg <= accelerationDistance_deg)
            axisDuration_s(axisIndex) = 2 .* sqrt( ...
                distance_deg ./ maximumAcceleration_deg_s2);
        else
            axisDuration_s(axisIndex) = ...
                distance_deg ./ maximumVelocity_deg_s + ...
                maximumVelocity_deg_s ./ maximumAcceleration_deg_s2;
        end
    else
        displacementBound_s = distance_deg ./ maximumVelocity_deg_s;
        velocityChangeBound_s = abs( ...
            goalState.velocity_deg_s(axisIndex) - ...
            initialState.velocity_deg_s(axisIndex)) ./ ...
            maximumAcceleration_deg_s2;
        axisDuration_s(axisIndex) = max( ...
            displacementBound_s, velocityChangeBound_s);
    end
end

bound = struct( ...
    "duration_s", max(axisDuration_s), ...
    "axisDuration_s", axisDuration_s, ...
    "isExactRestToRest", isRestToRest, ...
    "model", "bounded double integrator without obstacles");
end
