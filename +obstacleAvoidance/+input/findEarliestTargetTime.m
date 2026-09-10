function time_s = findEarliestTargetTime(target, initialState, horizon_s, limits)
%% Section 0: Header & Readme
% SYNTAX: time = obstacleAvoidance.input.findEarliestTargetTime(target, initial, horizon, limits)
% PURPOSE: Find the first rest-to-rest reachable point of a sampled target.
% INPUTS: Linear/pchip target, initial state, absolute horizon, axis limits.
% OUTPUTS: Earliest kinematically reachable time, or NaN if none exists.
% UNITS: Coordinate units, seconds, and physical derivative limits.

%% Section 1: Partition At Target And Reachability Polynomial Changes
obstacleAvoidance.input.targetPositionAtTime(target,horizon_s);
method = 'linear';
if isfield(target,'InterpolationMethod'), method = char(target.InterpolationMethod); end
velocity_units_s = limits.maxVelocity_units_s;
acceleration_units_s2 = limits.maxAcceleration_units_s2;
jerk_units_s3 = limits.maxJerk_units_s3;
ramp_s = min(acceleration_units_s2./jerk_units_s3,sqrt(velocity_units_s./jerk_units_s3));
hold_s = max(0,velocity_units_s./(jerk_units_s3.*ramp_s)-ramp_s);
cruiseStart_s = 2*(2*ramp_s+hold_s);
accelerationStart_s = 4*ramp_s;
breaks_s = unique([target.time_s(:);initialState.time_s;horizon_s; ...
    initialState.time_s+accelerationStart_s(:);initialState.time_s+cruiseStart_s(:)]);
breaks_s = breaks_s(breaks_s>=max(initialState.time_s,target.time_s(1)) & breaks_s<=horizon_s);
targetPolynomial = cell(1,2);
for axis = 1:2
    targetPolynomial{axis} = interp1(target.time_s,target.position_units(:,axis),method,'pp');
end
time_s = NaN;

%% Section 2: Intersect All Polynomial Reachability Inequalities
% Maximum rest-to-rest displacement is cubic before acceleration saturation,
% quadratic before velocity saturation, and linear during cruise. All target
% crossings can therefore be enumerated by cubic roots, including intervals
% that enter and leave the reachable set between two target samples.
for interval = 1:numel(breaks_s)-1
    start_s = breaks_s(interval);
    duration_s = breaks_s(interval+1)-start_s;
    midpoint_s = start_s+duration_s/2-initialState.time_s;
    boundaries = zeros(4,4);
    cuts_s = [0;duration_s];
    for axis = 1:2
        v = velocity_units_s(axis); a = acceleration_units_s2(axis); j = jerk_units_s3(axis);
        if midpoint_s < accelerationStart_s(axis)
            reach = [0,0,0,j/32];
        elseif midpoint_s < cruiseStart_s(axis)
            reach = [0,-a^2/(2*j),a/4,0];
        else
            reach = [-v*(2*ramp_s(axis)+hold_s(axis)),v,0,0];
        end
        reach = shiftPowers(reach,start_s-initialState.time_s);
        pp = targetPolynomial{axis};
        piece = find(pp.breaks<=start_s,1,'last');
        piece = min(piece,pp.pieces);
        position = zeros(1,4);
        position(1:pp.order) = fliplr(pp.coefs(piece,:));
        position = shiftPowers(position,start_s-pp.breaks(piece));
        position(1) = position(1)-initialState.position_units(axis);
        for side = 1:2
            row = 2*(axis-1)+side;
            boundaries(row,:) = (2*side-3)*position-reach;
            crossing_s = roots(fliplr(boundaries(row,:)));
            crossing_s = real(crossing_s(abs(imag(crossing_s))<=64*eps(max(1,abs(crossing_s)))));
            cuts_s = [cuts_s;crossing_s(crossing_s>=0 & crossing_s<=duration_s)]; %#ok<AGROW>
        end
    end
    cuts_s = unique(cuts_s);
    for k = 1:numel(cuts_s)
        residual = boundaries*[1;cuts_s(k);cuts_s(k)^2;cuts_s(k)^3];
        roundoff = 128*eps(max(1,max(abs(boundaries),[],'all')))*max(1,duration_s^3);
        atBoundary = all(residual<=roundoff);
        inInterval = false;
        if k<numel(cuts_s)
            middle_s = (cuts_s(k)+cuts_s(k+1))/2;
            inInterval = all(boundaries*[1;middle_s;middle_s^2;middle_s^3]<=0);
        end
        if (atBoundary || inInterval) && start_s+cuts_s(k)>initialState.time_s
            time_s = start_s+cuts_s(k);
            return;
        end
    end
end
end

function shifted = shiftPowers(powers,offset)
    % Translate a cubic's local origin using the binomial identity.
    shifted = [powers(1)+offset*(powers(2)+offset*(powers(3)+offset*powers(4))), ...
        powers(2)+offset*(2*powers(3)+3*offset*powers(4)), ...
        powers(3)+3*offset*powers(4),powers(4)];
end
