function descriptor = describeC3Profile(route_units,limits)
%% Section 0: Header & Readme
% SYNTAX: descriptor = bmtpEngine.describeC3Profile(route_units,limits)
% PURPOSE: Describe route geometry and derivative limits independent of origin.
% INPUTS: Nondegenerate route and resolved physical limits.
% OUTPUTS: Chord frame, normalized route samples, and dimensionless rate limits.
% UNITS: Coordinate units, seconds, and their derivative units on input.

%% Section 1: Normalize Geometry And Characteristic Time
origin_units=route_units(1,:);
displacement_units=route_units(end,:)-origin_units;
distance_units=norm(displacement_units);
direction=displacement_units/distance_units;
frame=[direction(:),[-direction(2);direction(1)]];
route=(route_units-origin_units)*frame/distance_units;
arc=[0;cumsum(vecnorm(diff(route),2,2))];
keep=[true;diff(arc)>0];
samples=interp1(arc(keep)/arc(end),route(keep,:),linspace(0,1,17).');
rates=[limits.maxVelocity_units_s;limits.maxAcceleration_units_s2;limits.maxJerk_units_s3];
characteristicTime_s=max((distance_units./min(rates,[],2)).^(1./(1:3).'));
signature=zeros(3,2);
for order=1:3
    % Pure motion along each canonical axis must obey both world-axis limits.
    canonicalRates=1./max(abs(frame)./rates(order,:).',[],1);
    signature(order,:)=canonicalRates*characteristicTime_s^order/distance_units;
end
descriptor=struct('Origin_units',origin_units,'Distance_units',distance_units, ...
    'Frame',frame,'Route',route,'RouteSamples',samples,'LimitSignature',signature);
end
