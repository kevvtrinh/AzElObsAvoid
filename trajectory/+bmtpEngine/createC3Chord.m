function [controls_units,durations_s,powers_units] = createC3Chord(start_units,goal_units,limits)
%% Section 0: Header & Readme
% SYNTAX: [controls,times,powers] = bmtpEngine.createC3Chord(start,goal,limits)
% PURPOSE: Smooth a rest-to-rest chord into C3 quintic spans with zero end jerk.
% INPUTS: Distinct endpoints and validated positive per-axis derivative limits.
% OUTPUTS: Quintic Bernstein controls, durations, and normalized position powers.
% UNITS: Coordinate units, seconds, and physical derivative limits.

%% Section 1: Smooth Scalar Jerk With A Positive Triangular Kernel
% Convolution preserves derivative bounds and total displacement. Two causal
% box averages add twice the smoothing width to the original motion duration.
[~,originalTimes_s,phases] = bmtpEngine.createJerkLimitedChord([0,0],[1,0], ...
    struct('maxVelocity_units_s',min(limits.maxVelocity_units_s./abs(goal_units-start_units))*[1,1], ...
    'maxAcceleration_units_s2',min(limits.maxAcceleration_units_s2./abs(goal_units-start_units))*[1,1], ...
    'maxJerk_units_s3',min(limits.maxJerk_units_s3./abs(goal_units-start_units))*[1,1]),5);
width_s = min(originalTimes_s)/10;
sourceBreaks_s = [0;cumsum(originalTimes_s)];
breaks_s = unique([sourceBreaks_s;sourceBreaks_s+width_s;sourceBreaks_s+2*width_s]);
% Coalescing roundoff-equivalent knots does not change the source clock.
breaks_s = breaks_s([true;diff(breaks_s)>64*eps(max(breaks_s))]);
durations_s = diff(breaks_s);
powers_units = zeros(numel(durations_s),2,6);
controls_units = zeros(numel(durations_s),6,2);
conversion = zeros(6);
for k=0:5
    for j=0:k, conversion(k+1,j+1)=nchoosek(k,j)/nchoosek(5,j); end
end
position=0; velocity_s1=0; acceleration_s2=0;
direction_units=goal_units-start_units;

%% Section 2: Integrate Quadratic Jerk Into Quintic Position
for span=1:numel(durations_s)
    h=durations_s(span); t=breaks_s(span)+h*[0,0.5,1];
    j=(originalVelocity(t)-2*originalVelocity(t-width_s)+originalVelocity(t-2*width_s))/width_s^2;
    jerkPower=[j(1),4*j(2)-3*j(1)-j(3),2*j(1)+2*j(3)-4*j(2)];
    power=[position,velocity_s1*h,acceleration_s2*h^2/2,jerkPower.*h^3./[6,24,60]];
    physical=direction_units.'*power; physical(:,1)=physical(:,1)+start_units.';
    powers_units(span,:,:)=physical;
    controls_units(span,:,:)=conversion*physical.';
    position=sum(power);
    velocity_s1=sum((1:5).*power(2:6))/h;
    acceleration_s2=sum((1:4).*(2:5).*power(3:6))/h^2;
end

    function velocity=originalVelocity(times_s)
        velocity=zeros(size(times_s));
        for k=1:numel(times_s)
            if times_s(k)<=0 || times_s(k)>=sourceBreaks_s(end), continue; end
            phase=find(sourceBreaks_s<=times_s(k),1,'last');
            elapsed_s=times_s(k)-sourceBreaks_s(phase);
            velocity(k)=phases.Velocity_units_s(phase,1)+elapsed_s*phases.Acceleration_units_s2(phase,1)+elapsed_s^2*phases.Jerk_units_s3(phase,1)/2;
        end
    end
end
