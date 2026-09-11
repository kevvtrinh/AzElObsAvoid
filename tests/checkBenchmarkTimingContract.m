function evidence = checkBenchmarkTimingContract()
%% Section 0: Header & Readme
% SYNTAX: evidence = checkBenchmarkTimingContract()
% PURPOSE: Check historical timing optima against exact bounded-jerk bounds.
% INPUTS: None; physical request is from exampleObstacleFree.
% OUTPUTS: Analytic lower bound, reference difference, required jerk jumps.
% UNITS: Coordinate units, seconds, and their derivatives.

%% Section 1: Derive The Active-Axis Minimum Time
distance_units = 4; acceleration_units_s2 = 1; jerk_units_s3 = 2;
rampTime_s = acceleration_units_s2 / jerk_units_s3;
minimumTime_s = rampTime_s + sqrt(rampTime_s^2 + 4*distance_units/acceleration_units_s2);
peakVelocity_units_s = acceleration_units_s2 * (minimumTime_s/2-rampTime_s);
assert(peakVelocity_units_s < 2);
reference = readcell(fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'benchmarks', 'bmtp_emptycore_benchmark.xlsx'));
row = find(strcmp(string(reference(:,1)), 'exampleObstacleFree'));
referenceTime_s = reference{row,14};
assert(abs(referenceTime_s-minimumTime_s) < 1e-12);
% Equality in the acceleration envelope requires jerk +J, 0, -J, 0, +J.
% A continuous jerk cannot take both one-sided limits at these switches.
evidence = struct('MinimumTime_s', minimumTime_s, ...
    'ReferenceTime_s', referenceTime_s, 'Difference_s', referenceTime_s-minimumTime_s, ...
    'PeakVelocity_units_s', peakVelocity_units_s, ...
    'RequiredJerkJumps_units_s3', [-jerk_units_s3, -jerk_units_s3, jerk_units_s3, jerk_units_s3], ...
    'ContinuousJerkCanAttainBound', false);

%% Section 2: Check Other Benchmarks Against Independent Axis Bounds
% These requests reach the velocity bound. Their rest-to-rest duration is
% D/V + V/A + A/J. For the affine moving target, D=6+0.2*T, so solve that
% scalar equality. Motion in the other axis cannot lower this time bound.
caseNames = ["exampleObstacleFree";"exampleDenseConcaveObstacle"; ...
    "exampleMovingCircleNoWrap";"exampleMovingRotatingObstacleField"; ...
    "exampleInterceptMovingTargetEarliest"];
lowerBound_s = [minimumTime_s;12/2+2/1+1/2;12/2+2/1+1/2; ...
    20/3+3/1.5+1.5/4;(6/2+2/1+1/2)/(1-0.2/2)];
referenceDuration_s = zeros(size(lowerBound_s));
for k = 1:numel(caseNames)
    row = find(strcmp(string(reference(:,1)),caseNames(k)));
    referenceDuration_s(k) = reference{row,14};
end
assert(all(abs(referenceDuration_s-lowerBound_s)<1e-12));
evidence.BenchmarkBounds = table(caseNames,lowerBound_s,referenceDuration_s);
disp(evidence);
disp(evidence.BenchmarkBounds);
end
