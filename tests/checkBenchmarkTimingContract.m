function evidence = checkBenchmarkTimingContract()
%% Section 0: Header & Readme
% SYNTAX: evidence = checkBenchmarkTimingContract()
% PURPOSE: Identify the incompatibility between the historical unconstrained
%          timing optimum and the empty core's continuous-jerk contract.
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
disp(evidence);
end
