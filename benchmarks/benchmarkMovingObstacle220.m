function [record, result] = benchmarkMovingObstacle220()
%% Section 0: Header & Readme
% SYNTAX
%   [record, result] = benchmarkMovingObstacle220()
%**************************************************************************
% PURPOSE
%   - Reproduce the exact 220-vertex moving-detour timing benchmark.
%**************************************************************************
% INPUTS
%   - None.
%       Inputs are shared with exampleMovingObstacle220.
%**************************************************************************
% OUTPUTS
%   - record (scalar struct)
%       Timing, validation, motion-quality, and preparation measurements.
%   - result (scalar struct)
%       Unmodified public planner result. Planning or validation failure
%       throws an assertion error.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds. The source is synthetic planar geometry.
%**************************************************************************

%% Section 1: Create The Fixed Request Outside The Timed Solve

repositoryRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(repositoryRoot, fullfile(repositoryRoot, 'trajectory'), ...
    fullfile(repositoryRoot, 'examples'));
[source, initialState, goalState, limits] = createMovingObstacle220Scenario();

%% Section 2: Measure Planning And Validate The Result

timer  = tic;
result = planner(source, initialState, goalState, limits, ...
    struct('GoalTimeMode', 'fixedArrival'));
wallTime_s = toc(timer);

assert(result.Success, result.Message);
validation = obstacleAvoidance.validateTrajectory(result);
assert(validation.Passed, validation.Message);

record = struct( ...
    'Wall_s',                    wallTime_s, ...
    'ValidationPassed',          validation.Passed, ...
    'Length_units',              result.MotionLength_units, ...
    'Duration_s',                result.TrajectoryDuration_s, ...
    'MinimumCertifiedGap_units', result.PlaneCertificate.MinimumSignedGap_units, ...
    'SquaredJerk_units2_s5',     result.IntegratedSquaredJerk_units2_s5, ...
    'SourceSamples',             numel(source.time_s), ...
    'VerticesPerSnapshot',       220, ...
    'PreparedSamples',           nnz(result.PreparedObstacles.InternalPreparation.SamplePrepared), ...
    'PreparedIntervals',         nnz(result.PreparedObstacles.InternalPreparation.IntervalPrepared));
end
