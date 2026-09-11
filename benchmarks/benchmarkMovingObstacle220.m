function [record,result] = benchmarkMovingObstacle220()
%% Section 0: Header & Readme
% SYNTAX: [record,result] = benchmarkMovingObstacle220()
% PURPOSE: Reproduce the exact 220-vertex moving-detour timing benchmark.
% INPUTS: None. Inputs are shared with exampleMovingObstacle220.
% OUTPUTS: Timing/quality measurements and the unmodified public planner result.
% UNITS: Coordinate units and seconds. The source is synthetic planar geometry.

%% Section 1: Generate The Fixed Source And Request Outside The Timed Solve
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
[source,initial,goal,limits]=createMovingObstacle220Scenario();

%% Section 2: Measure Planning And Recheck Independently
timer=tic;
result=planner(source,initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
wall_s=toc(timer);
assert(result.Success,result.Message);
validation=obstacleAvoidance.validateTrajectory(result);
assert(validation.Passed,validation.Message);
record=struct('Wall_s',wall_s,'ValidationPassed',validation.Passed, ...
    'Length_units',result.MotionLength_units,'Duration_s',result.TrajectoryDuration_s, ...
    'MinimumCertifiedGap_units',result.PlaneCertificate.MinimumSignedGap_units, ...
    'SquaredJerk_units2_s5',result.IntegratedSquaredJerk_units2_s5, ...
    'SourceSamples',numel(source.time_s),'VerticesPerSnapshot',220, ...
    'PreparedSamples',nnz(result.PreparedObstacles.InternalPreparation.SamplePrepared), ...
    'PreparedIntervals',nnz(result.PreparedObstacles.InternalPreparation.IntervalPrepared));
end
