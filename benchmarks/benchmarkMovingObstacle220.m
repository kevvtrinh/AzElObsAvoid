function [record,result] = benchmarkMovingObstacle220()
%% Section 0: Header & Readme
% SYNTAX: [record,result] = benchmarkMovingObstacle220()
% PURPOSE: Reproduce the exact 220-vertex moving-detour timing benchmark.
% INPUTS: None. All source coordinates, times, and limits are defined below.
% OUTPUTS: Timing/quality measurements and the unmodified public planner result.
% UNITS: Coordinate units and seconds. The source is synthetic planar geometry.

%% Section 1: Generate The Fixed Source And Request Outside The Timed Solve
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
time_s=(2236.5:0.25:3443.5).';
angle_rad=2*pi*(0:219).'/220;
outline_units=(20+6*cos(3*angle_rad)).*[cos(angle_rad),sin(angle_rad)];
elapsed_s=time_s-2840;
center_units=[40+0.005*elapsed_s,40+0.5*sin(elapsed_s/20)];
x_units=arrayfun(@(k)outline_units(:,1)+center_units(k,1),(1:numel(time_s)).','UniformOutput',false);
y_units=arrayfun(@(k)outline_units(:,2)+center_units(k,2),(1:numel(time_s)).','UniformOutput',false);
source=obstacleAvoidance.obstacles.createObstacle('synthetic 220-vertex history',time_s,x_units,y_units);
assert(all(cellfun(@(x)nnz(isfinite(x))==220,source.x_units)));
initial=struct('time_s',2770,'position_units',[80,0]);
goal=struct('time_s',3000,'position_units',[0,80]);
limits=struct('xInterval_units',[-180,180],'yInterval_units',[-90,90], ...
    'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75],'maxJerk_units_s3',[2,2]);

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
    'SourceSamples',numel(time_s),'VerticesPerSnapshot',220, ...
    'PreparedSamples',nnz(result.PreparedObstacles.InternalPreparation.SamplePrepared), ...
    'PreparedIntervals',nnz(result.PreparedObstacles.InternalPreparation.IntervalPrepared));
end
