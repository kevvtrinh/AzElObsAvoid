function [library,report] = buildExampleC3ProfileLibrary(outputFile)
%% Section 0: Header & Readme
% SYNTAX: [library,report] = buildExampleC3ProfileLibrary(outputFile)
% PURPOSE: Build a small reproducible C3 library from synthetic obstacle detours.
% INPUTS: Optional MAT output filename; omission returns data without saving.
% OUTPUTS: Independently validated profiles and every training outcome, including
%   failed solves. Add application-specific results with buildC3ProfileLibrary.
% UNITS: Coordinate units and seconds before normalization.

%% Section 1: Generate A Training Set Separate From The Existing U Example
if nargin<1, outputFile=""; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'trajectory'));
parameters=[0.90,0.95,-0.4;1.10,1.05,0.4;0.82,1.00,-0.2;1.18,1.00,0.2;0.95,0.85,-0.3;1.05,1.15,0.3];
results={};
report=repmat(struct('Case',0,'Success',false,'TerminationReason',"",'WallTime_s',0),8,1);
for index=1:8
    initial=struct('time_s',0,'position_units',[0,0]);
    goal=struct('time_s',120,'position_units',[0,-10]);
    limits=struct('maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75],'maxJerk_units_s3',[2.5,2.5]);
    vertices=[-8,7;-5,7;-5,-4;5,-4;5,7;8,7;8,-7;-8,-7];
    if index<=6
        row=parameters(index,:);
        vertices=vertices.*row(1:2);
        initial.position_units=[row(3),row(3)/2];
        goal.position_units=[0,-10*row(2)];
    elseif index==7
        vertices=[-8,7;-5,7;-5,-4;8,-4;8,-7;-8,-7];
    else
        vertices=[-4,-3;4,-3;3,3;-3,3];
        initial.position_units=[-10,0];
        goal.position_units=[10,0];
    end
    obstacles=obstacleAvoidance.obstacles.createObstacle('library training',[0;120],vertices(:,1),vertices(:,2),0.2);
    timer=tic;
    result=planner(obstacles,initial,goal,limits,struct('GoalTimeMode','earliestArrival'));
    report(index)=struct('Case',index,'Success',result.Success,'TerminationReason',result.TerminationReason,'WallTime_s',toc(timer));
    if result.Success, results{end+1}=result; end %#ok<AGROW>
    fprintf('C3 training %d/8: %s (%.3f s)\n',index,result.TerminationReason,report(index).WallTime_s);
end

%% Section 2: Revalidate, Normalize And Optionally Save The Library
library=bmtpEngine.buildC3ProfileLibrary(results,outputFile);
end
