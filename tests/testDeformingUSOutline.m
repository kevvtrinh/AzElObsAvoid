function tests = testDeformingUSOutline
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testDeformingUSOutline.m')
% PURPOSE: Keep the reduced deforming U.S. outline example supported and exact.
% INPUTS: MATLAB unit test framework and the maintained deforming U.S. example.
% OUTPUTS: Outline cap, interval model, independent validity, and quality checks.
% UNITS: Coordinate units, seconds, and physical derivatives.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
end

function testReducedOutlineIsSupportedAndValid(testCase)
    result = exampleMovingDeformingUSOutlineVisibility(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyEqual(testCase,result.TerminationReason,"goalReached");
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    % The supplied outline is the 100-vertex reduction of the 14613-vertex
    % source ring at every sample; the planner treats it exactly.
    outline = result.Diagnostics.PreparedObstacles(1);
    verifyEqual(testCase,cellfun(@numel,outline.originalX_units),100*ones(numel(outline.time_s),1));
    verifyEqual(testCase,string(outline.vertexCorrespondence),"sourceIndex");
    preparation = outline.InternalPreparation;
    verifyTrue(testCase,all(preparation.IntervalPrepared));
    verifyFalse(testCase,any(preparation.IntervalGeometryModel=="unsupportedContinuousDeformation"));
    verifyFalse(testCase,any(contains(preparation.IntervalGeometryModel,"ConvexHull",'IgnoreCase',true)));
    verifyLessThanOrEqual(testCase,max(preparation.IntervalMovingCellUncoveredProtectedArea_units2,[],'all'), ...
        4096*eps(max(1,max(cellfun(@area,preparation.SampleShapes)))));
    verifyEqual(testCase,result.ArrivalTime_s,18.5752,'RelTol',0.01);
    verifyLessThanOrEqual(testCase,abs(result.MotionLength_units/40.5138437-1),0.01);
    verifyGreaterThan(testCase,result.Diagnostics.SeparationProof.MinimumSignedGap_units,0);
end
