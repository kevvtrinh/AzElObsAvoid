function tests = testPlannerPathIsolation
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testPlannerPathIsolation.m')
% PURPOSE: Prevent archived MATLAB packages from mixing with the current core.
% INPUTS: MATLAB unit test framework and temporary stale-package sentinels.
% OUTPUTS: Public planning and graphical-example validation checks.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setup(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    originalPath = path;
    originalFolder = pwd;
    folder = tempname; mkdir(folder);
    mkdir(fullfile(folder,'+bmtpEngine'));
    mkdir(fullfile(folder,'+obstacleAvoidance'));
    testCase.addTeardown(@() rmdir(folder,'s'));
    testCase.addTeardown(@() path(originalPath));
    testCase.addTeardown(@() cd(originalFolder));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    cd(fullfile(root,'examples'));
    testCase.TestData.Root = root;
    testCase.TestData.StaleFolder = folder;
end

function testDirectPlannerSelectsItsEngine(testCase)
    folder = testCase.TestData.StaleFolder;
    writeSentinel(fullfile(folder,'+bmtpEngine','solve.m'),'solve');
    addpath(folder,'-begin');
    verifyEqual(testCase,which('bmtpEngine.solve'),fullfile(folder,'+bmtpEngine','solve.m'));
    % Resolve the stale function once, as in a session that already ran an
    % archived example, before invoking the current public planner.
    verifyError(testCase,@() bmtpEngine.solve(),'testPlannerPathIsolation:StalePackage');
    result = planner([],struct('time_s',0,'position_units',[0,0]), ...
        struct('time_s',8,'position_units',[4,2]),struct(),struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,which('bmtpEngine.solve'), ...
        fullfile(testCase.TestData.Root,'trajectory','+bmtpEngine','solve.m'));
end

function testGraphicsKeepMatchingHelpersAndValidator(testCase)
    folder = testCase.TestData.StaleFolder;
    % A partial stale package must not override helpers even when solve itself
    % already resolves to production. The validator belongs to the same root.
    writeSentinel(fullfile(folder,'+bmtpEngine','createSolveRequest.m'),'createSolveRequest');
    writeSentinel(fullfile(folder,'+obstacleAvoidance','validateTrajectory.m'),'validateTrajectory');
    addpath(folder,'-begin');
    verifyEqual(testCase,which('bmtpEngine.createSolveRequest'), ...
        fullfile(folder,'+bmtpEngine','createSolveRequest.m'));
    previousFigures = findall(groot,'Type','figure');
    testCase.addTeardown(@() close(setdiff(findall(groot,'Type','figure'),previousFigures)));
    result = exampleObstacleFree(struct('FigureVisible','off','ShowAnimation',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Polynomial.Degree,5);
    verifyNotEmpty(testCase,setdiff(findall(groot,'Type','figure'),previousFigures));
    verifyEqual(testCase,which('bmtpEngine.createSolveRequest'), ...
        fullfile(testCase.TestData.Root,'trajectory','+bmtpEngine','createSolveRequest.m'));
    verifyEqual(testCase,which('obstacleAvoidance.validateTrajectory'), ...
        fullfile(testCase.TestData.Root,'+obstacleAvoidance','validateTrajectory.m'));
end

function writeSentinel(filePath,functionName)
    file = fopen(filePath,'w');
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,'function varargout = %s(varargin)\nerror(''testPlannerPathIsolation:StalePackage'',''Archived package was called.'');\nend\n',functionName);
end
