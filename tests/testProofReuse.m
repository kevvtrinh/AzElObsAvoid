function tests = testProofReuse
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testProofReuse.m')
% PURPOSE: Verify exact proof reuse and invalidation after source edits.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Reuse, changed geometry, changed curve, and changed clearance checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end
function setupOnce(testCase)
    root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
    testCase.TestData.Request=struct('Regions_units',{{[2,-1;3,-1;3,1;2,1]}}, ...
        'Coverage',struct('Passed',true),'InitialState',struct('time_s',0));
    testCase.TestData.Motion=struct('ControlPoint_units',zeros(2,6,2),'SegmentTime_s',[1;1], ...
        'FinalTime_s',2);
end
function testUnchangedCurveReusesCompleteProof(testCase)
    request=testCase.TestData.Request; motion=testCase.TestData.Motion;
    [first,cache]=bmtpEngine.validation.checkFinalMotion(request,motion,1e-8,1e-7);
    second=bmtpEngine.validation.checkFinalMotion(request,motion,1e-8,1e-7,cache);
    verifyTrue(testCase,first.Passed); verifyTrue(testCase,second.Passed);
    verifyEqual(testCase,second.CachedPairCount,second.AllPairCount);
    verifyEqual(testCase,second.Planes,first.Planes);
end
function testSourceAndCurveChangesInvalidateReuse(testCase)
    request=testCase.TestData.Request; motion=testCase.TestData.Motion;
    [~,cache]=bmtpEngine.validation.checkFinalMotion(request,motion,1e-8,1e-7);
    changed=request; changed.Regions_units={[-1,-1;1,-1;1,1;-1,1]};
    proof=bmtpEngine.validation.checkFinalMotion(changed,motion,1e-8,1e-7,cache);
    verifyFalse(testCase,proof.Passed); verifyEqual(testCase,proof.CachedPairCount,0);
    motion.ControlPoint_units(:,:,1)=2.5;
    proof=bmtpEngine.validation.checkFinalMotion(request,motion,1e-8,1e-7,cache);
    verifyFalse(testCase,proof.Passed); verifyEqual(testCase,proof.CachedPairCount,0);
end
