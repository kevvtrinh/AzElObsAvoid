function tests = testCertificateReuse
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testCertificateReuse.m')
% PURPOSE: Verify exact certificate reuse and invalidation after source edits.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Reuse, changed geometry, changed curve, and changed clearance checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end
function setupOnce(testCase)
    root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
    testCase.TestData.Request=struct('Regions_units',{{[2,-1;3,-1;3,1;2,1]}}, ...
        'Coverage',struct('Passed',true),'InitialState',struct('time_s',0));
    testCase.TestData.Motion=struct('CertifiedControlPoint_units',zeros(2,6,2),'SegmentTime_s',[1;1]);
end
function testUnchangedCurveReusesCompleteCertificate(testCase)
    request=testCase.TestData.Request; motion=testCase.TestData.Motion;
    [first,cache]=bmtpEngine.checkFinalMotion(request,[],motion,1e-8,1e-7);
    second=bmtpEngine.checkFinalMotion(request,[],motion,1e-8,1e-7,cache);
    verifyTrue(testCase,first.Passed); verifyTrue(testCase,second.Passed);
    verifyEqual(testCase,second.CachedPairCount,second.AllPairCount);
    verifyEqual(testCase,second.Planes,first.Planes);
end
function testSourceAndCurveChangesInvalidateReuse(testCase)
    request=testCase.TestData.Request; motion=testCase.TestData.Motion;
    [~,cache]=bmtpEngine.checkFinalMotion(request,[],motion,1e-8,1e-7);
    changed=request; changed.Regions_units={[-1,-1;1,-1;1,1;-1,1]};
    certificate=bmtpEngine.checkFinalMotion(changed,[],motion,1e-8,1e-7,cache);
    verifyFalse(testCase,certificate.Passed); verifyEqual(testCase,certificate.CachedPairCount,0);
    motion.CertifiedControlPoint_units(:,:,1)=2.5;
    certificate=bmtpEngine.checkFinalMotion(request,[],motion,1e-8,1e-7,cache);
    verifyFalse(testCase,certificate.Passed); verifyEqual(testCase,certificate.CachedPairCount,0);
end
function testChangedClearanceAndCoverageInvalidateReuse(testCase)
    request=testCase.TestData.Request; motion=testCase.TestData.Motion;
    [~,cache]=bmtpEngine.checkFinalMotion(request,[],motion,1e-8,1e-7);
    certificate=bmtpEngine.checkFinalMotion(request,[],motion,1e-8,3,cache);
    verifyFalse(testCase,certificate.Passed); verifyEqual(testCase,certificate.CachedPairCount,0);
    request.Coverage.Passed=false;
    certificate=bmtpEngine.checkFinalMotion(request,[],motion,1e-8,1e-7,cache);
    verifyFalse(testCase,certificate.Passed); verifyEqual(testCase,certificate.CachedPairCount,0);
end
