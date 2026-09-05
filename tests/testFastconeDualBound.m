function tests=testFastconeDualBound
tests=functiontests(localfunctions);
end
function setupOnce(~)
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
end
function testFiniteBoxCorrectsBothSigns(testCase)
for multiplier=[0 .9 1 1.2 100]
    [bound,c]=fastcone.dualBoxBound(1,sparse(-1),-.4,multiplier,0,1,1,[]);
    verifyTrue(testCase,c.Finite);
    verifyLessThanOrEqual(testCase,bound,.4+1e-14);
    verifyEqual(testCase,bound,.4*multiplier+min(0,1-multiplier),'AbsTol',1e-13);
end
end
function testInfiniteUpperRepairsDualScale(testCase)
[bound,c]=fastcone.dualBoxBound(1,sparse(-1),-.4,1.2,0,inf,1,[]);
verifyTrue(testCase,c.Finite);
verifyLessThanOrEqual(testCase,bound,.4);
verifyGreaterThan(testCase,bound,.4-1e-12);
end
function testNoFiniteBoundIsReportedHonestly(testCase)
[bound,c]=fastcone.dualBoxBound(-1,sparse(1),0,0,0,inf,1,[]);
verifyFalse(testCase,c.Finite); verifyEqual(testCase,bound,-inf);
end
