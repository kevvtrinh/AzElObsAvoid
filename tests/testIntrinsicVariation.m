function tests=testIntrinsicVariation
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testIntrinsicVariation.m')
% PURPOSE: Check the bounded intrinsic snap objective against numerical integration.
% INPUTS: MATLAB unit test framework and Optimization Toolbox.
% OUTPUTS: Independent energy and cone-bound checks.
% UNITS: Physical jerk in units/s^3 and phase durations in seconds.
tests=functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testEnergyAndUpperBound(testCase)
    times_s=[0.013;0.7;8]; limits=struct('maxJerk_units_s3',[2,7]);
    count=numel(times_s); variableCount=7*count;
    map=[speye(6*count),sparse(6*count,count)];
    cones=bmtpEngine.createVariationCone(map,times_s,limits,6*count+(1:count));
    patterns={[-1,1,-1;1,-1,1],[0.2,0.2,0.2;-0.4,-0.4,-0.4], ...
        [-0.7,0.3,0.8;0.9,-0.4,0.2]};
    for pattern=patterns
        jerk=pattern{1}.*limits.maxJerk_units_s3.';
        x=[repmat(jerk(:),count,1);zeros(count,1)];
        for span=1:count
            normalized=jerk./limits.maxJerk_units_s3.';
            snap=@(tau)2/times_s(span)*((normalized(:,2)-normalized(:,1)).*(1-tau)+ ...
                (normalized(:,3)-normalized(:,2)).*tau);
            energy=times_s(span)*integral(@(tau)sum(snap(tau).^2,1),0,1);
            energy=energy/((32/3)*sum(1./times_s));
            x(6*count+span)=energy;
            cone=cones(span);
            verifyEqual(testCase,norm(cone.A*x-cone.b),cone.d.'*x-cone.gamma,'AbsTol',1e-12);
            if energy>1e-6
                smaller=x; smaller(6*count+span)=energy/2;
                verifyGreaterThan(testCase,norm(cone.A*smaller-cone.b),cone.d.'*smaller-cone.gamma);
            end
        end
        verifyLessThanOrEqual(testCase,sum(x(6*count+1:end)),1+1e-12);
    end
end
