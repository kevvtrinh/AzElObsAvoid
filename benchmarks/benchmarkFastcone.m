function results=benchmarkFastcone(cases,repetitions)
%% Section 0: Header & Readme
% SYNTAX: results=benchmarkFastcone(cases); results=benchmarkFastcone(cases,3)
% PURPOSE: Compare exact conic inputs with setup, failed attempts, and recovery.
% INPUTS: Struct array with Name and Args (nine coneprog arguments); repeats>=3.
% OUTPUTS: Per-case timing, primal residual, method, flags, and recovery counts.
% UNITS: Runtime is seconds; objective and constraints retain original units.

%% Section 1: Warm Each Identical Program
if nargin<2, repetitions=3; end
validateattributes(repetitions,{'numeric'},{'scalar','integer','>=',3});
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
destination=fullfile(root,'output'); if ~isfolder(destination), mkdir(destination); end
rows=cell(numel(cases),1); measurements=rows;
for k=1:numel(cases)
    args=cases(k).Args; assert(iscell(args) && numel(args)==9,'Each case requires nine coneprog arguments.');
    coneprog(args{:}); fastcone.solve(args{:});
    times=zeros(repetitions,2); flags=times; residuals=times;
    recoveries=false(repetitions,1); outputs=cell(repetitions,1);
    for repeat=1:repetitions
        for method=1+mod((0:1)+k+repeat,2)
            timer=tic;
            if method==1
                [x,~,flag]=coneprog(args{:});
            else
                [x,~,flag,out]=fastcone.solve(args{:});
                recoveries(repeat)=out.FallbackUsed; outputs{repeat}=out;
            end
            times(repeat,method)=toc(timer); flags(repeat,method)=flag;
            residuals(repeat,method)=fastcone.residual(x,args{:});
        end
    end
    rows{k}=struct('Case',string(cases(k).Name),'Coneprog_s',median(times(:,1)), ...
        'Fastcone_s',median(times(:,2)),'Speedup',median(times(:,1))/median(times(:,2)), ...
        'ConeprogMin_s',min(times(:,1)),'ConeprogMax_s',max(times(:,1)), ...
        'FastconeMin_s',min(times(:,2)),'FastconeMax_s',max(times(:,2)), ...
        'RecoveryCalls',nnz(recoveries),'ConeprogPositiveFlags',nnz(flags(:,1)>0), ...
        'FastconePositiveFlags',nnz(flags(:,2)>0),'ConeprogMaxResidual',max(residuals(:,1)), ...
        'FastconeMaxResidual',max(residuals(:,2)));
    measurements{k}=struct('Times_s',times,'Flags',flags,'Residuals',residuals,'Outputs',{outputs});
end

%% Section 2: Save Every Case Without Filtering Unfavorable Outcomes
results=struct2table(vertcat(rows{:}));
metadata=struct('MatlabVersion',version,'Computer',computer,'RecordedAt',datetime('now'));
save(fullfile(destination,'fastcone-benchmark.mat'),'results','cases','measurements','repetitions','metadata');
writetable(results,fullfile(destination,'fastcone-benchmark.csv')); disp(results);
end
