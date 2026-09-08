function report = auditProductionSize(maximumLineCount)
%% Section 0: Header & Readme
% SYNTAX: report = auditProductionSize(maximumLineCount)
% PURPOSE: Audit physical production size separately from code, comments, and blanks.
% INPUTS: Inclusive physical ceiling; default 15527 is strictly below the 15528-line baseline.
% OUTPUTS: Per-file and subsystem counts, total, ceiling, and pass flag.
% UNITS: Physical source lines and files; examples, tests, and reports are excluded.

%% Section 1: Enumerate Production Including The Root Planner
if nargin == 0, maximumLineCount = 15527; end
validateattributes(maximumLineCount, {'numeric'}, {'scalar','integer','positive','finite'});
repositoryRoot = fileparts(fileparts(mfilename('fullpath')));
records = [dir(fullfile(repositoryRoot,'+obstacleAvoidance','**','*.m')); ...
    dir(fullfile(repositoryRoot,'trajectory','**','*.m')); dir(fullfile(repositoryRoot,'planner.m'))];
paths = strings(numel(records),1);
subsystems = paths;
counts = zeros(numel(records),4);
for k = 1:numel(records)
    paths(k) = string(fullfile(records(k).folder,records(k).name));
    source = fileread(paths(k));
    lines = regexp(source, '\r\n|\n|\r', 'split');
    if ~isempty(lines) && isempty(lines{end}), lines(end) = []; end
    lines = strtrim(string(lines));
    blank = strlength(lines)==0;
    comment = startsWith(lines,'%');
    counts(k,:) = [numel(lines),nnz(~blank & ~comment),nnz(comment),nnz(blank)];
    paths(k) = erase(paths(k),string(repositoryRoot)+filesep);
    components = split(paths(k),filesep);
    subsystems(k) = components(1);
    if numel(components)>2, subsystems(k) = join(components(1:2),'/'); end
end

%% Section 2: Report Reproducible Counts
report = struct();
report.Rule = "Physical lines, including comments and blanks; code starts with a non-% nonspace character.";
report.Files = table(paths,subsystems,counts(:,1),counts(:,2),counts(:,3),counts(:,4), ...
    'VariableNames',{'Path','Subsystem','Physical','Code','Comment','Blank'});
names = unique(subsystems);
totals = zeros(numel(names),4);
for k = 1:numel(names), totals(k,:) = sum(counts(subsystems==names(k),:),1); end
report.Subsystems = table(names,totals(:,1),totals(:,2),totals(:,3),totals(:,4), ...
    'VariableNames',{'Subsystem','Physical','Code','Comment','Blank'});
report.FileCount = numel(records);
report.TotalLineCount = sum(counts(:,1));
report.MaximumLineCount = maximumLineCount;
report.Passed = report.TotalLineCount <= maximumLineCount;
fprintf('PRODUCTION_SIZE files=%d physical=%d code=%d comment=%d blank=%d ceiling=%d passed=%d\n', ...
    report.FileCount,sum(counts,1),maximumLineCount,report.Passed);
end
