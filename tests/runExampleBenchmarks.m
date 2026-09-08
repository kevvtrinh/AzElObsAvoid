function summary = runExampleBenchmarks(caseNames, repetitions)
%% Section 0: Header & Readme
% SYNTAX: summary = runExampleBenchmarks(caseNames, repetitions)
% PURPOSE: Compare unchanged example requests with the historical workbook.
% INPUTS: Optional example names and repetitions (default three).
% OUTPUTS: Per-example medians, independent validity, and benchmark gates.
% UNITS: Seconds and coordinate units; production lines include comments.

%% Section 1: Load References And Initialize
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root, fullfile(root, 'trajectory'), fullfile(root, 'examples'));
reference = readcell(fullfile(root, 'benchmarks', 'bmtp_emptycore_benchmark.xlsx'));
reference = reference(6:end, :);
if nargin < 1 || isempty(caseNames), caseNames = string(reference(:, 1)); end
if nargin < 2, repetitions = 3; end
caseNames = string(caseNames);
records = struct([]);
runs = struct([]);
subruns = struct([]);
warningState = warning;
restoreWarnings = onCleanup(@() warning(warningState)); %#ok<NASGU>
productionFiles = [dir(fullfile(root, '+obstacleAvoidance', '**', '*.m')); ...
    dir(fullfile(root, 'trajectory', '**', '*.m')); dir(fullfile(root, 'planner.m'))];
productionLines = 0;
for k = 1:numel(productionFiles)
    productionLines = productionLines + numel(splitlines(string(fileread(fullfile(productionFiles(k).folder, productionFiles(k).name))))) - 1;
end
fprintf('Production physical lines (including comments/blanks): %d\n', productionLines);

%% Section 2: Execute All Requested Cases Without Concealing Exceptions
for caseIndex = 1:numel(caseNames)
    name = caseNames(caseIndex);
    warning('error',name+":ValidationFailed");
    row = reference(strcmp(string(reference(:, 1)), name), :);
    assert(size(row, 1) == 1, 'Unknown benchmark case.');
    elapsed_s = NaN(repetitions, 1);
    arrival_s = elapsed_s; length_units = elapsed_s; routeLength_units = elapsed_s;
    passed = false(repetitions, 1); message = "";
    for repeatIndex = 1:repetitions
        try
            timer = tic;
            if nargout(name)>=3
                [result,~,caseResults] = feval(name,struct('PlotOutputs',false,'Verbose',false));
            else
                result = feval(name,struct('PlotOutputs',false,'Verbose',false));
                caseResults = {result};
            end
            elapsed_s(repeatIndex) = toc(timer);
            validation = obstacleAvoidance.validateTrajectory(result);
            expectedSuccess = logical(row{10});
            passed(repeatIndex) = result.Success == expectedSuccess && ...
                (validation.Passed || (~expectedSuccess && isempty(result.time_s) && result.TerminationReason=="noVisibilityRoute"));
            for subcaseIndex = 1:numel(caseResults)
                subcase = caseResults{subcaseIndex};
                subvalidation = obstacleAvoidance.validateTrajectory(subcase);
                valid = subcase.Success==expectedSuccess && (subvalidation.Passed || ...
                    (~expectedSuccess && isempty(subcase.time_s) && subcase.TerminationReason=="noVisibilityRoute"));
                passed(repeatIndex) = passed(repeatIndex) && valid;
                subLength_units = NaN;
                if subcase.Success, subLength_units = subcase.MotionLength_units; end
                subruns = [subruns;struct('Case',name,'Repetition',repeatIndex,'Subcase',subcaseIndex, ...
                    'Valid',valid,'Duration_s',subcase.TrajectoryDuration_s,'Length_units',subLength_units, ...
                    'PlannerTime_s',subcase.ElapsedTime_s,'Message',subcase.TerminationReason)]; %#ok<AGROW>
            end
            if result.Success
                arrival_s(repeatIndex) = result.TrajectoryDuration_s;
                length_units(repeatIndex) = result.MotionLength_units;
                routeLength_units(repeatIndex) = sum(vecnorm(diff(result.Route_units), 2, 2));
            end
            message = result.TerminationReason;
        catch exception
            elapsed_s(repeatIndex) = toc(timer);
            message = string(exception.identifier) + ": " + string(exception.message);
        end
        runs = [runs;struct('Case',name,'Repetition',repeatIndex,'Valid',passed(repeatIndex), ...
            'Duration_s',arrival_s(repeatIndex),'Length_units',length_units(repeatIndex), ...
            'RouteLength_units',routeLength_units(repeatIndex),'WallTime_s',elapsed_s(repeatIndex), ...
            'Message',message)]; %#ok<AGROW>
    end
    record = struct('Case', name, 'Valid', all(passed), ...
        'Duration_s', median(arrival_s), 'Length_units', median(length_units), ...
        'RouteLength_units', median(routeLength_units), 'WallTime_s', median(elapsed_s), ...
        'ReferenceDuration_s', numericReference(row{14}), ...
        'ReferenceLength_units', numericReference(row{13}), ...
        'ReferenceRouteLength_units', numericReference(row{12}), ...
        'ReferenceWallTime_s', row{18}, 'ProductionLines', productionLines, 'Message', message);
    record.MeetsQuality = record.Valid && (~logical(row{10}) || ...
        (record.Duration_s <= record.ReferenceDuration_s + 1e-8 && ...
        record.Length_units <= record.ReferenceLength_units + 1e-8));
    % Historical Route_units can be a blocked direct seed. Preserve the
    % reported comparison, but compare executable motion for path quality.
    record.MeetsSeedLengthReference = record.RouteLength_units <= record.ReferenceRouteLength_units+1e-8;
    record.MeetsAll = record.MeetsQuality && record.WallTime_s <= record.ReferenceWallTime_s && productionLines < 7000;
    records = [records; record]; %#ok<AGROW>
    fprintf('%s: valid=%d, duration=%.6g, length=%.6g, wall=%.4g, all=%d, %s\n', ...
        name, record.Valid, record.Duration_s, record.Length_units, record.WallTime_s, record.MeetsAll, message);
    summary = struct2table(records);
    writetable(summary, fullfile(root, 'benchmarks', 'current_summary.csv'));
    writetable(struct2table(runs),fullfile(root,'benchmarks','current_runs.csv'));
    if ~isempty(subruns), writetable(struct2table(subruns),fullfile(root,'benchmarks','current_subcase_runs.csv')); end
end
end

function value = numericReference(value)
    if ~isnumeric(value), value = str2double(string(value)); end
end
