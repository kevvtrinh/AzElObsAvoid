function report = auditProductionSize(maximumLineCount)
%% Section 0: Header & Readme
% SYNTAX
%   report = auditProductionSize()
%   report = auditProductionSize(maximumLineCount)
%**************************************************************************
% PURPOSE
%   - Count nonblank, noncomment MATLAB lines in maintained production roots.
%**************************************************************************
% INPUTS
%   - maximumLineCount (positive integer scalar, optional; default 4999)
%       Inclusive feasibility ceiling for the complete production count.
%**************************************************************************
% OUTPUTS
%   - report (scalar struct)
%       Contains the count rule, file table, total, ceiling, and pass flag.
%**************************************************************************
% UNITS
%   - Counts are source lines and files.
%**************************************************************************

%% Section 1: Resolve The Maintained Production Roots

if nargin == 0
    maximumLineCount = 4999;
end
validateattributes(maximumLineCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
productionRoots = ["+obstacleAvoidance", "trajectory"];
filePaths = strings(0, 1);
for rootName = productionRoots
    rootPath = fullfile(repositoryRoot, rootName);
    if isfolder(rootPath)
        found = dir(fullfile(rootPath, "**", "*.m"));
        for foundIndex = 1:numel(found)
            filePaths(end + 1, 1) = fullfile( ...
                found(foundIndex).folder, found(foundIndex).name); %#ok<AGROW>
        end
    end
end

%% Section 2: Count Executable Source Lines

relativePath = strings(numel(filePaths), 1);
noncommentLineCount = zeros(numel(filePaths), 1);
for fileIndex = 1:numel(filePaths)
    filePath = filePaths(fileIndex);
    sourceLines = readlines(filePath);
    isExecutableLine = strlength(strtrim(sourceLines)) > 0 & ...
        ~startsWith(strtrim(sourceLines), "%");
    noncommentLineCount(fileIndex) = nnz(isExecutableLine);
    relativePath(fileIndex) = erase( ...
        string(filePath), string(repositoryRoot) + string(filesep));
end
[noncommentLineCount, order] = sort(noncommentLineCount, "descend");
relativePath = relativePath(order);
fileTable = table(relativePath, noncommentLineCount, ...
    'VariableNames', {'Path', 'NoncommentLineCount'});

%% Section 3: Assemble Reproducible Evidence

totalLineCount = sum(noncommentLineCount);
report = struct( ...
    "Rule", "Nonblank lines whose first nonspace character is not %.", ...
    "ProductionRoots", productionRoots, ...
    "Files", fileTable, "FileCount", height(fileTable), ...
    "TotalLineCount", totalLineCount, ...
    "MaximumLineCount", double(maximumLineCount), ...
    "Passed", totalLineCount <= maximumLineCount);
fprintf("PRODUCTION_SIZE files=%d lines=%d ceiling=%d passed=%d\n", ...
    report.FileCount, report.TotalLineCount, ...
    report.MaximumLineCount, report.Passed);
end
