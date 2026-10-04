function runCases(testCase, cases)
% Run each named case even when an earlier case throws or asserts.
assert(iscell(cases) && ~isempty(cases) && isvector(cases), ...
    'runCases requires a non-empty cell vector of cases.');
assert(all(cellfun(@(caseHandle) isa(caseHandle, 'function_handle'), cases(:))), ...
    'Every runCases entry must be a function handle.');

for caseHandle = reshape(cases, 1, [])
    try
        caseHandle{1}(testCase);
    catch caseException
        % Assumptions and fatal assertions keep their framework meaning.
        if isa(caseException, 'matlab.unittest.qualifications.AssumptionFailedException') || ...
                isa(caseException, 'matlab.unittest.qualifications.FatalAssertionFailedException')
            rethrow(caseException);
        end
        diagnostic = sprintf('%s:\n%s', func2str(caseHandle{1}), ...
            getReport(caseException, 'extended', 'hyperlinks', 'off'));
        verifyTrue(testCase, false, diagnostic);
    end
end
end
