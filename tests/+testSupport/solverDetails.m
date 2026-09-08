function details = solverDetails(diagnosis, attempt)
% Read raw solver evidence with the independent table adapter used by assertions.
if iscell(diagnosis.SolverDetails)
    details = testSupport.flattenDiagnosis(diagnosis.SolverDetails{attempt});
else
    details = diagnosis.SolverDetails(diagnosis.SolverDetails.Attempt == attempt, {'Field', 'Value'});
end
end
