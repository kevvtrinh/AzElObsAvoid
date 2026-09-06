function details = solverDetails(diagnosis, attempt)
% Select the flat solver evidence for one planning attempt.
details = diagnosis.SolverDetails(diagnosis.SolverDetails.Attempt == attempt, {'Field', 'Value'});
end
