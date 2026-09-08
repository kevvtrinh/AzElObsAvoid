function value = diagnosisValue(details, field)
% Read one leaf from an optional diagnostic detail table.
if ~istable(details)
    details = testSupport.flattenDiagnosis(details);
end
index = find(details.Field == field);
assert(isscalar(index), 'Expected exactly one diagnostic field: %s', field);
value = details.Value{index};
end
