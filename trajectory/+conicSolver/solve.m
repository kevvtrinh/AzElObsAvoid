function result = solve(problem, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = conicSolver.solve()
%   result = conicSolver.solve(problem)
%   result = conicSolver.solve(problem, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Solve one linear-objective second-order cone problem through the
%     retained conic backend and return backend-independent diagnostics.
%**************************************************************************
% INPUTS
%   - problem (scalar struct)
%       Requires Objective, Cones, InequalityMatrix, InequalityBound,
%       EqualityMatrix, EqualityBound, LowerBound, and UpperBound.
%       Kind is optional diagnostic text and defaults to "unspecified".
%   - optionOverrides (scalar struct, optional; default struct())
%       Backend defaults to "coneprog". LinearSolver defaults to "auto".
%       ConstraintTolerance, OptimalityTolerance, and MaxIterations default
%       to empty so the backend retains its documented defaults.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       A stable success-or-failure record. Invalid inputs throw; expected
%       solver failure returns Success=false and preserves solver evidence.
%       A zero-input call returns fully populated default options.
%**************************************************************************
% UNITS
%   - The interface is unit-neutral. Callers own the units of variables,
%     objectives, bounds, and residuals.
%**************************************************************************

%% Section 1: Resolve Options And Validate The Problem

defaults = struct( ...
    "Backend", "coneprog", ...
    "LinearSolver", "auto", ...
    "ConstraintTolerance", [], ...
    "OptimalityTolerance", [], ...
    "MaxIterations", []);
if nargin == 0
    result = defaults;
    return;
end
if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end
options = resolveOptions(defaults, optionOverrides);
problem = validateProblem(problem);

%% Section 2: Solve Through The Retained Backend

if options.Backend ~= "coneprog"
    error("conicSolver:UnsupportedBackend", ...
        "Backend must be ""coneprog""; observed ""%s"".", ...
        options.Backend);
end
solverOptions = optimoptions("coneprog", ...
    "Display", "none", "LinearSolver", options.LinearSolver);
if ~isempty(options.ConstraintTolerance)
    solverOptions.ConstraintTolerance = options.ConstraintTolerance;
end
if ~isempty(options.OptimalityTolerance)
    solverOptions.OptimalityTolerance = options.OptimalityTolerance;
end
if ~isempty(options.MaxIterations)
    solverOptions.MaxIterations = options.MaxIterations;
end
solveTimer = tic;
[primal, objectiveValue, exitFlag, output] = coneprog( ...
    problem.Objective, problem.Cones, ...
    problem.InequalityMatrix, problem.InequalityBound, ...
    problem.EqualityMatrix, problem.EqualityBound, ...
    problem.LowerBound, problem.UpperBound, solverOptions);
elapsedTime_s = toc(solveTimer);

%% Section 3: Assemble Stable Solver Evidence

hasFinitePrimal = ~isempty(primal) && all(isfinite(primal));
success = exitFlag > 0 && hasFinitePrimal;
if success
    terminationReason = "solved";
    message = "The conic backend returned a finite accepted solution.";
elseif exitFlag > 0
    terminationReason = "nonfiniteSolution";
    message = "The conic backend reported success without a finite solution.";
else
    terminationReason = "solverFailed";
    message = string(output.message);
end
result = struct( ...
    "Success", success, ...
    "Message", message, ...
    "TerminationReason", terminationReason, ...
    "Backend", options.Backend, ...
    "ProblemKind", problem.Kind, ...
    "PrimalSolution", primal, ...
    "ObjectiveValue", objectiveValue, ...
    "ExitFlag", exitFlag, ...
    "Output", output, ...
    "ElapsedTime_s", elapsedTime_s, ...
    "Options", options);
end

%% Section 4: Local Functions

function options = resolveOptions(defaults, overrides)
% Merge partial overrides, warn once for unknown fields, and validate values.
if ~isstruct(overrides) || ~isscalar(overrides)
    error("conicSolver:InvalidOptions", ...
        "optionOverrides must be a scalar structure or empty.");
end
options = defaults;
names = string(fieldnames(overrides));
knownNames = string(fieldnames(defaults));
unknownNames = setdiff(names, knownNames, "stable");
if ~isempty(unknownNames)
    warning("conicSolver:UnknownOptions", ...
        "Ignored unknown option fields without changing behavior: %s.", ...
        strjoin(unknownNames, ", "));
end
for name = reshape(intersect(names, knownNames, "stable"), 1, [])
    if ~isempty(overrides.(name))
        options.(name) = overrides.(name);
    end
end
options.Backend = normalizeText(options.Backend, "Backend");
options.LinearSolver = normalizeText(options.LinearSolver, "LinearSolver");
validLinearSolvers = ["auto", "augmented", "normal", "prodchol", ...
    "schur", "normal-dense"];
if ~any(options.LinearSolver == validLinearSolvers)
    error("conicSolver:InvalidLinearSolver", ...
        "LinearSolver must be one of %s; observed ""%s"".", ...
        strjoin(validLinearSolvers, ", "), options.LinearSolver);
end
validateOptionalPositiveScalar( ...
    options.ConstraintTolerance, "ConstraintTolerance");
validateOptionalPositiveScalar( ...
    options.OptimalityTolerance, "OptimalityTolerance");
if ~isempty(options.MaxIterations)
    validateattributes(options.MaxIterations, {'numeric'}, ...
        {'scalar', 'integer', 'positive', 'finite'}, ...
        "conicSolver.solve", "MaxIterations");
end
end

function problem = validateProblem(problem)
% Normalize vector orientation and reject structural dimension mismatches.
if ~isstruct(problem) || ~isscalar(problem)
    error("conicSolver:InvalidProblem", ...
        "problem must be a scalar structure.");
end
requiredNames = ["Objective", "Cones", "InequalityMatrix", ...
    "InequalityBound", "EqualityMatrix", "EqualityBound", ...
    "LowerBound", "UpperBound"];
missingNames = requiredNames(~isfield(problem, requiredNames));
if ~isempty(missingNames)
    error("conicSolver:MissingProblemFields", ...
        "problem is missing required fields: %s.", ...
        strjoin(missingNames, ", "));
end
if ~isfield(problem, "Kind") || isempty(problem.Kind)
    problem.Kind = "unspecified";
end
problem.Kind = normalizeText(problem.Kind, "problem.Kind");
problem.Objective = normalizeFiniteColumn(problem.Objective, "Objective");
variableCount = numel(problem.Objective);
problem.InequalityMatrix = normalizeMatrix( ...
    problem.InequalityMatrix, variableCount, "InequalityMatrix");
problem.InequalityBound = normalizeFiniteColumn( ...
    problem.InequalityBound, "InequalityBound");
problem.EqualityMatrix = normalizeMatrix( ...
    problem.EqualityMatrix, variableCount, "EqualityMatrix");
problem.EqualityBound = normalizeFiniteColumn( ...
    problem.EqualityBound, "EqualityBound");
problem.LowerBound = normalizeBound( ...
    problem.LowerBound, variableCount, "LowerBound");
problem.UpperBound = normalizeBound( ...
    problem.UpperBound, variableCount, "UpperBound");
if size(problem.InequalityMatrix, 1) ~= numel(problem.InequalityBound)
    error("conicSolver:InequalityCountMismatch", ...
        "InequalityMatrix rows must equal InequalityBound entries.");
end
if size(problem.EqualityMatrix, 1) ~= numel(problem.EqualityBound)
    error("conicSolver:EqualityCountMismatch", ...
        "EqualityMatrix rows must equal EqualityBound entries.");
end
if any(problem.LowerBound > problem.UpperBound)
    error("conicSolver:InvalidBounds", ...
        "LowerBound must not exceed UpperBound.");
end
end

function value = normalizeText(value, name)
% Normalize one scalar text value once at the package boundary.
if ~(isstring(value) || ischar(value)) || numel(string(value)) ~= 1
    error("conicSolver:InvalidTextOption", ...
        "%s must be scalar text.", name);
end
value = lower(string(value));
end

function values = normalizeFiniteColumn(values, name)
% Return one finite real numeric column with an explicit empty shape.
if isempty(values)
    values = zeros(0, 1);
    return;
end
validateattributes(values, {'numeric'}, ...
    {'vector', 'real', 'finite'}, "conicSolver.solve", name);
values = double(values(:));
end

function matrix = normalizeMatrix(matrix, variableCount, name)
% Return one finite matrix with the required decision-variable column count.
if isempty(matrix)
    matrix = zeros(0, variableCount);
    return;
end
validateattributes(matrix, {'numeric'}, ...
    {'2d', 'real', 'finite'}, "conicSolver.solve", name);
if size(matrix, 2) ~= variableCount
    error("conicSolver:VariableCountMismatch", ...
        "%s must have %d columns; observed %d.", ...
        name, variableCount, size(matrix, 2));
end
matrix = double(matrix);
end

function bound = normalizeBound(bound, variableCount, name)
% Return one real decision-variable bound column; signed infinity is valid.
if isempty(bound)
    bound = zeros(0, 1);
    return;
end
validateattributes(bound, {'numeric'}, ...
    {'vector', 'real', 'nonnan'}, "conicSolver.solve", name);
if numel(bound) ~= variableCount
    error("conicSolver:BoundCountMismatch", ...
        "%s must have %d entries; observed %d.", ...
        name, variableCount, numel(bound));
end
bound = double(bound(:));
end

function validateOptionalPositiveScalar(value, name)
% Validate a backend tolerance only when the caller overrides its default.
if isempty(value)
    return;
end
validateattributes(value, {'numeric'}, ...
    {'scalar', 'positive', 'finite'}, ...
    "conicSolver.solve", name);
end
