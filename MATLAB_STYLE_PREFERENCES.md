# MATLAB Style Preferences

This document records the preferred presentation style for maintained MATLAB
code in this repository. Style-only work must preserve behavior, public fields,
validation strength, geometry, tolerances, and deterministic results.

Use `ApplyCommentAndStyle.md` as the current reference for the user's commenting,
naming, and readability preferences. This document supplies the matching MATLAB
formatting rules. Update both together when a preference changes.

## Public Help Blocks

Begin each public function with:

```matlab
function result = functionName(requiredInput, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   defaults = functionName()
%   result = functionName(requiredInput)
%   result = functionName(requiredInput, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Describe the function's responsibility.
%**************************************************************************
% INPUTS
%   - requiredInput (type and shape)
%       Give a short description of its meaning and constraints.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Describe the stable success-or-failure result.
%**************************************************************************
% UNITS
%   - State physical units and coordinate ordering.
%**************************************************************************
```

- Keep the blocks in this exact order: `SYNTAX`, `PURPOSE`, `INPUTS`,
  `OUTPUTS`, `UNITS`.
- Separate blocks with `%` followed by exactly 74 asterisks.
- Under `SYNTAX`, list only intended, acceptable public calls. A documented
  zero-input defaults call must return defaults rather than run a scenario. Do
  not advertise every technically possible partial-input or internal call.
- Keep input descriptions concise. Include only details needed to use the
  interface correctly.
- Mention expected failure behavior in `OUTPUTS`: ordinary planning failure
  returns `Success = false`, while invalid input throws an error.
- Local functions need only one to five concise purpose or invariant comments.

## Executable Sections

- Use numbered, title-cased sections in execution order.
- Section names must describe the actual work performed.
- Do not place top-level `%%` sections inside loops or conditionals.
- Use a short, plain comment for an internal stage that does not warrant a new
  top-level section. Do not add decorative `---` separators.

Runnable examples will use this top-level order:

```matlab
%% Section 0: Header & Readme
%% Section 1: Resolve Example Controls
%% Section 2: Create Obstacles
%% Section 3: Create Planner Inputs
%% Section 4: Run Planner
%% Section 5: Validate Result
%% Section 6: Plot Diagnostics And Motion
```

## Spacing And Alignment

- Put one space on both sides of comparison and binary arithmetic operators:

```matlab
options.GoalTimeMode == "fixedArrival"
departureCandidate.ArrivalTime_s <= timedResult.ArrivalTime_s
```

- Put a space after commas and semicolons used as list separators:

```matlab
[candidate, diagnostics] = solve(seed, limits, options);
route_units = [start_units; goal_units];
```

- Align assignment operators within short, related blocks:

```matlab
plannerFolder  = fileparts(mfilename('fullpath'));
engineFolder   = fullfile(plannerFolder, 'trajectory');
productionPath = [plannerFolder pathsep engineFolder];
```

- Align related structure fields or name-value pairs when that makes the data
  easier to scan.
- Do not align unrelated statements across distant control-flow branches.

## Line Layout

- Do not compress control flow onto one line.

```matlab
if result.Success
    return
end
```

- Keep a complete statement on one line whenever it remains readable. Reserve
  `...` for genuinely long or structurally complex statements.
- Keep descriptive names when a statement needs to wrap. Break the call at a
  natural argument boundary instead of abbreviating the values.

```matlab
[~, targetVelocity_units_s, targetAcceleration_units_s2] = ...
    targetPositionAtTime(targetMotion, time_s);
```

- Break long function calls at logical argument boundaries, not at arbitrary
  points in an expression.
- Keep short lists and simple diagnostic calls compact. Do not place every list
  item or argument on its own line when two or three readable lines suffice.
- Use blank lines to separate validation phases instead of surrounding every
  statement with additional formatting.
- Give each substantial `struct` field its own line. Align name-value pairs:

```matlab
options = struct( ...
    "GoalTimeMode",        "fixedArrival", ...
    "SampleTime_s",        0.05, ...
    "MaxArrivalTrials",    100);
```

## Readable Logic

- Replace dense nested expressions with named intermediate conditions.
- Prefer a straightforward loop when it makes first-match behavior easier to
  understand than nested `arrayfun`, `find`, and anonymous-function logic.
- Name Boolean intermediates as assertions, such as
  `intervalIsUnsupported` and `intervalOverlapsRequest`.
- Separate selection, message construction, and failure-result assembly into
  visually distinct blocks.
- Do not add helper layers solely to hide readable one-owner logic.

## Naming And Units

- Use descriptive lower-camel-case names and physical-unit suffixes.
- Use names that explain the value's role, such as `targetVelocity_units_s` and
  `targetAcceleration_units_s2`. Avoid abbreviations such as `tgtVel` and `tgtAcc`.
- Prefer `planningEnvironment` for prepared obstacle data and
  `arrivalPlanningProgress` for the result and attempts carried between arrival
  planning stages. Keep physical state names such as `initialState` and
  `goalState`; their meaning is already specific.
- Use singular names for one record and plural names for collections.
- End indices with `Index`, counts with `Count`, and graphics handles with
  `Handle` or `Handles`.
- Preserve established public field and option names for compatibility.

## Verification For Style-Only Changes

- Review the final diff to confirm that edits are limited to presentation or a
  clearly equivalent readability refactor.
- Run `git diff --check`.
- Run MATLAB `checkcode` on every edited MATLAB file.
- Run focused behavioral tests whenever readability work changes expression or
  control-flow structure.
