# ApplyCommentAndStyle

This is the current reference for the user's comments, names, and code layout.
Use it when reviewing MATLAB files together. Write for a junior or
associate engineer who understands basic programming but is learning the planner.
Follow `MATLAB_STYLE_PREFERENCES.md` for the help-block structure and MATLAB
formatting. The preferences below take precedence for comments and naming.

## Comments

- Use plain, direct language while keeping the engineering meaning accurate.
- Explain what a non-obvious block does and why. Do not narrate routine syntax.
- Keep comments short, close to the code, and specific to the calculation.
- Prefer everyday wording over terms such as "provenance," "residual,"
  "broadcast," "query domain," or "outer bound." Explain technical terms when
  they are necessary.
- Use short equations and symbols, including `x` for multiplication in comments:
  `maximum travel distance = maximum speed x available time`.
- Use concrete numbers when they make a concept easier to understand:
  `start = 350, goal = 10, loop length = 360; use goal = 370, distance = 20`.
- Describe edge cases by the result: "If two copies are equally close, choose
  the higher coordinate."
- State limitations accurately. A speed-based travel limit does not guarantee
  that acceleration limits and obstacles allow the vehicle to reach it.
- Use plain comments without decorative `---` separators.

## Layout And Names

- Separate logical steps with blank lines: check structure, fill defaults,
  handle special cases, calculate results, and format outputs.
- Keep numbered top-level sections for substantial stages. Use short plain
  comments for smaller steps; avoid headings around every statement.
- Keep the main function body flush left and local helper bodies indented four spaces.
- Keep readable logic in its owning function. Do not create tiny helpers just
  to move code out of sight.
- Keep the main planning decisions visible in one place. A local helper should
  represent a meaningful stage or share substantial logic. Inline wrappers that
  only forward a call and set one field when they add no useful explanation.
- Use descriptive lower-camel-case names and physical-unit suffixes.
- Spell out names such as `targetVelocity_units_s` and
  `targetAcceleration_units_s2` instead of `tgtVel` and `tgtAcc`.
- Distinguish field names, values, and flags: `derivativeFieldNames`,
  `targetDerivativeValues`, and `goalDerivativeWasSupplied`.
- Name values for their role, such as `stateValue` or `limitValue`, when a
  generic `value` would make the reader work harder.
- Use `planningEnvironment` for prepared obstacles and planning geometry.
  Use `arrivalPlanningProgress` for the current result and attempts passed
  between earliest-arrival stages, with `IsComplete` to say when to stop.
- Do not rename every short name mechanically. Physical names such as
  `initialState`, `goalState`, and a local loop's `axisIndex` are already clear.
- Give complicated conditions a readable name when it clarifies their purpose.
- Align assignments within short related groups, without excessive padding.
- Keep short statements together; wrap long calls at natural argument boundaries.
- Preserve public names and error identifiers. Error text should say what is
  missing or invalid and, when useful, how to correct it.
- Explain the next substantial step before its code. For example: build a
  route, turn it into motion with BMTP, then check the returned motion with the
  independent validator. Explain terms such as a starting curve or a separating
  plane where an associate first needs them.

## Review And Verification

- Work one file at a time. Read the full file, make the changes, review the diff,
  and finish the relevant checks before moving to the next file.
- For every substantial block, ask: "Does an associate understand this?"
  Check whether the names show what each value represents, the comments explain
  why the step is needed, and the next decision is easy to follow. Use a short
  example when it resolves a likely misunderstanding. Do not rush through a
  repository-wide replacement as a substitute for this review.
- Before calling a file reviewed, answer these five questions:
  1. **Will this comment help an associate?** Explain the purpose, physical
     meaning, or reason for a non-obvious choice. Remove comments that only
     repeat the statement below them.
  2. **Am I missing comments that would help an associate?** Look for unexplained
     stage transitions, assumptions, equations, tolerances, and failure paths.
     Add the explanation where the reader first needs it, without commenting
     on every routine line.
  3. **Is the style consistent between files?** Compare related functions so
     the same concept uses consistent names, wording, units, and layout.
     Distinguish different concepts instead of forcing them into one generic name.
  4. **Are any names too abstract for an associate?** Prefer names that identify
     the value's role. Explain necessary engineering terms and avoid vague
     containers or abbreviations that make the reader trace assignments to
     discover what a value means.
  5. **Is the decision flow clear and straightforward?** The reader should be
     able to follow the main cases, understand why planning continues or stops,
     and see how the result is selected. Keep readable decisions together and
     use helpers for meaningful stages or shared logic.
- Preserve behavior, geometry, tolerances, validation strength, and output shapes
  during layout and comment work. Treat input-validation changes as behavior
  changes, even when they improve the error message.
- Review the diff and run `git diff --check` and MATLAB `checkcode`.
- Run focused MATLAB tests when expressions or control flow change. Report
  blocked checks honestly; do not describe unrun tests as passing.
- Track each file in the untracked `output/comment-review/comment-review.xlsx`.
  Use `In progress` while awaiting review and `Done` once the review is complete.
- Keep notes tied to each file path. Preserve the workbook's existing formatting
  and other entries, and keep it out of source control.
