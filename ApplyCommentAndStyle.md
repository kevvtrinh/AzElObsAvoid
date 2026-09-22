# ApplyCommentAndStyle

Use this guide when reviewing MATLAB files together. Write for a junior or
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
- Use descriptive lower-camel-case names and physical-unit suffixes.
- Spell out names such as `targetVelocity_units_s` and
  `targetAcceleration_units_s2` instead of `tgtVel` and `tgtAcc`.
- Distinguish field names, values, and flags: `derivativeFieldNames`,
  `targetDerivativeValues`, and `goalDerivativeWasSupplied`.
- Name values for their role, such as `stateValue` or `limitValue`, when a
  generic `value` would make the reader work harder.
- Give complicated conditions a readable name when it clarifies their purpose.
- Align assignments within short related groups, without excessive padding.
- Keep short statements together; wrap long calls at natural argument boundaries.
- Preserve public names and error identifiers. Error text should say what is
  missing or invalid and, when useful, how to correct it.

## Review And Verification

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
