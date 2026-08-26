---
name: engineering-design-report
description: Create or substantially update an evidence-backed Word engineering design report for the Az/El planner when explicitly requested; do not use for ordinary README or inline documentation work.
---

# Engineering Design Report

Use only when explicitly invoked. Create a standalone `.docx` that lets a peer
engineer understand the implemented planner, its data flow, governing
mechanisms, interfaces, validation evidence, and known limitations.

Use the available Word-document workflow for authoring, rendering, and
page-by-page visual verification. Unless the user chooses another location,
write `docs/engineering-design-report.docx` and keep temporary rendering or
diagnostic artifacts outside the repository.

## Establish The Design

Trace the public planner, major stages, inputs, options, result schema, expected
failure path, validation, tests, maintained examples, and current evidence.
Separate facts observed in source, explanations inferred from behavior, and
claims verified by runs. Never invent rationale, equations, options, metrics,
or successful results.

Organize the report around the actual implementation:

1. purpose, scope, and operational boundaries;
2. end-to-end architecture and data products;
3. coordinates, geometry, time, units, and invariants;
4. each implemented planning stage with a useful figure and governing math;
5. independent verification, representative success, and expected failure;
6. design decisions, limitations, interfaces, options, and traceability.

Use actual planner output where practical. Label conceptual schematics clearly.
Every important figure states its source scenario, options, seed, units, and
whether values were postprocessed for presentation.

Run only focused non-destructive checks needed to establish report facts. Mark
economically unverified claims as `Not verified in this report`. This workflow
does not authorize planner refactors, commits, pushes, or publication.

Before delivery, render the DOCX, inspect every page, verify captions,
cross-references, equations, units, implementation names, and evidence, then
report the output path, commands run, unverified scope, and created artifacts.
