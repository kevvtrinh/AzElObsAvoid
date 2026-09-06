# Shortcut comparison input fixtures

These files preserve the exact input generation used for the 2026-09-06 comparison. They are test fixtures, not planner entry points. `buildShortcutCases(root)` writes `root/cases.mat`; it expects the frozen production package, trajectory engine, and example helpers under `root/on/`, with these fixture functions on the MATLAB path. Seeds and request ordering are fixed. The two capture functions contain the unchanged setup sections from the maintained examples at the recorded revision; they do not run the examples' planner/validation/plotting wrappers.

The original frozen trees, on/off worker, watchdog, protocol, hashes, requests, and complete run outputs are retained locally at `tmp/shortcut-benefit-20260906/`. See [the report](../../results/shortcut_benefit_20260906.md) for the comparison and limits.
