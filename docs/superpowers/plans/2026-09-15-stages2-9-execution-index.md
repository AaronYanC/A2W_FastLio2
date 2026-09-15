# Stages 2-9 Execution Index

The approved architecture is defined in:

```text
docs/superpowers/specs/2026-09-15-a2w-slam-localization-backend-design.md
```

Execute these plans in order. A later Stage may consume only committed, verified interfaces from
earlier Stages:

- [x] Stage 2: `2026-09-15-stage2-scan-context-top-k.md`
- [x] Stage 3: `2026-09-15-stage3-registration-pipeline.md`
- [x] Stage 4: `2026-09-15-stage4-pose-graph.md`
- [x] Stage 5: `2026-09-15-stage5-mapping-outputs.md`
- [x] Stage 6: `2026-09-15-stage6-map-bundle.md`
- [x] Stage 7: `2026-09-15-stage7-localization.md`
- [x] Stage 8: `2026-09-15-stage8-localization-monitor.md`
- [x] Stage 9: `2026-09-15-stage9-global-relocalization.md`

Every checked Stage requires its own commit to be present on
`origin/feature/slam-localization-backend`, a clean worktree, full build/test evidence, and an empty
protected-path diff. Hardware validation is not part of these checkboxes.

After Stage 9, the repository status line is:

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```
