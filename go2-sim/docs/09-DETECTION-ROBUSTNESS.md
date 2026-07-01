# ADR-020 — Detection robustness: consistent recall and precision

**Status:** Implemented and validated against ground truth. All decisions are principled sensing/pipeline
changes — no hardcoded positions, no ground truth used in any decision, no threshold fitted to a known
answer.

## Context

The mission detected gauges but did so inconsistently: across runs a gauge would be found once and missed
the next, and a single gauge sometimes produced two markers. Benchmarked against the world's ground-truth
gauge poses, an early run scored **recall 3/4** with **one duplicate**. For a credible inspection system,
every gauge must be found exactly once, and the fix has to be a real improvement to sensing rather than a
threshold tuned to pass a specific scene.

## Root causes (from the run data)

1. **Recall miss — the depth-localization gate was too strict for distant objects.** The missed gauge *was*
   detected and correctly localized (0.04 m from truth), but on only **one** frame, so the persistence
   filter dropped it. A detection is localized only when enough of its bounding-box depth patch is valid;
   the old rule required 30 % of the *patch*. A gauge seen across the room subtends few pixels and a thin
   round dial has sparse, edge-holed depth, so most frames failed a fractional test on a tiny patch. A
   fraction of a small patch is the wrong criterion.
2. **Precision — a localization-noise duplicate.** With the looser gate (below), a few sparse-depth frames
   of one gauge occasionally localize ~0.7–1.1 m off (a noisy median depth), producing a weak second
   detection that escapes the de-duplication radius — a second marker for a gauge that is also strongly
   detected.

## Decisions

### Recall — an absolute valid-pixel floor, plus denser viewpoints
What makes a median depth reliable is an **absolute count** of valid samples, not a fraction of a tiny
patch. The localization gate now requires `valid ≥ max(min_valid_px, min_valid_frac · patch)`:
- `min_valid_px = 12` — the absolute floor, which protects small and distant bounding boxes;
- `min_valid_frac = 0.15` (was 0.30) — still guards a large, mostly-invalid patch.

Viewpoint sampling was densified slightly (`vp_spacing` 3.0 → 2.5 m, `max_viewpoints` 4 → 5) so a
far-corner gauge is also seen from a closer viewpoint — closer means denser depth, hence more localized
observations — without lowering any detection threshold. The previously-missed gauge went from **1
observation to 17**, and recall improved from 3/4 to **4/4**.

### Precision — observation-aware consolidation
A final pass folds a *weak* detection into a *much stronger* same-class detection within a same-object
radius (1.5 m), but only when the weak one was seen in ≤ 50 % of the strong one's frames. The radius is the
primary same-object prior (facility gauges are metres apart); the observation guard ensures two
comparably-observed distinct gauges are never merged, so precision improves without any cost to recall. The
decision is a pure function, `inspect_planner.weak_duplicate_map(objects, radius, obs_frac)`, unit-tested in
`test/test_inspect_planner.py` for the real case and each recall-safety edge.

## Honesty of the evaluation
- Every decision uses only observation counts, geometry, and the depth measurement — never the gauges'
  true positions.
- The valid-pixel thresholds are set from the geometry of distant objects (a small box has few pixels), not
  from a target; the floor is a generic "enough samples for a stable median".
- The consolidation parameters are matched to the measured single-object localization spread and a stated
  domain prior (gauges metres apart), not to a known answer.
- Results are measured by an independent benchmark against the world description (`go2_inspection
  benchmark`); ground truth is used only to score, never to decide.

## Validation

Full mission, benchmarked against ground truth: **recall 4/4** (every gauge — the far-corner gauge now
accumulates 17 observations), **precision 4/4** after consolidation, mean localization error ~0.2 m. The
34-test suite passes.

## Files
`go2_inspection/zone_inspector.py` (localization gate, consolidation, parameters),
`go2_inspection/inspect_planner.py` (`weak_duplicate_map`), `test/test_inspect_planner.py`.
