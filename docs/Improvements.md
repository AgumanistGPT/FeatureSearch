# Improvements

## 1) Robustness for cases (A) and (B)

- Add explicit two-sided short-line analysis around each curve feature:
  - Evaluate both `left->right` and `right->left` short-line patterns.
  - Keep a per-candidate reason tag (`ideal`, `skip_short`, `merge_short`) for later filtering.
- Add local contour-length gating:
  - Prefer candidates with shorter geodesic distance along contour between line endpoints.
  - Helps reject accidental long-range line pairings when extra segments exist.
- Use curvature intensity at the feature point in scoring:
  - Stronger peak near curve center should score better.

## 2) Better merge policy (avoid over-merge)

- Require all merge checks simultaneously:
  - small inter-line angle,
  - endpoint gap threshold,
  - continuity direction check,
  - low refit error on merged point range.
- Add guard against cross-wing merge:
  - reject merges if merged span exceeds a max fraction of full contour length.
- Use hysteresis thresholds:
  - strict threshold for first merge,
  - slightly relaxed only if neighboring evidence supports same console.

## 3) Fast line-pair prefilters (without quality loss)

- Filter by rough angle first (`|dAngle| <= 20 deg`) before expensive relation metrics.
- Reject pairs with extreme length ratio early (`<0.1` or `>10`).
- Use cheap bbox overlap or projected distance estimate to skip obviously distant pairs.
- Cache `LineRelations_v2` outputs for repeated pair checks across nearby curve features.

## 4) How to prune extras when consoles > 4

- Cluster candidates by contour position of `curveFeature.idx` and keep best score in each cluster.
- Enforce approximate bilateral symmetry (left/right wing consistency) at post-filter stage.
- Rank by composite score (distance ratio, angle, side consistency, short-line penalties) and keep top-4.
- If tie remains, prefer candidates with non-short endpoints and stronger curvature peaks.
