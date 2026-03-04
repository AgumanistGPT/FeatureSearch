# FeatureSearch

MATLAB project for robust wing-console detection from aircraft contour segments.

## Implemented Function

`src/DetectWingConsoles.m`

```matlab
function consoles = DetectWingConsoles(segments, contour, curvature, tangent_angles)
```

Output format (minimal required fields):
- `console.lineA_idx`
- `console.lineB_idx`
- `console.curveFeature` (`idx`, `curve_idx`, `x`, `y`)

## Detection Logic

- Rebuild contour topology from `start_idx` to avoid dependence on input `segments` order.
- Extract one positive curvature feature point (`curveFeature`) per curve segment.
- Enumerate line-line pairs around each curve feature.
- Apply short-line policy inspired by `FragmentCodeExample.txt`:
  - `SKIP` for short line with big angle (case A)
  - `MERGE` preference for short line with small angle (case B)
- Validate pair by criteria from fragment approach:
  - angle between lines
  - side relation consistency
  - `distance_bw_lines / max(line_length)` in `[0.02, 0.30]`

## Project Layout

- `src/DetectWingConsoles.m`
- `tests/TestDetectWingConsoles.m`
- `data/RealData.mat`
- `data/RealContourExample.fig`
- `docs/Improvements.md`
- `docs/MissingDependencies.md`
- `FragmentCodeExample.txt`
- `LineRelations_v2.m`

## Run Tests

From MATLAB:

```matlab
cd('C:/Artemyev/OpenAI/Codex/FeatureSearch');
addpath(pwd);
addpath(fullfile(pwd, 'src'));
runtests('tests/TestDetectWingConsoles.m');
```

Expected requirement: each test run returns `numel(consoles) >= 4`.
