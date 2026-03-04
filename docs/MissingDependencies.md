# Missing Dependencies

Current implementation of `DetectWingConsoles` has all required local dependencies in repository.

No missing function/file is required for the current tests.

Optional note:
- For exact extraction of fragment merge code into a standalone reusable utility, a separate file `merge_lines.m` can be added. Current detector uses local skip/merge policy and does not require `merge_lines.m`.

Also, the fragment expects precomputed `PeakIdxes` from an upstream stage. In this repository, curve features are derived directly from positive local curvature maxima.
