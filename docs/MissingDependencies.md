# Missing Dependencies

Current implementation of `DetectWingConsoles` is self-contained and does not call missing files.

For a full 1:1 reproduction of all fragment utilities, the following are absent in this repository:

- Нужна функция/файл `check_consol` (нет в проекте). Пользователь добавит её в папку.
- Нужна функция/файл `GetPrevNextIndexes` (нет в проекте). Пользователь добавит её в папку.
- Нужна функция/файл `angle_bw_2lines` (нет в проекте). Пользователь добавит её в папку.
- Нужна функция/файл `check_continuation_direction` (нет в проекте). Пользователь добавит её в папку.
- Нужна функция/файл `fit_line_parametric` (нет в проекте). Пользователь добавит её в папку.
- Нужна функция/файл `merge_line_with_intermediates` (нет в проекте). Пользователь добавит её в папку.

Also, the fragment expects precomputed `PeakIdxes` from an upstream stage. In this repository, curve features are derived directly from positive local curvature maxima.
