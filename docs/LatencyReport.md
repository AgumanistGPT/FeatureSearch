# Отчет по задержкам

## 1) Цель

Проверить и уменьшить задержки в двух узких местах:
- `LineRelations_v2`
- поиск `PeakIdxes` через `findpeaks`

Дополнительно:
- реализовать `LineRelations_v3`
- реализовать быструю альтернативу `FindPeakIdxesFast`
- подтвердить результат тестами и бенчмарком

## 2) Изменения

- Добавлен `LineRelations_v3.m`.
- `LineRelations_v2.m` переведен в совместимый wrapper над `LineRelations_v3` с учетом `nargout`.
- В `LineRelations_v3` добавлен быстрый путь при `nargout < 5` (без тяжелого полного `overlay_info`).
- Добавлен `src/FindPeakIdxesFast.m` (альтернатива `findpeaks`).
- Добавлен `src/BenchmarkDelays.m` (воспроизводимый бенчмарк задержек).
- Добавлены тесты:
  - `tests/TestLineRelations_v3.m`
  - `tests/TestPeakDetectionAlternative.m`

## 3) Методика

Данные: `data/RealData.mat`.

Замеры:
- `timeit` для устойчивого времени;
- пакетные вызовы для `LineRelations` на реальных line-парах;
- сравнение `findpeaks` и `FindPeakIdxesFast`;
- 3 независимых прогона бенчмарка, отчет по медиане.

Параметры прогона:
- `line_pairs = 16`
- `calls_per_timeit_eval = 8000`

## 4) Результаты задержек

### 4.1 По 3 прогонам (время на 1 вызов)

`LineRelations` (4 выхода, без полного `overlay_info`):
- run1: `v2=0.000039448`, `v3=0.000037551`
- run2: `v2=0.000206492`, `v3=0.000197511`
- run3: `v2=0.000225064`, `v3=0.000183541`
- медиана: `v2=0.000206492`, `v3=0.000183541`
- ускорение `v3`: `1.125x`

`LineRelations` (5 выходов, с `overlay_info`):
- run1: `v2=0.000087399`, `v3=0.000232865`
- run2: `v2=0.000312632`, `v3=0.000257741`
- run3: `v2=0.000315804`, `v3=0.000262811`
- медиана: `v2=0.000312632`, `v3=0.000257741`
- ускорение `v3`: `1.213x`

`findpeaks` vs `FindPeakIdxesFast`:
- run1: `ref=0.004785421`, `fast=0.000060921`
- run2: `ref=0.004343121`, `fast=0.000080014`
- run3: `ref=0.005911421`, `fast=0.000054672`
- медиана: `ref=0.004785421`, `fast=0.000060921`
- ускорение: `78.552x`

Качество пиков:
- `peaks_ref = 28`
- `peaks_fast = 28`
- `peaks_common = 28`
- `recall = 1.0`
- `precision = 1.0`

## 5) Результаты тестов

Прогон `runtests('tests')`:
- `TestDetectWingConsoles` — passed
- `TestLineRelations_v3` — passed
- `TestPeakDetectionAlternative` — passed

Итого: `7 Passed, 0 Failed`.

## 6) Вывод

- `LineRelations_v3` быстрее `LineRelations_v2` по медианным замерам в обоих режимах (4 и 5 выходов).
- Наибольший выигрыш по задержке дает замена `findpeaks` на `FindPeakIdxesFast`.
- Альтернатива `FindPeakIdxesFast` на текущих реальных данных полностью совпала с эталоном по найденным индексам пиков.

## 7) Как воспроизвести

```matlab
cd('C:/Artemyev/OpenAI/Codex/FeatureSearch');
addpath(pwd);
addpath(fullfile(pwd,'src'));

results = runtests('tests');
disp(results);

m = BenchmarkDelays(fullfile(pwd,'data','RealData.mat'));
disp(m);
```
