function metrics = BenchmarkDelays(dataFile)
%BENCHMARKDELAYS Measure latency of line relations and peak detectors.

if nargin < 1 || isempty(dataFile)
    here = fileparts(mfilename('fullpath'));
    dataFile = fullfile(here, '..', 'data', 'RealData.mat');
end

S = load(dataFile);
segments = S.segments;
contour = S.contour;
curvature = S.curvature(:);

[pairs, keyPoints] = buildBenchmarkPairs(segments, contour, curvature);
if isempty(pairs)
    error('BenchmarkDelays:NoPairs', 'No line pairs were prepared for benchmark.');
end

repeats = 500;
f2 = @() runLineRelationsBatch(2, segments, pairs, keyPoints, repeats, false);
f3 = @() runLineRelationsBatch(3, segments, pairs, keyPoints, repeats, false);
f2full = @() runLineRelationsBatch(2, segments, pairs, keyPoints, repeats, true);
f3full = @() runLineRelationsBatch(3, segments, pairs, keyPoints, repeats, true);

t2 = timeit(f2);
t3 = timeit(f3);
t2full = timeit(f2full);
t3full = timeit(f3full);

callsPerEval = size(pairs, 1) * repeats;

fRef = @() findPeakIdxesReference(curvature, 20, 0.05);
fFast = @() FindPeakIdxesFast(curvature, 20, 0.05);

tRef = timeit(fRef);
tFast = timeit(fFast);

refIdx = fRef();
fastIdx = fFast();
common = numel(intersect(refIdx, fastIdx));

metrics = struct();
metrics.line_pairs = size(pairs, 1);
metrics.calls_per_timeit_eval = callsPerEval;
metrics.line_relations_v2_batch_sec = t2;
metrics.line_relations_v3_batch_sec = t3;
metrics.line_relations_v2_per_call_sec = t2 / callsPerEval;
metrics.line_relations_v3_per_call_sec = t3 / callsPerEval;
metrics.line_relations_speedup = t2 / max(t3, eps);
metrics.line_relations_v2_batch_sec_full = t2full;
metrics.line_relations_v3_batch_sec_full = t3full;
metrics.line_relations_v2_per_call_sec_full = t2full / callsPerEval;
metrics.line_relations_v3_per_call_sec_full = t3full / callsPerEval;
metrics.line_relations_speedup_full = t2full / max(t3full, eps);
metrics.findpeaks_reference_sec = tRef;
metrics.findpeaks_fast_sec = tFast;
metrics.findpeaks_speedup = tRef / max(tFast, eps);
metrics.peaks_ref = numel(refIdx);
metrics.peaks_fast = numel(fastIdx);
metrics.peaks_common = common;
metrics.peaks_recall = common / max(1, numel(refIdx));
metrics.peaks_precision = common / max(1, numel(fastIdx));

end

function [pairs, keyPoints] = buildBenchmarkPairs(segments, contour, curvature)
n = numel(segments);
pairs = zeros(0, 2);
keyPoints = repmat(struct('x', 0, 'y', 0), 0, 1);

for i = 1:n
    if ~strcmp(segments{i}.type, 'curve')
        continue;
    end

    prevIdx = mod(i - 2, n) + 1;
    nextIdx = mod(i, n) + 1;
    if ~strcmp(segments{prevIdx}.type, 'line') || ~strcmp(segments{nextIdx}.type, 'line')
        continue;
    end

    pIdx = pickCurvePeak(segments{i}, curvature);
    if pIdx == 0
        continue;
    end

    pairs(end + 1, :) = [prevIdx, nextIdx]; %#ok<AGROW>
    keyPoints(end + 1, 1) = struct('x', contour.x(pIdx), 'y', contour.y(pIdx)); %#ok<AGROW>
end
end

function pIdx = pickCurvePeak(curveSeg, curvature)
n = numel(curvature);
st = normalizeIndex(curveSeg.start_idx, n);
en = normalizeIndex(curveSeg.end_idx, n);

if st <= en
    idx = st:en;
else
    idx = [st:n, 1:en];
end

idx = idx(curvature(idx) > 0);
if isempty(idx)
    pIdx = 0;
    return;
end

[~, k] = max(curvature(idx));
pIdx = idx(k);
end

function idx = normalizeIndex(idx, n)
idx = mod(double(idx) - 1, n) + 1;
end

function acc = runLineRelationsBatch(version, segments, pairs, keyPoints, repeats, withOverlay)
acc = 0;
for r = 1:repeats
    for i = 1:size(pairs, 1)
        lIdx = pairs(i, 1);
        rIdx = pairs(i, 2);
        key = keyPoints(i);
        if version == 2
            if withOverlay
                [d, s, a, lr, ov] = LineRelations_v2(segments{lIdx}, segments{rIdx}, key); %#ok<ASGLU>
            else
                [d, s, a, lr] = LineRelations_v2(segments{lIdx}, segments{rIdx}, key); %#ok<ASGLU>
            end
        else
            if withOverlay
                [d, s, a, lr, ov] = LineRelations_v3(segments{lIdx}, segments{rIdx}, key); %#ok<ASGLU>
            else
                [d, s, a, lr] = LineRelations_v3(segments{lIdx}, segments{rIdx}, key); %#ok<ASGLU>
            end
        end
        if withOverlay
            acc = acc + d + a + lr + s(1) + s(2) + ov.distance_bw_lines;
        else
            acc = acc + d + a + lr + s(1) + s(2);
        end
    end
end
end

function peakIdx = findPeakIdxesReference(curvature, overlap, peakThreshold)
curv = double(curvature(:));
n = numel(curv);
overlap = min(max(1, round(overlap)), max(1, floor(n / 3)));
extCurv = [curv(end-overlap+1:end); curv; curv(1:overlap)];

[~, peakIdxP] = findpeaks(extCurv, ...
    'MinPeakHeight', peakThreshold, ...
    'MinPeakDistance', 1, ...
    'MinPeakProminence', 0.001);
[~, peakIdxM] = findpeaks(-extCurv, ...
    'MinPeakHeight', peakThreshold, ...
    'MinPeakDistance', 1, ...
    'MinPeakProminence', 0.001);

peakIdx = sort([peakIdxP(:); peakIdxM(:)], 'ascend') - overlap;
peakIdx = peakIdx(peakIdx > 0 & peakIdx <= n);
peakIdx = unique(peakIdx, 'stable');
end
