function tests = TestLineRelations_v3
%TESTLINERELATIONS_V3 Functional and performance tests for LineRelations_v3.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(repoRoot);
addpath(fullfile(repoRoot, 'src'));

S = load(fullfile(repoRoot, 'data', 'RealData.mat'));
[pairs, keyPoints] = buildBenchmarkPairs(S.segments, S.contour, S.curvature);

if isempty(pairs)
    error('Failed to build benchmark pairs for LineRelations_v3 tests.');
end

testCase.TestData.repoRoot = repoRoot;
testCase.TestData.S = S;
testCase.TestData.pairs = pairs;
testCase.TestData.keyPoints = keyPoints;
end

function testOutputValidity(testCase)
S = testCase.TestData.S;
pairs = testCase.TestData.pairs;
keyPoints = testCase.TestData.keyPoints;

for i = 1:size(pairs, 1)
    lIdx = pairs(i, 1);
    rIdx = pairs(i, 2);
    key = keyPoints(i);

    [dist, side, angle, lenRatio, overlay] = LineRelations_v3(S.segments{lIdx}, S.segments{rIdx}, key); %#ok<ASGLU>

    verifyTrue(testCase, isfinite(dist));
    verifyTrue(testCase, all(isfinite(side)));
    verifyTrue(testCase, isfinite(angle));
    verifyTrue(testCase, isfinite(lenRatio));

    verifyTrue(testCase, isstruct(overlay));
    verifyTrue(testCase, isfield(overlay, 'distance_bw_lines'));
    verifyTrue(testCase, isfield(overlay, 'overlay_ratio'));
end
end

function testV2V3Consistency(testCase)
S = testCase.TestData.S;
pairs = testCase.TestData.pairs;
keyPoints = testCase.TestData.keyPoints;

for i = 1:size(pairs, 1)
    lIdx = pairs(i, 1);
    rIdx = pairs(i, 2);
    key = keyPoints(i);

    [d2, s2, a2, lr2, ov2] = LineRelations_v2(S.segments{lIdx}, S.segments{rIdx}, key);
    [d3, s3, a3, lr3, ov3] = LineRelations_v3(S.segments{lIdx}, S.segments{rIdx}, key);

    verifyEqual(testCase, d3, d2, 'AbsTol', 1e-12);
    verifyEqual(testCase, s3, s2, 'AbsTol', 1e-12);
    verifyEqual(testCase, a3, a2, 'AbsTol', 1e-12);
    verifyEqual(testCase, lr3, lr2, 'AbsTol', 1e-12);
    verifyEqual(testCase, ov3.distance_bw_lines, ov2.distance_bw_lines, 'AbsTol', 1e-12);
end
end

function testV3Performance(testCase)
S = testCase.TestData.S;
pairs = testCase.TestData.pairs;
keyPoints = testCase.TestData.keyPoints;

f2 = @() runBatchV2(S.segments, pairs, keyPoints, 300);
f3 = @() runBatchV3(S.segments, pairs, keyPoints, 300);

% Use median of multiple runs to reduce timing noise in shared environments.
nRuns = 5;
t2 = zeros(nRuns, 1);
t3 = zeros(nRuns, 1);
for i = 1:nRuns
    t2(i) = timeit(f2);
    t3(i) = timeit(f3);
end

t2m = median(t2);
t3m = median(t3);
verifyLessThanOrEqual(testCase, t3m, t2m * 1.20);
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

if isempty(pairs)
    lineIdx = find(cellfun(@(s) strcmp(s.type, 'line'), segments));
    if numel(lineIdx) >= 2
        pairs = [lineIdx(1), lineIdx(2)];
        keyPoints = struct('x', mean([segments{lineIdx(1)}.points_x(1), segments{lineIdx(2)}.points_x(1)]), ...
            'y', mean([segments{lineIdx(1)}.points_y(1), segments{lineIdx(2)}.points_y(1)]));
    end
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

function acc = runBatchV2(segments, pairs, keyPoints, repeats)
acc = 0;
for r = 1:repeats
    for i = 1:size(pairs, 1)
        [d, s, a, lr] = LineRelations_v2(segments{pairs(i, 1)}, segments{pairs(i, 2)}, keyPoints(i)); %#ok<ASGLU>
        acc = acc + d + a + lr + s(1) + s(2);
    end
end
end

function acc = runBatchV3(segments, pairs, keyPoints, repeats)
acc = 0;
for r = 1:repeats
    for i = 1:size(pairs, 1)
        [d, s, a, lr] = LineRelations_v3(segments{pairs(i, 1)}, segments{pairs(i, 2)}, keyPoints(i)); %#ok<ASGLU>
        acc = acc + d + a + lr + s(1) + s(2);
    end
end
end
