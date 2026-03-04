function tests = TestPeakDetectionAlternative
%TESTPEAKDETECTIONALTERNATIVE Compare findpeaks and fast alternative.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(repoRoot);
addpath(fullfile(repoRoot, 'src'));

S = load(fullfile(repoRoot, 'data', 'RealData.mat'));
testCase.TestData.curvature = S.curvature(:);
end

function testAlternativeQuality(testCase)
curvature = testCase.TestData.curvature;
[fastIdx, ~] = FindPeakIdxesFast(curvature, 20, 0.05);
refIdx = findPeakIdxesReference(curvature, 20, 0.05);

verifyGreaterThanOrEqual(testCase, numel(fastIdx), 1);
verifyGreaterThanOrEqual(testCase, numel(refIdx), 1);

common = numel(intersect(fastIdx, refIdx));
recall = common / numel(refIdx);
precision = common / numel(fastIdx);

verifyGreaterThanOrEqual(testCase, recall, 0.95);
verifyGreaterThanOrEqual(testCase, precision, 0.95);
end

function testAlternativePerformance(testCase)
curvature = testCase.TestData.curvature;

fRef = @() findPeakIdxesReference(curvature, 20, 0.05);
fFast = @() FindPeakIdxesFast(curvature, 20, 0.05);

tRef = timeit(fRef);
tFast = timeit(fFast);

verifyLessThan(testCase, tFast, tRef);
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
