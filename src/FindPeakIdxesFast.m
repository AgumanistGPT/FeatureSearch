function [peakIdx, info] = FindPeakIdxesFast(curvature, overlap, peakThreshold)
%FINDPEAKIDXESFAST Fast curvature peak extraction with wrap-around handling.
% [peakIdx, info] = FindPeakIdxesFast(curvature, overlap, peakThreshold)

if nargin < 2 || isempty(overlap)
    overlap = 20;
end
if nargin < 3 || isempty(peakThreshold)
    peakThreshold = 0.05;
end

curv = double(curvature(:));
n = numel(curv);
if n < 3
    peakIdx = zeros(0, 1);
    info = struct('positiveCount', 0, 'negativeCount', 0, 'overlap', 0, 'threshold', peakThreshold);
    return;
end

overlap = min(max(1, round(overlap)), max(1, floor(n / 3)));
extCurv = [curv(end-overlap+1:end); curv; curv(1:overlap)];

mid = extCurv(2:end-1);
prev = extCurv(1:end-2);
next = extCurv(3:end);

peakPos = find(mid >= prev & mid >= next & mid >= peakThreshold) + 1;
peakNeg = find(mid <= prev & mid <= next & (-mid) >= peakThreshold) + 1;

allPeaks = sort([peakPos(:); peakNeg(:)], 'ascend') - overlap;
allPeaks = allPeaks(allPeaks > 0 & allPeaks <= n);
peakIdx = unique(allPeaks, 'stable');

info = struct( ...
    'positiveCount', numel(peakPos), ...
    'negativeCount', numel(peakNeg), ...
    'overlap', overlap, ...
    'threshold', peakThreshold);

end
