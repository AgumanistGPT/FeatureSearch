function consoles = DetectWingConsoles(segments, contour, curvature, tangent_angles)
%DETECTWINGCONSOLES Fast detection of wing consoles as line-curve-line triplets.

if nargin < 4
    error('DetectWingConsoles requires 4 inputs: segments, contour, curvature, tangent_angles.');
end
% Signature compatibility.
%#ok<NASGU>

consoles = repmat(struct('lineA_idx', [], 'lineB_idx', [], ...
    'curveFeature', struct('idx', [], 'curve_idx', [], 'x', [], 'y', [])), 0, 1);

if isempty(segments) || isempty(curvature) || ~isstruct(contour)
    return;
end

nContour = numel(curvature);
if nContour < 3
    return;
end

nSeg = numel(segments);
if nSeg == 0
    return;
end

% Rebuild contour topology (order-independent input).
startIdx = zeros(nSeg, 1);
for i = 1:nSeg
    startIdx(i) = normalizeIndex(segments{i}.start_idx, nContour);
end
[~, order] = sortrows([startIdx, (1:nSeg)'], [1, 2]);
sortedSeg = segments(order);
sortedToOrig = order;

% Precompute per-segment geometry for speed.
isLine = false(nSeg, 1);
isCurve = false(nSeg, 1);
lineLen = zeros(nSeg, 1);
lineDir = zeros(nSeg, 2);
lineStart = zeros(nSeg, 2);
lineCenter = zeros(nSeg, 2);

for i = 1:nSeg
    seg = sortedSeg{i};
    t = seg.type;
    if strcmp(t, 'line')
        isLine(i) = true;
        [lineLen(i), lineDir(i, :), lineStart(i, :), lineCenter(i, :)] = extractLineGeom(seg);
    elseif strcmp(t, 'curve')
        isCurve(i) = true;
    end
end

maxLineLen = max(lineLen);
if maxLineLen <= 0
    return;
end

params = struct();
params.shortLineLen = 0.16 * maxLineLen;
params.skipAngleDeg = 25;
params.mergeAngleDeg = 12;
params.consoleAngleDeg = 12;
params.minDistRatio = 0.02;
params.maxDistRatio = 0.30;
params.targetDistRatio = 0.10;
params.minLengthRatio = 0.15;
params.maxLengthRatio = 7.0;

shortLine = isLine & (lineLen <= params.shortLineLen);
peakIdx = findPeakIdxesFast(curvature);
if isempty(peakIdx)
    return;
end

curvePos = find(isCurve);
maxOut = numel(curvePos);
outA = zeros(maxOut, 1);
outB = zeros(maxOut, 1);
outCurvePos = zeros(maxOut, 1);
outFeature = zeros(maxOut, 1);
outCount = 0;

for k = 1:maxOut
    cPos = curvePos(k);
    cSeg = sortedSeg{cPos};

    fIdx = selectCurveFeatureIdx(cSeg, peakIdx, curvature, nContour);
    if fIdx == 0
        continue;
    end

    [l1, l2] = findNearestLines(isLine, cPos, -1);
    [r1, r2] = findNearestLines(isLine, cPos, +1);
    if l1 == 0 || r1 == 0
        continue;
    end

    leftOptions = l1;
    rightOptions = r1;

    % Short-line policy around curve: skip or merge-prefer.
    if shortLine(r1) && r2 > 0
        a = angleDiffDeg(lineDir(l1, :), lineDir(r1, :));
        if a >= params.skipAngleDeg || a <= params.mergeAngleDeg
            rightOptions = r2;
        end
    end
    if shortLine(l1) && l2 > 0
        a = angleDiffDeg(lineDir(l1, :), lineDir(r1, :));
        if a >= params.skipAngleDeg || a <= params.mergeAngleDeg
            leftOptions = l2;
        end
    end

    leftOptions = uniqueNonZero(leftOptions);
    rightOptions = uniqueNonZero(rightOptions);
    if isempty(leftOptions) || isempty(rightOptions)
        continue;
    end

    kp = [contour.x(fIdx), contour.y(fIdx)];
    bestScore = inf;
    bestL = 0;
    bestR = 0;

    for li = 1:numel(leftOptions)
        pL = leftOptions(li);
        for ri = 1:numel(rightOptions)
            pR = rightOptions(ri);
            if pL == pR
                continue;
            end

            [ok, score] = evalPairFast(pL, pR, kp, lineLen, lineDir, lineStart, lineCenter, params);
            if ok && score < bestScore
                bestScore = score;
                bestL = pL;
                bestR = pR;
            end
        end
    end

    if bestL == 0 || bestR == 0
        continue;
    end

    outCount = outCount + 1;
    outA(outCount) = sortedToOrig(bestL);
    outB(outCount) = sortedToOrig(bestR);
    outCurvePos(outCount) = cPos;
    outFeature(outCount) = fIdx;
end

if outCount == 0
    return;
end

keys = [min(outA(1:outCount), outB(1:outCount)), ...
        max(outA(1:outCount), outB(1:outCount)), ...
        outFeature(1:outCount)];
[~, keep] = unique(keys, 'rows', 'stable');

m = numel(keep);
consoles = repmat(struct('lineA_idx', 0, 'lineB_idx', 0, ...
    'curveFeature', struct('idx', 0, 'curve_idx', 0, 'x', 0, 'y', 0)), m, 1);

for i = 1:m
    src = keep(i);
    fIdx = outFeature(src);
    cPos = outCurvePos(src);

    consoles(i).lineA_idx = outA(src);
    consoles(i).lineB_idx = outB(src);
    consoles(i).curveFeature = struct( ...
        'idx', fIdx, ...
        'curve_idx', sortedToOrig(cPos), ...
        'x', contour.x(fIdx), ...
        'y', contour.y(fIdx));
end

end

function idx = normalizeIndex(idx, n)
idx = mod(double(idx) - 1, n) + 1;
end

function [len, dirVec, startPt, centerPt] = extractLineGeom(seg)
len = 0;
dirVec = [1, 0];
startPt = [seg.points_x(1), seg.points_y(1)];
endPt = [seg.points_x(end), seg.points_y(end)];

if isfield(seg, 'params') && isfield(seg.params, 'line_metrics') && ...
        isfield(seg.params.line_metrics, 'segment_length')
    len = double(seg.params.line_metrics.segment_length);
else
    dx = diff(double(seg.points_x(:)));
    dy = diff(double(seg.points_y(:)));
    len = sum(hypot(dx, dy));
end

if isfield(seg, 'params') && isfield(seg.params, 'line_data')
    ld = seg.params.line_data;
    if isfield(ld, 'direction_vector')
        d = double(ld.direction_vector(:))';
        nrm = hypot(d(1), d(2));
        if nrm > 0
            dirVec = d / nrm;
        end
    end
    if isfield(ld, 'start_point')
        startPt = double(ld.start_point(:))';
    end
    if isfield(ld, 'end_point')
        endPt = double(ld.end_point(:))';
    end
else
    d = endPt - startPt;
    nrm = hypot(d(1), d(2));
    if nrm > 0
        dirVec = d / nrm;
    end
end

centerPt = 0.5 * (startPt + endPt);
end

function peakIdx = findPeakIdxesFast(curvature)
curv = double(curvature(:));
n = numel(curv);
if n < 3
    peakIdx = zeros(0, 1);
    return;
end

overlap = min(20, max(1, floor(n / 4)));
peakThreshold = 0.05;
extCurv = [curv(end-overlap+1:end); curv; curv(1:overlap)];

mid = extCurv(2:end-1);
prev = extCurv(1:end-2);
next = extCurv(3:end);

peakPos = find(mid >= prev & mid >= next & mid >= peakThreshold) + 1;
peakNeg = find(mid <= prev & mid <= next & (-mid) >= peakThreshold) + 1;

allPeaks = sort([peakPos(:); peakNeg(:)], 'ascend') - overlap;
allPeaks = allPeaks(allPeaks > 0 & allPeaks <= n);
peakIdx = unique(allPeaks, 'stable');
end

function fIdx = selectCurveFeatureIdx(curveSeg, peakIdx, curvature, nContour)
st = normalizeIndex(curveSeg.start_idx, nContour);
en = normalizeIndex(curveSeg.end_idx, nContour);

if st <= en
    mask = peakIdx >= st & peakIdx <= en;
else
    mask = peakIdx >= st | peakIdx <= en;
end

if ~any(mask)
    fIdx = 0;
    return;
end

localPeaks = peakIdx(mask);
localPeaks = localPeaks(curvature(localPeaks) > 0);
if isempty(localPeaks)
    fIdx = 0;
    return;
end

[~, k] = max(curvature(localPeaks));
fIdx = localPeaks(k);
end

function [near1, near2] = findNearestLines(isLine, pos0, direction)
n = numel(isLine);
near1 = 0;
near2 = 0;
count = 0;
pos = pos0;
for i = 1:n-1
    pos = pos + direction;
    if pos < 1
        pos = n;
    elseif pos > n
        pos = 1;
    end

    if isLine(pos)
        count = count + 1;
        if count == 1
            near1 = pos;
        elseif count == 2
            near2 = pos;
            return;
        end
    end
end
end

function vals = uniqueNonZero(vals)
vals = vals(vals > 0);
if isempty(vals)
    return;
end
vals = unique(vals, 'stable');
end

function d = angleDiffDeg(v1, v2)
c = abs(v1(1) * v2(1) + v1(2) * v2(2));
if c > 1
    c = 1;
end
d = acosd(c);
end

function [ok, score] = evalPairFast(pL, pR, kp, lineLen, lineDir, lineStart, lineCenter, params)
ok = false;
score = inf;

lenL = lineLen(pL);
lenR = lineLen(pR);
if lenL <= 0 || lenR <= 0
    return;
end

vL = lineDir(pL, :);
vR = lineDir(pR, :);

ang = angleDiffDeg(vL, vR);
if ang > params.consoleAngleDeg
    return;
end

vecKL = kp - lineStart(pL, :);
vecKR = kp - lineStart(pR, :);
sideL = sign(vL(1) * vecKL(2) - vL(2) * vecKL(1));
sideR = sign(vR(1) * vecKR(2) - vR(2) * vecKR(1));
if sideL == 0 || sideR == 0 || sideL ~= sideR
    return;
end

vecLC = lineCenter(pL, :) - lineStart(pR, :);
vecRC = lineCenter(pR, :) - lineStart(pL, :);
distLtoR = abs(vR(1) * vecLC(2) - vR(2) * vecLC(1));
distRtoL = abs(vL(1) * vecRC(2) - vL(2) * vecRC(1));
distance = 0.5 * (distLtoR + distRtoL);

maxLen = max(lenL, lenR);
distRatio = distance / maxLen;
if distRatio < params.minDistRatio || distRatio > params.maxDistRatio
    return;
end

lenRatio = lenL / lenR;
if lenRatio < params.minLengthRatio || lenRatio > params.maxLengthRatio
    return;
end

ok = true;
score = abs(distRatio - params.targetDistRatio) + 0.03 * ang + 0.10 * abs(log(max(lenRatio, eps)));
end
