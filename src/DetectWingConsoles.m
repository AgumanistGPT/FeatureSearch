function consoles = DetectWingConsoles(segments, contour, curvature, tangent_angles)
%DETECTWINGCONSOLES Detect wing consoles as line-curveFeature-line triplets.
%   consoles = DetectWingConsoles(segments, contour, curvature, tangent_angles)
%   returns a struct array with fields:
%     - lineA_idx
%     - lineB_idx
%     - curveFeature (struct with idx, curve_idx, x, y)
%
%   The detector is order-independent for input `segments`: topology is
%   reconstructed from contour indexes.

% Keep signature compatibility with existing pipeline.
if nargin < 4
    error('DetectWingConsoles requires 4 inputs: segments, contour, curvature, tangent_angles.');
end

% tangent_angles is intentionally accepted by API but not required here.
%#ok<NASGU>

consoles = emptyConsoles();

if isempty(segments) || isempty(curvature) || ~isstruct(contour)
    return;
end

if exist('LineRelations_v2', 'file') ~= 2
    error('Need function/file LineRelations_v2 (not in project). User will add it to the folder.');
end

contourLen = numel(curvature);
if contourLen < 3
    return;
end

[sortedSegments, sortedToOriginal] = sortSegmentsByTopology(segments, contourLen);
maxLineLength = getMaxLineLength(sortedSegments);
if maxLineLength <= 0
    return;
end

params = getDefaultParams(maxLineLength);
peakIdx = findPositiveCurvaturePeaks(curvature);
if isempty(peakIdx)
    return;
end

for curvePos = 1:numel(sortedSegments)
    curveSeg = sortedSegments{curvePos};
    if ~isCurveSegment(curveSeg)
        continue;
    end

    featureIdx = selectCurveFeaturePoint(curveSeg, peakIdx, curvature, contourLen);
    if isempty(featureIdx)
        continue;
    end

    curveFeature = struct( ...
        'idx', featureIdx, ...
        'curve_idx', sortedToOriginal(curvePos), ...
        'x', contour.x(featureIdx), ...
        'y', contour.y(featureIdx));

    [leftCandidates, rightCandidates] = collectLineCandidates( ...
        sortedSegments, sortedToOriginal, curvePos, params);

    if isempty(leftCandidates) || isempty(rightCandidates)
        continue;
    end

    bestPair = pickBestConsolePair(leftCandidates, rightCandidates, curveFeature, params);
    if isempty(bestPair)
        continue;
    end

    consoles(end + 1, 1).lineA_idx = bestPair.lineA_idx; %#ok<AGROW>
    consoles(end, 1).lineB_idx = bestPair.lineB_idx;
    consoles(end, 1).curveFeature = curveFeature;
end

consoles = uniqueConsoles(consoles);

end

function consoles = emptyConsoles()
consoles = repmat(struct( ...
    'lineA_idx', [], ...
    'lineB_idx', [], ...
    'curveFeature', struct('idx', [], 'curve_idx', [], 'x', [], 'y', [])), 0, 1);
end

function params = getDefaultParams(maxLineLength)
params = struct();
params.maxLineLength = maxLineLength;
params.shortLineRatio = 0.16;
params.skipAngleThresholdDeg = 25;
params.mergeAngleThresholdDeg = 12;
params.consoleAngleThresholdDeg = 12;
params.minDistanceRatio = 0.02;
params.maxDistanceRatio = 0.30;
params.targetDistanceRatio = 0.10;
params.maxCandidatesPerSide = 3;
params.lengthRatioMin = 0.15;
params.lengthRatioMax = 7.0;
end

function [sortedSegments, sortedToOriginal] = sortSegmentsByTopology(segments, contourLen)
n = numel(segments);
anchor = zeros(n, 1);
for i = 1:n
    anchor(i) = normalizeIndex(segments{i}.start_idx, contourLen);
end

sortMatrix = [anchor, (1:n)'];
[~, order] = sortrows(sortMatrix, [1, 2]);
sortedSegments = segments(order);
sortedToOriginal = order;
end

function idx = normalizeIndex(idx, contourLen)
idx = mod(double(idx) - 1, contourLen) + 1;
end

function maxLen = getMaxLineLength(segments)
maxLen = 0;
for i = 1:numel(segments)
    if ~isLineSegment(segments{i})
        continue;
    end
    segLen = getLineLength(segments{i});
    if segLen > maxLen
        maxLen = segLen;
    end
end
end

function tf = isLineSegment(segment)
tf = isfield(segment, 'type') && strcmp(segment.type, 'line');
end

function tf = isCurveSegment(segment)
tf = isfield(segment, 'type') && strcmp(segment.type, 'curve');
end

function segLen = getLineLength(segment)
segLen = 0;
if isfield(segment, 'params') && isfield(segment.params, 'line_metrics') && ...
        isfield(segment.params.line_metrics, 'segment_length')
    segLen = double(segment.params.line_metrics.segment_length);
    return;
end

if isfield(segment, 'points_x') && isfield(segment, 'points_y')
    dx = diff(double(segment.points_x(:)));
    dy = diff(double(segment.points_y(:)));
    segLen = sum(hypot(dx, dy));
end
end

function peakIdx = findPositiveCurvaturePeaks(curvature)
curv = double(curvature(:));
n = numel(curv);
if n < 3
    peakIdx = [];
    return;
end

prev = curv([n; (1:n-1)']);
next = curv([(2:n)'; 1]);
peakIdx = find(curv > 0 & curv >= prev & curv >= next);
end

function featureIdx = selectCurveFeaturePoint(curveSeg, peakIdx, curvature, contourLen)
st = normalizeIndex(curveSeg.start_idx, contourLen);
en = normalizeIndex(curveSeg.end_idx, contourLen);

inMask = false(size(peakIdx));
for i = 1:numel(peakIdx)
    inMask(i) = indexInCircularRange(peakIdx(i), st, en);
end

if ~any(inMask)
    featureIdx = [];
    return;
end

localPeaks = peakIdx(inMask);
[~, bestLocal] = max(curvature(localPeaks));
featureIdx = localPeaks(bestLocal);
end

function tf = indexInCircularRange(idx, st, en)
if st <= en
    tf = idx >= st && idx <= en;
else
    tf = idx >= st || idx <= en;
end
end

function [leftCandidates, rightCandidates] = collectLineCandidates(sortedSegments, sortedToOriginal, curvePos, params)
leftCandidates = collectDirectionalLines(sortedSegments, sortedToOriginal, curvePos, -1, params);
rightCandidates = collectDirectionalLines(sortedSegments, sortedToOriginal, curvePos, +1, params);

if isempty(leftCandidates) || isempty(rightCandidates)
    return;
end

[leftCandidates, rightCandidates] = applyShortLinePolicy(leftCandidates, rightCandidates, params);
leftCandidates = keepUniqueByOriginalIndex(leftCandidates, params.maxCandidatesPerSide);
rightCandidates = keepUniqueByOriginalIndex(rightCandidates, params.maxCandidatesPerSide);
end

function candidates = collectDirectionalLines(sortedSegments, sortedToOriginal, startPos, direction, params)
n = numel(sortedSegments);
candidates = struct('orig_idx', {}, 'sorted_pos', {}, 'segment', {}, 'is_short', {});

pos = startPos;
visited = 0;
while visited < n - 1 && numel(candidates) < params.maxCandidatesPerSide
    pos = pos + direction;
    if pos < 1
        pos = n;
    elseif pos > n
        pos = 1;
    end

    visited = visited + 1;
    seg = sortedSegments{pos};
    if ~isLineSegment(seg)
        continue;
    end

    isShort = getLineLength(seg) <= params.shortLineRatio * params.maxLineLength;
    candidates(end + 1) = struct( ...
        'orig_idx', sortedToOriginal(pos), ...
        'sorted_pos', pos, ...
        'segment', seg, ...
        'is_short', isShort); %#ok<AGROW>
end
end

function [leftCandidates, rightCandidates] = applyShortLinePolicy(leftCandidates, rightCandidates, params)
if isempty(leftCandidates) || isempty(rightCandidates)
    return;
end

leftMain = leftCandidates(1);
rightMain = rightCandidates(1);

if rightMain.is_short && numel(rightCandidates) >= 2
    angleWithLeft = lineAngleDiffDeg(leftMain.segment, rightMain.segment);
    if angleWithLeft >= params.skipAngleThresholdDeg
        % Case (A): good line - curve - short line with big angle -> skip.
        rightCandidates = rightCandidates(2:end);
    elseif angleWithLeft <= params.mergeAngleThresholdDeg
        % Case (B): short line continues good line -> prefer merged continuation.
        rightCandidates = [rightCandidates(2), rightCandidates];
    end
end

if isempty(rightCandidates)
    return;
end

rightMain = rightCandidates(1);
if leftMain.is_short && numel(leftCandidates) >= 2
    angleWithRight = lineAngleDiffDeg(leftMain.segment, rightMain.segment);
    if angleWithRight >= params.skipAngleThresholdDeg
        leftCandidates = leftCandidates(2:end);
    elseif angleWithRight <= params.mergeAngleThresholdDeg
        leftCandidates = [leftCandidates(2), leftCandidates];
    end
end
end

function diffDeg = lineAngleDiffDeg(lineA, lineB)
angleA = readLineAngleDeg(lineA);
angleB = readLineAngleDeg(lineB);
rawDiff = abs(angleA - angleB);
rawDiff = mod(rawDiff, 180);
if rawDiff > 90
    rawDiff = 180 - rawDiff;
end
diffDeg = rawDiff;
end

function angleDeg = readLineAngleDeg(lineSegment)
if isfield(lineSegment, 'params') && isfield(lineSegment.params, 'line_data') && ...
        isfield(lineSegment.params.line_data, 'angle_deg')
    angleDeg = double(lineSegment.params.line_data.angle_deg);
    return;
end

if isfield(lineSegment, 'params') && isfield(lineSegment.params, 'a')
    angleDeg = atan(double(lineSegment.params.a)) * 180 / pi;
    return;
end

angleDeg = 0;
end

function uniqueCandidates = keepUniqueByOriginalIndex(candidates, maxCount)
if isempty(candidates)
    uniqueCandidates = candidates;
    return;
end

used = [];
uniqueCandidates = struct('orig_idx', {}, 'sorted_pos', {}, 'segment', {}, 'is_short', {});
for i = 1:numel(candidates)
    idx = candidates(i).orig_idx;
    if any(used == idx)
        continue;
    end

    uniqueCandidates(end + 1) = candidates(i); %#ok<AGROW>
    used(end + 1) = idx; %#ok<AGROW>
    if numel(uniqueCandidates) >= maxCount
        break;
    end
end
end

function bestPair = pickBestConsolePair(leftCandidates, rightCandidates, curveFeature, params)
bestPair = [];
keyPoint = struct('x', curveFeature.x, 'y', curveFeature.y);

for li = 1:numel(leftCandidates)
    for ri = 1:numel(rightCandidates)
        left = leftCandidates(li);
        right = rightCandidates(ri);

        if left.orig_idx == right.orig_idx
            continue;
        end

        try
            [distanceBwLines, sideRelation, angleBwLines, lengthRatio] = ...
                LineRelations_v2(left.segment, right.segment, keyPoint);
        catch
            continue;
        end

        if ~isfinite(distanceBwLines) || ~isfinite(angleBwLines) || ~isfinite(lengthRatio)
            continue;
        end

        maxSection = max(getLineLength(left.segment), getLineLength(right.segment));
        if maxSection <= 0
            continue;
        end

        distanceRatio = distanceBwLines / maxSection;
        sideMatch = sign(sideRelation(1)) == sign(sideRelation(2)) && sign(sideRelation(1)) ~= 0;
        linesRelation = struct('angle_bw_lines', angleBwLines, 'side_relation', sideRelation);

        if exist('check_consol', 'file') == 2
            isConsolPair = check_consol(params.consoleAngleThresholdDeg, linesRelation);
        else
            isConsolPair = angleBwLines <= params.consoleAngleThresholdDeg && sideMatch;
        end

        if ~isConsolPair || ~sideMatch
            continue;
        end
        if distanceRatio < params.minDistanceRatio || distanceRatio > params.maxDistanceRatio
            continue;
        end
        if lengthRatio < params.lengthRatioMin || lengthRatio > params.lengthRatioMax
            continue;
        end

        shortPenalty = 0.08 * (double(left.is_short) + double(right.is_short));
        score = abs(distanceRatio - params.targetDistanceRatio) + ...
            0.02 * angleBwLines + ...
            0.10 * abs(log(max(lengthRatio, eps))) + ...
            shortPenalty;

        if isempty(bestPair) || score < bestPair.score
            bestPair = struct( ...
                'lineA_idx', left.orig_idx, ...
                'lineB_idx', right.orig_idx, ...
                'score', score);
        end
    end
end
end

function consoles = uniqueConsoles(consoles)
if isempty(consoles)
    return;
end

keys = zeros(numel(consoles), 3);
for i = 1:numel(consoles)
    a = consoles(i).lineA_idx;
    b = consoles(i).lineB_idx;
    keys(i, :) = [min(a, b), max(a, b), consoles(i).curveFeature.idx];
end

[~, keep] = unique(keys, 'rows', 'stable');
consoles = consoles(keep);
end
