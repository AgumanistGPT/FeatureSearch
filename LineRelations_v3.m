function [distance_bw_lines, side_relation, angle_bw_lines, length_ratio, overlay_info] = LineRelations_v3(line1_segment, line2_segment, key_point)
%LINERELATIONS_V3 Fast relation analysis between two line segments.
% [distance_bw_lines, side_relation, angle_bw_lines, length_ratio, overlay_info] = ...
%   LineRelations_v3(line1_segment, line2_segment, key_point)

[dummyDistance, dummySide, dummyAngle, dummyRatio, dummyOverlay] = defaultOutputs();
distance_bw_lines = dummyDistance;
side_relation = dummySide;
angle_bw_lines = dummyAngle;
length_ratio = dummyRatio;
overlay_info = dummyOverlay;

if ~isstruct(line1_segment) || ~isstruct(line2_segment) || ...
        ~isfield(line1_segment, 'type') || ~isfield(line2_segment, 'type') || ...
        ~strcmp(line1_segment.type, 'line') || ~strcmp(line2_segment.type, 'line')
    return;
end

[line1_data, line1_len, ok1] = extractLineData(line1_segment);
[line2_data, line2_len, ok2] = extractLineData(line2_segment);
if ~ok1 || ~ok2 || line1_len <= eps || line2_len <= eps
    return;
end

if ~isstruct(key_point) || ~isfield(key_point, 'x') || ~isfield(key_point, 'y')
    return;
end

side_relation = [getLineSideFast(line1_data, key_point), getLineSideFast(line2_data, key_point)];
angle_bw_lines = angleBetweenLinesFast(line1_data.direction_vector, line2_data.direction_vector);
length_ratio = line1_len / max(line2_len, eps);
if nargout < 5
    distance_bw_lines = estimateDistanceFast(line1_data, line2_data);
    overlay_info = struct('distance_bw_lines', distance_bw_lines, 'overlay_ratio', [0, 0], 'has_overlay', false);
else
    overlay_info = calculateOverlayFast(line1_data, line2_data);
    distance_bw_lines = overlay_info.distance_bw_lines;
end

end

function [distance_bw_lines, side_relation, angle_bw_lines, length_ratio, overlay_info] = defaultOutputs()
distance_bw_lines = 0;
side_relation = [0, 0];
angle_bw_lines = 0;
length_ratio = 1;
overlay_info = struct( ...
    'has_overlay', false, ...
    'overlay1_start', [0, 0], ...
    'overlay1_mead', [0, 0], ...
    'overlay1_end', [0, 0], ...
    'mid_line1', [0, 0], ...
    'overlay2_start', [0, 0], ...
    'overlay2_mead', [0, 0], ...
    'overlay2_end', [0, 0], ...
    'mid_line2', [0, 0], ...
    'distance_bw_lines', 0, ...
    'bisector_vector', [1, 0], ...
    'projection_origin', [0, 0], ...
    'intersection_point', [NaN, NaN], ...
    'bi_overlay_start', 0, ...
    'bi_overlay_end', 0, ...
    'bi_overlay_length', 0, ...
    'overlay_ratio', [0, 0]);
end

function [line_data, seg_len, ok] = extractLineData(segment)
ok = false;
line_data = struct();
seg_len = 0;

if isfield(segment, 'params') && isfield(segment.params, 'line_data')
    line_data = segment.params.line_data;
end

if ~isfield(line_data, 'start_point') || ~isfield(line_data, 'end_point')
    if isfield(segment, 'points_x') && isfield(segment, 'points_y') && ...
            numel(segment.points_x) >= 2 && numel(segment.points_y) >= 2
        line_data.start_point = [double(segment.points_x(1)), double(segment.points_y(1))];
        line_data.end_point = [double(segment.points_x(end)), double(segment.points_y(end))];
    else
        return;
    end
end

v = double(line_data.end_point(:)' - line_data.start_point(:)');
vn = hypot(v(1), v(2));
if vn <= eps
    return;
end

if ~isfield(line_data, 'direction_vector')
    line_data.direction_vector = v / vn;
else
    d = double(line_data.direction_vector(:)');
    dn = hypot(d(1), d(2));
    if dn <= eps
        line_data.direction_vector = v / vn;
    else
        line_data.direction_vector = d / dn;
    end
end

if isfield(segment, 'params') && isfield(segment.params, 'line_metrics') && ...
        isfield(segment.params.line_metrics, 'segment_length')
    seg_len = double(segment.params.line_metrics.segment_length);
else
    seg_len = vn;
end

ok = true;
end

function side = getLineSideFast(line, point)
start_pt = double(line.start_point(:)');
end_pt = double(line.end_point(:)');
pt = [double(point.x), double(point.y)];

dir_vec = end_pt - start_pt;
seg_len = hypot(dir_vec(1), dir_vec(2));
if seg_len <= eps
    side = 0;
    return;
end

dir_vec = dir_vec / seg_len;
t = dot(pt - start_pt, dir_vec);

if t < 0
    side = -1;
elseif t > seg_len
    side = 1;
else
    dist_to_min = t;
    dist_to_max = seg_len - t;
    denom = dist_to_min + dist_to_max + eps;
    if dist_to_min > dist_to_max
        side = dist_to_min / denom;
    else
        side = -dist_to_max / denom;
    end
end
end

function angle_deg = angleBetweenLinesFast(dir1, dir2)
d1 = double(dir1(:)');
d2 = double(dir2(:)');
n1 = hypot(d1(1), d1(2));
n2 = hypot(d2(1), d2(2));
if n1 <= eps || n2 <= eps
    angle_deg = 0;
    return;
end
d1 = d1 / n1;
d2 = d2 / n2;
cos_val = abs(dot(d1, d2));
cos_val = min(1, max(-1, cos_val));
angle_deg = acosd(cos_val);
end

function distance_bw_lines = estimateDistanceFast(line1_data, line2_data)
s1 = double(line1_data.start_point(:)');
e1 = double(line1_data.end_point(:)');
s2 = double(line2_data.start_point(:)');
e2 = double(line2_data.end_point(:)');

d1 = e1 - s1;
d2 = e2 - s2;
n1 = hypot(d1(1), d1(2));
n2 = hypot(d2(1), d2(2));
if n1 <= eps || n2 <= eps
    distance_bw_lines = 0;
    return;
end
d1 = d1 / n1;
d2 = d2 / n2;

c1 = 0.5 * (s1 + e1);
c2 = 0.5 * (s2 + e2);

dist1 = abs(cross2(d2, c1 - s2));
dist2 = abs(cross2(d1, c2 - s1));
distance_bw_lines = 0.5 * (dist1 + dist2);
end

function overlay_info = calculateOverlayFast(line1_data, line2_data)
seg1 = [double(line1_data.start_point(:)'), double(line1_data.end_point(:)')];
seg2 = [double(line2_data.start_point(:)'), double(line2_data.end_point(:)')];

s1 = seg1(1:2);
e1 = seg1(3:4);
s2 = seg2(1:2);
e2 = seg2(3:4);

v1 = e1 - s1;
v2 = e2 - s2;

n1 = hypot(v1(1), v1(2));
n2 = hypot(v2(1), v2(2));
if n1 <= eps || n2 <= eps
    [~, ~, ~, ~, overlay_info] = defaultOutputs();
    return;
end

v1 = v1 / n1;
v2 = v2 / n2;
if dot(v1, v2) < 0
    v2 = -v2;
end

bisector = v1 + v2;
if hypot(bisector(1), bisector(2)) <= 1e-12
    bisector = [-v1(2), v1(1)];
end
bisector = bisector / hypot(bisector(1), bisector(2));

mid1 = 0.5 * (s1 + e1);
mid2 = 0.5 * (s2 + e2);
origin = 0.5 * (mid1 + mid2);

p1 = [dot(s1 - origin, bisector), dot(e1 - origin, bisector)];
p2 = [dot(s2 - origin, bisector), dot(e2 - origin, bisector)];
min1 = min(p1);
max1 = max(p1);
min2 = min(p2);
max2 = max(p2);

ov_start = max(min1, min2);
ov_end = min(max1, max2);
has_overlay = ov_start < ov_end;
ov_len = ov_end - ov_start;

if has_overlay
    ov1 = clipSegmentToProjectionRange(s1, e1, origin, bisector, ov_start, ov_end);
    ov2 = clipSegmentToProjectionRange(s2, e2, origin, bisector, ov_start, ov_end);
    mead1 = 0.5 * (ov1(1, :) + ov1(2, :));
    mead2 = 0.5 * (ov2(1, :) + ov2(2, :));
    dist_bw = hypot(mead1(1) - mead2(1), mead1(2) - mead2(2));
else
    if max1 < min2
        p1_sel = e1;
        p2_sel = s2;
        ov_len = -(min2 - max1);
    else
        p1_sel = s1;
        p2_sel = e2;
        ov_len = -(min1 - max2);
    end
    ov1 = [p1_sel; p1_sel];
    ov2 = [p2_sel; p2_sel];
    mead1 = p1_sel;
    mead2 = p2_sel;
    dist_bw = abs(cross2(bisector, mid1 - origin)) + abs(cross2(bisector, mid2 - origin));
end

ratio = [ov_len / max(n1, eps), ov_len / max(n2, eps)];
intersection_point = lineIntersectionFast(s1, v1, s2, v2);

overlay_info = struct( ...
    'has_overlay', has_overlay, ...
    'overlay1_start', ov1(1, :), ...
    'overlay1_mead', mead1, ...
    'overlay1_end', ov1(2, :), ...
    'mid_line1', mid1, ...
    'overlay2_start', ov2(1, :), ...
    'overlay2_mead', mead2, ...
    'overlay2_end', ov2(2, :), ...
    'mid_line2', mid2, ...
    'distance_bw_lines', dist_bw, ...
    'bisector_vector', bisector, ...
    'projection_origin', origin, ...
    'intersection_point', intersection_point, ...
    'bi_overlay_start', ov_start, ...
    'bi_overlay_end', ov_end, ...
    'bi_overlay_length', ov_len, ...
    'overlay_ratio', ratio);
end

function clipped = clipSegmentToProjectionRange(s, e, origin, axis_dir, p_start, p_end)
proj_s = dot(s - origin, axis_dir);
proj_e = dot(e - origin, axis_dir);
denom = proj_e - proj_s;

if abs(denom) <= eps
    clipped = [s; s];
    return;
end

t1 = (p_start - proj_s) / denom;
t2 = (p_end - proj_s) / denom;
t1 = min(1, max(0, t1));
t2 = min(1, max(0, t2));

pt1 = s + t1 * (e - s);
pt2 = s + t2 * (e - s);
clipped = [pt1; pt2];
end

function value = cross2(a, b)
value = a(1) * b(2) - a(2) * b(1);
end

function p = lineIntersectionFast(s1, d1, s2, d2)
den = cross2(d1, d2);
if abs(den) <= 1e-12
    p = [NaN, NaN];
    return;
end

t = cross2((s2 - s1), d2) / den;
p = s1 + t * d1;
end
