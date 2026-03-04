function merged_line = merge_line_with_intermediates(line1, line2, intermediate_segments)
%MERGE_LINE_WITH_INTERMEDIATES Merge two lines and intermediate segments.

all_points_x = line1.points_x(:);
all_points_y = line1.points_y(:);

for k = 1:numel(intermediate_segments)
    seg = intermediate_segments{k};
    all_points_x = [all_points_x; seg.points_x(:)]; %#ok<AGROW>
    all_points_y = [all_points_y; seg.points_y(:)]; %#ok<AGROW>
end

all_points_x = [all_points_x; line2.points_x(:)];
all_points_y = [all_points_y; line2.points_y(:)];

[line_data, line_metrics] = fit_line_parametric(all_points_x, all_points_y);

dx = line_data.end_point(1) - line_data.start_point(1);
dy = line_data.end_point(2) - line_data.start_point(2);
if abs(dx) < 1e-9
    a_final = sign(dy) * 1e6;
else
    a_final = dy / dx;
end
b_final = line_data.start_point(2) - a_final * line_data.start_point(1);

avg_distance_final = line_metrics.avg_distance;
straightness_metric = line_metrics.straightness_metric;

all_segments = {line1};
for k = 1:numel(intermediate_segments)
    all_segments{end + 1} = intermediate_segments{k}; %#ok<AGROW>
end
all_segments{end + 1} = line2;

merged_line = struct( ...
    'start_idx', line1.start_idx, ...
    'end_idx', line2.end_idx, ...
    'points_x', all_points_x, ...
    'points_y', all_points_y, ...
    'params', struct( ...
        'a', a_final, ...
        'b', b_final, ...
        'avg_distance', avg_distance_final, ...
        'straightness_metric', straightness_metric, ...
        'length', sum(hypot(diff(all_points_x), diff(all_points_y))), ...
        'line_data', line_data, ...
        'line_metrics', line_metrics), ...
    'type', 'line', ...
    'line_segments', {all_segments});

end
