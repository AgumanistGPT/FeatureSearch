function is_continuing = check_continuation_direction(line1, line2)
%CHECK_CONTINUATION_DIRECTION True when line2 continues line1 direction.

is_continuing = false;

if ~isfield(line1, 'points_x') || ~isfield(line1, 'points_y') || ...
        ~isfield(line2, 'points_x') || ~isfield(line2, 'points_y')
    return;
end

start_point1 = [line1.points_x(1), line1.points_y(1)];
end_point1 = [line1.points_x(end), line1.points_y(end)];

start_point2 = [line2.points_x(1), line2.points_y(1)];
end_point2 = [line2.points_x(end), line2.points_y(end)];

dir1 = end_point1 - start_point1;
dir2 = end_point2 - start_point2;

n1 = norm(dir1);
n2 = norm(dir2);
if n1 <= eps || n2 <= eps
    return;
end

dir1 = dir1 / n1;
dir2 = dir2 / n2;

cos_angle_between = dot(dir1, dir2);
is_continuing = cos_angle_between > 0;

end
