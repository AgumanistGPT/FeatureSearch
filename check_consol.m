function result = check_consol(angle_bw_lines_threshold, lines_relation)
%CHECK_CONSOL Validate a line pair as console-like by angle and side relation.

result = false;

if ~isstruct(lines_relation)
    return;
end
if ~isfield(lines_relation, 'angle_bw_lines') || ~isfield(lines_relation, 'side_relation')
    return;
end

side_relation = lines_relation.side_relation;
if numel(side_relation) < 2
    return;
end

result = lines_relation.angle_bw_lines <= angle_bw_lines_threshold && ...
    sign(side_relation(1)) == sign(side_relation(2));

end
