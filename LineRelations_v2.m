function varargout = LineRelations_v2(line1_segment, line2_segment, key_point)
%LINERELATIONS_V2 Compatibility wrapper over LineRelations_v3.

if nargout == 0
    LineRelations_v3(line1_segment, line2_segment, key_point);
    return;
end

[varargout{1:nargout}] = LineRelations_v3(line1_segment, line2_segment, key_point);

end
