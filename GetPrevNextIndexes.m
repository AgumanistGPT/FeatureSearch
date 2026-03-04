function [prev_idx, next_idx] = GetPrevNextIndexes(sIdx, Seg_qnt)
    prev_idx = mod(sIdx - 2, Seg_qnt) + 1;
    next_idx = mod(sIdx, Seg_qnt) + 1;
end

% function [prev_idx, next_idx] =  GetPrevNextIndexes(sIdx, Seg_qnt)
% 
%         prev_idx = sIdx;
%         next_idx = sIdx;
% 
%         isOnBound_1 = sIdx == 1;
%         if sum(isOnBound_1)
%             prev_idx = Seg_qnt;
%             next_idx = sIdx + 1;
%         end
% 
%         isOnBound_end = sIdx == Seg_qnt;
% 
%         if sum(isOnBound_end)
%             prev_idx(isOnBound_end) = sIdx(isOnBound_end) - 1;
%             next_idx(isOnBound_end) = 1;
%         end
% 
%         blBound = isOnBound_1|isOnBound_end;
% 
%         prev_idx(~blBound) = sIdx(~blBound) - 1;
%         next_idx(~blBound) = sIdx(~blBound) + 1;
% end