function [distance_bw_lines, side_relation, angle_bw_lines, length_ratio, overlay_info] = LineRelations_v2(line1_segment, line2_segment, key_point)
%ANALYZELINERELATIONS Анализирует отношения между двумя линиями и ключевой точкой

    % Инициализация выходных переменных
    distance_bw_lines = 0;
    side_relation = [0, 0];
    angle_bw_lines = 0;
    length_ratio = 1;
    
    % Проверка типов сегментов
    if ~strcmp(line1_segment.type, 'line') || ~strcmp(line2_segment.type, 'line')
        warning('Один или оба сегмента не являются линиями');
        return;
    end
     
    % 1. Определяем side_relation для обеих линий относительно ключевой точки
    side_relation(1) = getLineSide(line1_segment.params.line_data, key_point);
    side_relation(2) = getLineSide(line2_segment.params.line_data, key_point);
    
    % 2. Вычисляем угол между линиями
    angle_bw_lines = angleBetweenLines(line1_segment.params.line_data, line2_segment.params.line_data);
    
    % 3. Вычисляем соотношение длин
    length_ratio = line1_segment.params.line_metrics.segment_length / line2_segment.params.line_metrics.segment_length;

    % 4. Вычисляем все параметры перекрытия
    [ overlay_info ] = calculateLineOverlayWithPoints(line1_segment, line2_segment);

    % 6. Вычисляем расстояния по нормали в ключевых точках
    distance_bw_lines = overlay_info.distance_bw_lines;
end

function [overlay_info] = calculateLineOverlayWithPoints(line1_seg, line2_seg)
% Функция для вычисления перекрытия через проекцию на вектор биссектрисы

    line1_data = line1_seg.params.line_data;
    line2_data = line2_seg.params.line_data;
    
    % Преобразуем в формат отрезков [x1,y1; x2,y2]
    seg1 = [line1_data.start_point; line1_data.end_point];
    seg2 = [line2_data.start_point; line2_data.end_point];
    
    % 1. Вычисляем вектор биссектрисы
    bisector_vector = computeBisectorVector(line1_data, line2_data);
    
    % 2. Находим точку пересечения линий (бесконечных)
    intersection_point = findLineIntersection(line1_data, line2_data);
    
    % Если линии параллельны, используем среднюю точку между серединами
    mid1 = (seg1(1,:) + seg1(2,:)) / 2;
    mid2 = (seg2(1,:) + seg2(2,:)) / 2;
    projection_origin = (mid1 + mid2) / 2;
    
    % 3. Проецируем все точки обоих отрезков на ось биссектрисы
    points1 = [seg1(1,:); seg1(2,:)];
    points2 = [seg2(1,:); seg2(2,:)];
    
    projections1 = projectPointsOnVector(points1, bisector_vector, projection_origin);
    projections2 = projectPointsOnVector(points2, bisector_vector, projection_origin);
    
    % 4. Находим область перекрытия на оси биссектрисы
    min1 = min(projections1);
    max1 = max(projections1);
    min2 = min(projections2);
    max2 = max(projections2);
    
    bi_overlap_start = max(min1, min2);
    bi_overlap_end = min(max1, max2);
    
    has_overlay = bi_overlap_start < bi_overlap_end;
    
    % 5. Вычисляем перекрытие для каждой линии
    if has_overlay
        % Длина перекрытия (одинаковая для обеих линий)
        overlay_length = bi_overlap_end - bi_overlap_start;
        
        % Находим точки перекрытия на исходных линиях
        overlay1 = findOverlapPoints(seg1, bisector_vector, projection_origin, bi_overlap_start, bi_overlap_end);
        overlay2 = findOverlapPoints(seg2, bisector_vector, projection_origin, bi_overlap_start, bi_overlap_end);
        mead_overlay1 = (overlay1(1,:) + overlay1(2,:)) / 2;
        mead_overlay2 = (overlay2(1,:) + overlay2(2,:)) / 2;  
    else
        % Нет перекрытия - вычисляем отрицательное перекрытие
        % Расстояние между проекциями отрезков на биссектрисе
        if max1 < min2
            % Отрезок 1 лежит слева от отрезка 2
            gap_length = min2 - max1;
            overlay1 = [seg1(2,:); seg1(2,:)]; % конечная точка отрезка 1
            overlay2 = [seg2(1,:); seg2(1,:)]; % начальная точка отрезка 2
        else
            % Отрезок 2 лежит слева от отрезка 1
            gap_length = min1 - max2;
            overlay1 = [seg1(1,:); seg1(1,:)]; % начальная точка отрезка 1
            overlay2 = [seg2(2,:); seg2(2,:)]; % конечная точка отрезка 2
        end
        
        overlay_length = -gap_length; % Отрицательное значение - расстояние между линиями
        mead_overlay1 = overlay1(1,:);
        mead_overlay2 = overlay2(1,:);
    end
    
    % 6. Находим середины отрезков
    mid_line1 = (seg1(1,:) + seg1(2,:)) / 2;
    mid_line2 = (seg2(1,:) + seg2(2,:)) / 2;
    
    % 7. Вычисляем коэффициенты перекрытия
    length1 = norm(seg1(2,:) - seg1(1,:));
    length2 = norm(seg2(2,:) - seg2(1,:));
    
    %Коэффициенты перекрытия
    overlay_first = overlay_length / length1;
    overlay_second = overlay_length / length2;
    
    overlay_ratio = [overlay_first, overlay_second];
    
    % 8. Вычисляем расстояние между линиями
    if has_overlay
        % Если есть перекрытие - расстояние между средними точками перекрытия
        distance_bw_lines = norm(mead_overlay1 - mead_overlay2);
    else
        % Если нет перекрытия - сумма расстояний середин отрезков до биссектрисы
        dist_mid1_to_bisector = distancePointToLine(mid_line1, bisector_vector, projection_origin);
        dist_mid2_to_bisector = distancePointToLine(mid_line2, bisector_vector, projection_origin);
        distance_bw_lines = dist_mid1_to_bisector + dist_mid2_to_bisector;
    end

    % Собираем информацию о перекрытии
    overlay_info = struct(...
        'has_overlay', has_overlay, ...
        'overlay1_start', overlay1(1,:), ...
        'overlay1_mead', mead_overlay1, ...        
        'overlay1_end', overlay1(2,:), ...
        'mid_line1', mid_line1, ...        
        'overlay2_start', overlay2(1,:), ...
        'overlay2_mead', mead_overlay2, ...        
        'overlay2_end', overlay2(2,:), ...
        'mid_line2', mid_line2, ...    
        'distance_bw_lines', distance_bw_lines,...
        'bisector_vector', bisector_vector, ...
        'projection_origin', projection_origin, ...
        'intersection_point', intersection_point, ...
        'bi_overlay_start', bi_overlap_start, ...
        'bi_overlay_end', bi_overlap_end, ...
        'bi_overlay_length', overlay_length, ...
        'overlay_ratio', overlay_ratio);
end

function distance = distancePointToLine(point, direction_vector, origin)
% Вычисляет расстояние от точки до линии (биссектрисы)
% по нормали к биссектрисе

    % Вектор от начала к точке
    vec_to_point = point - origin;
    
    % Проекция на биссектрису
    projection_length = dot(vec_to_point, direction_vector);
    
    % Координата проекции
    projection_point = origin + projection_length * direction_vector;
    
    % Расстояние по нормали
    distance = norm(point - projection_point);
end

% function [overlay_info] = calculateLineOverlayWithPoints(line1_seg, line2_seg)
% % Функция для вычисления перекрытия через проекцию на вектор биссектрисы
% 
%     line1_data = line1_seg.params.line_data;
%     line2_data = line2_seg.params.line_data;
% 
%     % Преобразуем в формат отрезков [x1,y1; x2,y2]
%     seg1 = [line1_data.start_point; line1_data.end_point];
%     seg2 = [line2_data.start_point; line2_data.end_point];
% 
%     % 1. Вычисляем вектор биссектрисы
%     bisector_vector = computeBisectorVector(line1_data, line2_data);
% 
%     % 2. Находим точку пересечения линий (бесконечных)
%     intersection_point = findLineIntersection(line1_data, line2_data);
% 
%     % Если линии параллельны, используем среднюю точку между серединами
%     mid1 = (seg1(1,:) + seg1(2,:)) / 2;
%     mid2 = (seg2(1,:) + seg2(2,:)) / 2;
%     projection_origin = (mid1 + mid2) / 2;
% 
%     % 3. Проецируем все точки обоих отрезков на ось биссектрисы
%     points1 = [seg1(1,:); seg1(2,:)];
%     points2 = [seg2(1,:); seg2(2,:)];
% 
%     projections1 = projectPointsOnVector(points1, bisector_vector, projection_origin);
%     projections2 = projectPointsOnVector(points2, bisector_vector, projection_origin);
% 
%     % 4. Находим область перекрытия на оси биссектрисы
%     min1 = min(projections1);
%     max1 = max(projections1);
%     min2 = min(projections2);
%     max2 = max(projections2);
% 
%     bi_overlap_start = max(min1, min2);
%     bi_overlap_end = min(max1, max2);
% 
%     has_overlay = bi_overlap_start < bi_overlap_end;
% 
%     % 5. Вычисляем перекрытие для каждой линии
%     if has_overlay
%         % Длина перекрытия (одинаковая для обеих линий)
%         overlay_length = bi_overlap_end - bi_overlap_start;
% 
%         % Находим точки перекрытия на исходных линиях
%         overlay1 = findOverlapPoints(seg1, bisector_vector, projection_origin, bi_overlap_start, bi_overlap_end);
%         overlay2 = findOverlapPoints(seg2, bisector_vector, projection_origin, bi_overlap_start, bi_overlap_end);
%         mead_overlay1 = (overlay1(1,:) + overlay1(2,:)) / 2;
%         mead_overlay2 = (overlay2(1,:) + overlay2(2,:)) / 2;  
%     else
%         % Нет перекрытия
%         overlay1 = [0, 0; 0, 0];
%         overlay2 = [0, 0; 0, 0];
%         overlay_length = 0;
%         mead_overlay1 = [0, 0];
%         mead_overlay2 = [0, 0];
%     end
% 
%     % 7. Находим середины отрезков
%     mid_line1 = (seg1(1,:) + seg1(2,:)) / 2;
%     mid_line2 = (seg2(1,:) + seg2(2,:)) / 2;
% 
% 
% 
%     % 11. Вычисляем коэффициенты перекрытия
%     length1 = norm(seg1(2,:) - seg1(1,:));
%     length2 = norm(seg2(2,:) - seg2(1,:));
% 
%     if has_overlay
%         overlay_first = overlay_length / length1;
%         overlay_second = overlay_length / length2;
%     else
%         overlay_first = 0;
%         overlay_second = 0;
%     end
% 
%     overlay_ratio = [overlay_first, overlay_second];
% 
%     distanse_bw_lines = norm( mead_overlay1 - mead_overlay2);
% 
%     % Собираем информацию о перекрытии
%     overlay_info = struct(...
%         'has_overlay', has_overlay, ...
%         'overlay1_start', overlay1(1,:), ...
%         'overlay1_mead', mead_overlay1, ...        
%         'overlay1_end', overlay1(2,:), ...
%         'mid_line1', mid_line1, ...        
%         'overlay2_start', overlay2(1,:), ...
%         'overlay2_mead', mead_overlay2, ...        
%         'overlay2_end', overlay2(2,:), ...
%         'mid_line2', mid_line2, ...    
%         'distanse_bw_lines', distanse_bw_lines,...
%         'bisector_vector', bisector_vector, ...
%         'projection_origin', projection_origin, ...
%         'intersection_point', intersection_point, ...
%         'bi_overlap_start', bi_overlap_start, ...
%         'bi_boverlap_end', bi_overlap_end, ...
%         'bi_overlay_length', overlay_length, ...
%         'overlay_ratio', overlay_ratio);
% end

function bisector_vector = computeBisectorVector(line1_data, line2_data)
% Вычисляет единичный вектор биссектрисы между двумя линиями

    % Получаем направляющие векторы линий
    dir1 = line1_data.direction_vector;
    dir2 = line2_data.direction_vector;
    
    % Нормализуем векторы
    dir1 = dir1 / norm(dir1);
    dir2 = dir2 / norm(dir2);
    
    % Проверяем угол между векторами
    dot_product = dot(dir1, dir2);
    
    if dot_product < 0
        % Если угол больше 90 градусов, инвертируем один из векторов
        dir2 = -dir2;
    end
    
    % Вычисляем вектор биссектрисы как сумму единичных векторов
    bisector_vector = dir1 + dir2;
    
    % Нормализуем результат
    if norm(bisector_vector) > 1e-10
        bisector_vector = bisector_vector / norm(bisector_vector);
    else
        % Если векторы противоположны после инверсии, берем перпендикуляр
        bisector_vector = [-dir1(2), dir1(1)];
    end
end

function projections = projectPointsOnVector(points, direction_vector, origin)
% Проецирует точки на ось, заданную вектором направления и началом

    n = size(points, 1);
    projections = zeros(n, 1);
    
    for i = 1:n
        % Вектор от начала к точке
        vec_to_point = points(i,:) - origin;
        
        % Проекция на направляющий вектор
        projection = dot(vec_to_point, direction_vector);
        projections(i) = projection;
    end
end

function overlap_points = findOverlapPoints(segment, bisector_vector, origin, overlap_start, overlap_end)
% Находит точки перекрытия на исходном отрезке

    % Проецируем конечные точки отрезка
    points = [segment(1,:); segment(2,:)];
    projections = projectPointsOnVector(points, bisector_vector, origin);
    
    % Находим параметры t для точек перекрытия
    t_start = (overlap_start - projections(1)) / (projections(2) - projections(1));
    t_end = (overlap_end - projections(1)) / (projections(2) - projections(1));
    
    % Ограничиваем параметры границами отрезка
    t_start = max(0, min(1, t_start));
    t_end = max(0, min(1, t_end));
    
    % Вычисляем координаты точек перекрытия
    point_start = segment(1,:) + t_start * (segment(2,:) - segment(1,:));
    point_end = segment(1,:) + t_end * (segment(2,:) - segment(1,:));
    
    overlap_points = [point_start; point_end];
end


function proj = projectSegment(segA, segB)
    % Проекция отрезка segA на отрезок segB
    % segA = [x1,y1; x2,y2] - проецируемый отрезок
    % segB = [x1,y1; x2,y2] - отрезок-основа
    
    % Проекция точек segA на прямую segB
    v = segB(2,:) - segB(1,:);
    t1 = dot(segA(1,:) - segB(1,:), v) / dot(v, v);
    t2 = dot(segA(2,:) - segB(1,:), v) / dot(v, v);
    
    % Ограничиваем проекцию границами отрезка segB
    t1 = max(0, min(1, t1));
    t2 = max(0, min(1, t2));
    
    % Вычисляем координаты проекции
    proj = [segB(1,:) + t1 * v;
            segB(1,:) + t2 * v];
end


function normal_distances = calculateNormalDistances(line1_seg, line2_seg, overlay_info, key_points_data)
% Вычисляет расстояния между линиями по нормали в ключевых точках

    
    normal_distances = struct();
    point_types = fieldnames(key_points_data);
    
    for i = 1:length(point_types)
        point_type = point_types{i};
        point_info = key_points_data.(point_type);
        point = point_info.point;
        
        % Определяем, до какой линии измерять расстояние
        if contains(point_type, 'line1')
            dist = distanceToLine(point, line2_seg);
            target_line = 'line2';
        elseif contains(point_type, 'line2')
            dist = distanceToLine(point, line1_seg);
            target_line = 'line1';
        else
            % Для точек перекрытия используем среднее расстояние
            dist1 = distanceToLine(point, line1_seg);
            dist2 = distanceToLine(point, line2_seg);
            dist = (dist1 + dist2) / 2;
            target_line = 'both';
        end
        
        normal_distances.(point_type) = struct(...
            'point', point, ...
            'distance', dist, ...
            'target_line', target_line);
    end
    
    normal_distances.overlay_info = overlay_info;
end


function [side] = getLineSide(line, point)
    % line - структура с полями:
    %   start_point - массив [x, y] начальной точки линии
    %   end_point - массив [x, y] конечной точки линии  
    %   angle_rad - угол линии в радианах
    % point - массив [x, y] точки для проверки
    
    % Извлекаем координаты из структур
    point_x = point.x;
    point_y = point.y;
    
    start_x = line.start_point(1);
    start_y = line.start_point(2);
    
    end_x = line.end_point(1);
    end_y = line.end_point(2);
    
    line_angle = line.angle_rad;
    
    % Матрица вращения
    R = [cos(-line_angle), -sin(-line_angle); 
         sin(-line_angle),  cos(-line_angle)];
    
    % Поворачиваем точки
    point_rot = R * [point_x; point_y];
    start_rot = R * [start_x; start_y];
    end_rot = R * [end_x; end_y];
    
    % Минимальная и максимальная x-координаты линии после поворота
    min_x = min(start_rot(1), end_rot(1));
    max_x = max(start_rot(1), end_rot(1));
    
    % Определяем сторону
    if point_rot(1) < min_x && point_rot(1) < max_x
        side = -1;      % точка слева от всей линии
    elseif point_rot(1) > max_x && point_rot(1) > min_x
        side = 1;       % точка справа от всей линии
    else
        % Точка внутри проекции линии
        dist_to_min = point_rot(1) - min_x;
        dist_to_max = max_x - point_rot(1);
        if dist_to_min > dist_to_max
            % справа
            side = (dist_to_min/(dist_to_min + dist_to_max));
        else
            % слева
            side = -(dist_to_max/(dist_to_min + dist_to_max));
        end
    end
end

