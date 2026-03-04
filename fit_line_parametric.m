function [line_data, metrics] = fit_line_parametric(x, y)
    % Аппроксимация прямой в параметрической форме
    % Вход: x, y - координаты точек
    % Выход: line_data - структура с параметрами линии
    %         metrics - метрики качества аппроксимации
    
    % Преобразуем single в double для вычислений
    x = double(x);
    y = double(y);
    
    n = length(x);
    
    % Проверка на достаточное количество точек
    if n < 2
        [line_data, metrics] = create_invalid_result();
        return;
    end
    
    % Вычисление физической длины контура
    physical_length = sum(sqrt(diff(x).^2 + diff(y).^2));
    
    if physical_length < eps(1)
        [line_data, metrics] = create_invalid_result();
        return;
    end
    
    % 1. Вычисление центра масс (точка на прямой)
    center_x = mean(x);
    center_y = mean(y);
    
    % 2. ПРАВИЛЬНОЕ вычисление направляющего вектора через PCA
    points = [x, y];
    centered_points = points - [center_x, center_y];
    
    % Ковариационная матрица
    covariance_matrix = (centered_points' * centered_points) / (n - 1);
    
    % Собственные векторы и значения
    [eigenvectors, eigenvalues] = eig(covariance_matrix);
    
    % Направляющий вектор - собственный вектор с наибольшим собственным значением
    [~, max_idx] = max(diag(eigenvalues));
    direction_vector = eigenvectors(:, max_idx)';
    direction_vector = direction_vector / norm(direction_vector);
    
    % 3. Нормальный вектор (перпендикулярно направлению)
    normal_vector = [-direction_vector(2), direction_vector(1)];
    normal_vector = normal_vector / norm(normal_vector);  % нормализация
    
    % 4. Параметрическое уравнение: P = center + t * direction_vector
    % Проекции всех точек на направляющий вектор
    t_values = (x - center_x) * direction_vector(1) + (y - center_y) * direction_vector(2);
    
    % 5. Находим концы отрезка (проекции крайних точек)
    % Ищем точки, ближайшие к исходным концевым точкам
    t_start = (x(1) - center_x) * direction_vector(1) + (y(1) - center_y) * direction_vector(2);
    t_end = (x(end) - center_x) * direction_vector(1) + (y(end) - center_y) * direction_vector(2);
    
    start_point = [center_x, center_y] + t_start * direction_vector;
    end_point = [center_x, center_y] + t_end * direction_vector;
    
% Согласование направления: вектор должен указывать от начала к концу сегмента
% Определяем фактическое направление сегмента от начала к концу
segment_direction = [x(end) - x(1), y(end) - y(1)];
segment_direction = segment_direction / norm(segment_direction);

    % Сравниваем с направляющим вектором из PCA
    if dot(direction_vector, segment_direction) < 0
        % Если направления противоположны, разворачиваем
        direction_vector = -direction_vector;
        normal_vector = [-direction_vector(2), direction_vector(1)];
    end

    % 6. Вычисление угла с нормализацией к диапазону [-π/2, π/2]
    angle_rad = atan2(direction_vector(2), direction_vector(1));


    fullangle_rad = angle_rad;
    % Нормализация к диапазону [-π/2, π/2]
    if angle_rad > pi/2
        angle_rad = angle_rad - pi;
    elseif angle_rad < -pi/2
        angle_rad = angle_rad + pi;
    end
    
    % 7. Вычисление метрик
    distances = compute_distances(x, y, center_x, center_y, normal_vector);
    avg_distance = mean(distances);
    max_distance = max(distances);
    
    % 8. Метрика прямолинейности (1 - идеально прямая)
    straightness_metric = 1 - (avg_distance / physical_length);
    straightness_metric = max(0, min(1, straightness_metric)); % Ограничиваем [0, 1]
    
    % 9. Дополнительные метрики
    length_segment = norm(end_point - start_point);
    efficiency_ratio = length_segment / physical_length;
    
    % Упаковываем результаты
    line_data = struct(...
        'center_point', [center_x, center_y], ...
        'direction_vector', direction_vector, ...
        'normal_vector', normal_vector, ...
        'start_point', start_point, ...
        'end_point', end_point, ...
        't_min', min(t_values), ...  % сохраняем для полноты
        't_max', max(t_values), ...  % сохраняем для полноты
        't_start', t_start, ...      % проекция первой точки
        't_end', t_end, ...          % проекция последней точки
        'angle_rad', angle_rad, ...
        'angle_deg', rad2deg(angle_rad), ...
        'fullangle_rad', fullangle_rad...
    );
    
    metrics = struct(...
        'avg_distance', avg_distance, ...
        'max_distance', max_distance, ...
        'straightness_metric', straightness_metric, ...
        'physical_length', physical_length, ...
        'segment_length', length_segment, ...
        'efficiency_ratio', efficiency_ratio, ...
        'distances', distances ...
    );

% ПРОВЕРКА СОВПАДЕНИЯ КОНЦОВ
% Расстояния между точками
% dist_start = norm([x(1), y(1)] - start_point);
% dist_end = norm([x(end), y(end)] - end_point);
% if dist_start > 2 || dist_end > 2
%     fprintf('Расстояние от первой точки до start_point: %.6f\n', dist_start);
%     fprintf('Расстояние от последней точки до end_point: %.6f\n', dist_end);
% end    

end

function distances = compute_distances(x, y, center_x, center_y, normal_vector)
    % Вычисление расстояний от точек до линии
    n_points = length(x);
    distances = zeros(n_points, 1);

    for i = 1:n_points
        % Вектор от центра до точки
        vec_to_point = [x(i) - center_x, y(i) - center_y];
        % Расстояние = |(vec_to_point) · normal_vector|
        distances(i) = abs(dot(vec_to_point, normal_vector));
    end
end

function [line_data, metrics] = create_invalid_result()
    % Создание результата для невалидных случаев
    line_data = struct(...
        'center_point', [0, 0], ...
        'direction_vector', [1, 0], ...
        'normal_vector', [0, 1], ...
        'start_point', [0, 0], ...
        'end_point', [0, 0], ...
        't_min', 0, ...
        't_max', 0, ...
        'angle_rad', 0, ...
        'angle_deg', 0 ...
    );

    metrics = struct(...
        'avg_distance', inf, ...
        'max_distance', inf, ...
        'straightness_metric', 0, ...
        'physical_length', 0, ...
        'segment_length', 0, ...
        'efficiency_ratio', 0, ...
        'distances', [] ...
    );
end

