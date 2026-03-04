function [angle_rad, angle_deg] = angle_bw_2lines(a1, a2)
% Для линий вида y = a*x + b

% Проверка на параллельность (угол равен 0)
if abs(a1 - a2) < 1e-10 % сравнение с учетом погрешности вычислений
    angle_rad = 0;
    angle_deg = 0;
    
% Проверка на перпендикулярность (угол равен 90 градусам)
elseif abs(1 + a1 * a2) < 1e-10
    angle_rad = pi/2;
    angle_deg = 90;
    
% Общий случай
else
    tangent_of_angle = abs((a1 - a2) / (1 + a1 * a2));
    angle_rad = atan(tangent_of_angle);
    angle_deg = rad2deg(angle_rad);
end
end