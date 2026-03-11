function [d, q] = park_transform(alpha, beta, theta)
% Park变换 - MATLAB Function
% 将两相静止坐标系 αβ -> 两相旋转坐标系 dq
%
% 输入:
%   alpha - α轴分量
%   beta  - β轴分量
%   theta - 电角度 (rad)
% 输出:
%   d - d轴分量
%   q - q轴分量
%
% 变换矩阵:
%   | id |   |  cosθ   sinθ | | iα |
%   | iq | = | -sinθ   cosθ | | iβ |
%

    sin_theta = sin(theta);
    cos_theta = cos(theta);

    % id = iα*cosθ + iβ*sinθ
    d = alpha * cos_theta + beta * sin_theta;

    % iq = -iα*sinθ + iβ*cosθ
    q = -alpha * sin_theta + beta * cos_theta;

end
