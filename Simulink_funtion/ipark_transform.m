function [alpha, beta] = ipark_transform(d, q, theta)
% 逆Park变换 - MATLAB Function
% 将两相旋转坐标系 dq -> 两相静止坐标系 αβ
%
% 输入:
%   d     - d轴分量
%   q     - q轴分量
%   theta - 电角度 (rad)
% 输出:
%   alpha - α轴分量
%   beta  - β轴分量
%
% 逆变换矩阵:
%   | iα |   | cosθ  -sinθ | | id |
%   | iβ | = | sinθ   cosθ | | iq |
%

    sin_theta = sin(theta);
    cos_theta = cos(theta);

    % iα = Id*cosθ - Iq*sinθ
    alpha = d * cos_theta - q * sin_theta;

    % iβ = Id*sinθ + Iq*cosθ
    beta = d * sin_theta + q * cos_theta;

end
