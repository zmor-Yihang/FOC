function [ia, ib, ic] = iclark_transform(alpha, beta)
% 逆Clark变换 (等幅值) - MATLAB Function
% 将两相静止坐标系 αβ -> 三相静止坐标系 abc
%
% 输入:
%   alpha - α轴分量
%   beta  - β轴分量
% 输出:
%   ia - A相电流 (A)
%   ib - B相电流 (A)
%   ic - C相电流 (A)
%
% 逆变换矩阵 (等幅值):
%   | Ia |   | 1      0     |
%   | Ib | = | -1/2   √3/2  | | iα |
%   | Ic |   | -1/2  -√3/2  | | iβ |
%

    sqrt3_by_2 = sqrt(3.0) / 2.0;

    % Ia = iα
    ia = alpha;

    % Ib = -iα/2 + (√3/2)*iβ
    ib = -0.5 * alpha + sqrt3_by_2 * beta;

    % Ic = -Ia - Ib
    ic = -ia - ib;

end
