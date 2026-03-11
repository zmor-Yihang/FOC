function [alpha, beta] = clark(a, b, c)
% Clark变换 (等幅值) - MATLAB Function
% 将三相静止坐标系 abc -> 两相静止坐标系 αβ
% 采用 a + b + c = 0 的约束，仅需两相电流输入
%
% 输入:
%   a - A相电流 (A)
%   b - B相电流 (A)
% 输出:
%   alpha - α轴分量
%   beta  - β轴分量
%

    one_by_sqrt3 = 1.0 / sqrt(3.0);

    % α = a
    alpha = 1 * (2 * a - b - c) / 3;

    % β = (1/√3)*a + (2/√3)*b
    beta = one_by_sqrt3 * a + 2.0 * one_by_sqrt3 * b;

end
